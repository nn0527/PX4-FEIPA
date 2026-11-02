#!/usr/bin/env python3
"""Pilot position/altitude mode -> OFFBOARD -> controlled pilot-mode handover.

The output mux remains the sole MAVROS setpoint publisher. This node requests
its attitude prestream and uses MAVROS only to change flight modes.
"""

import threading
import time
import math
import json

import rospy
from mavros_msgs.msg import EstimatorStatus, OpticalFlowRad, RCIn, State
from mavros_msgs.srv import SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt8
from px4_wall_ceiling_control.adapters.kill_switch import install_kill_guard, kill_clear

from px4_wall_ceiling_control.adapters.odometry import (
    odometry_is_finite,
    odometry_attitude_horizontal_is_finite,
    vertical_speed_enu_mps,
)
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion
from px4_wall_ceiling_control.freshness import is_fresh_receipt as is_fresh, is_fresh_source
from px4_wall_ceiling_control.msg import (
    CeilingAttachmentStatus,
    OperatorCommand,
    SensorHealth,
    WallPerchStatus,
    WallSensorFusionStatus,
)


from px4_wall_ceiling_control.adapters.rc_takeover import RCTakeoverManager


class FlightModeManagerNode(RCTakeoverManager):
    IDLE = "IDLE"
    WAITING_WALL = "WAITING_WALL"
    PRESTREAM = "PRESTREAM"
    ENTERING = "ENTERING"
    CANCELING_ENTRY = "CANCELING_ENTRY"
    ACTIVE = "ACTIVE"
    RETURNING = "RETURNING"
    LOCKED = "LOCKED"

    def __init__(self):
        install_kill_guard(self, rospy, String)
        self.enabled = bool(rospy.get_param("~enabled", False))
        self.prestream_s = max(1.2, float(rospy.get_param("~prestream_s", 1.5)))
        self.retry_s = max(1.0, float(rospy.get_param("~retry_s", 2.0)))
        self.enter_timeout_s = max(2.0, float(rospy.get_param("~enter_timeout_s", 6.0)))
        self.entry_stream_dropout_grace_s = float(rospy.get_param("~entry_stream_dropout_grace_s", 0.12))
        if not 0.0 <= self.entry_stream_dropout_grace_s <= 0.15:
            raise ValueError("entry dropout grace must be between 0 and 0.15 seconds")
        self.entry_stream_loss_since = None
        self.canceled_entry_mode = None
        self.cancel_entry_started = None
        self.cancel_entry_pilot_samples = 0
        self.cancel_entry_stable_since = None
        self.last_offboard_finished_at = None
        self.offboard_exit_cleanup = False
        self.pilot_intent_request_at = None
        self.acquire_timeout_s = max(2.0, float(rospy.get_param("~acquire_timeout_s", 6.0)))
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 1.5))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.5))
        self.estimator_timeout_s = float(rospy.get_param("~estimator_timeout_s", 1.5))
        self.ceiling_failsafe_guard_required = bool(rospy.get_param('~ceiling_failsafe_guard_required', False))
        self.ceiling_failsafe_unverified_warning_only = bool(rospy.get_param('~ceiling_failsafe_unverified_warning_only', False))
        self.ceiling_failsafe_ready = False
        self.ceiling_failsafe_at = None
        rospy.Subscriber('/attachment/ceiling_failsafe_ready', Bool, self._on_ceiling_failsafe, queue_size=1)
        self.operator_timeout_s = float(rospy.get_param("~operator_timeout_s", 0.5))
        self.sensor_timeout_s = float(rospy.get_param("~sensor_timeout_s", 0.5))
        self.wall_manual_approach = bool(rospy.get_param("~wall_manual_approach", False))
        self.ceiling_baro_handover_enabled = bool(rospy.get_param("~ceiling_height_handover_enabled", rospy.get_param("~ceiling_baro_handover_enabled", False)))
        self.ceiling_motion_enabled = bool(rospy.get_param("/ceiling_attachment_decision_node/motion_control_enabled", False))
        self.ceiling_distance_gate_enabled = bool(rospy.get_param("~ceiling_distance_gate_enabled", False))
        self.ceiling_explicit_detach = bool(rospy.get_param('/ceiling_attachment_decision_node/hold_require_explicit_detach', False))
        self.ceiling_detach_pending_at = None
        self.ceiling_local_land_fault = ''
        self.ceiling_local_land_id = ''
        self.wall_pilot_mode = str(rospy.get_param("~wall_pilot_mode", "ALTCTL")).upper()
        self.wall_horizontal_velocity_enabled = bool(rospy.get_param("/wall_perch_decision_node/horizontal_velocity_enabled",False))
        self.wall_sensor_fusion_enabled = bool(rospy.get_param("/wall_sensor_fusion_manager_node/enabled", False))
        self.wall_fusion_status_snapshot = self.wall_fusion_flow_snapshot = None
        self.wall_fusion_epoch = None
        self.wall_fusion_reason = ''
        self.wall_fusion_since = self.wall_fusion_last_sample = None
        self.wall_fusion_samples = 0
        if self.wall_pilot_mode not in ("ALTCTL","POSCTL"):raise ValueError("unsupported wall pilot mode")
        self.wall_auto_approach = bool(rospy.get_param("~wall_auto_approach", False))
        if self.wall_manual_approach and self.wall_auto_approach:
            raise ValueError("manual and automatic wall approach cannot both be enabled")
        self.manual_wall_wait_timeout_s = max(2.0, float(rospy.get_param("~manual_wall_wait_timeout_s", 60.0)))
        self.manual_wall_wait_started = None
        self.manual_wait_canceled = False
        rospy.Subscriber("/wall_perch/cancel_wait", Bool, self._on_wait_cancel, queue_size=1)
        self.cancel_pub = rospy.Publisher("/attachment/terminal_command", String, queue_size=1)
        self.wall_bench_wait_for_ranges = bool(rospy.get_param("~wall_bench_wait_for_ranges", False))
        self.ignore_range_loss_during_attachment = bool(rospy.get_param(
            "~ignore_range_loss_during_attachment",
            rospy.get_param("/wall_perch_decision_node/ignore_range_loss_during_attachment", False)))
        self.output_timeout_s = float(rospy.get_param("~output_timeout_s", 0.3))
        self.status_timeout_s = float(rospy.get_param("~status_timeout_s", 0.5))
        self.require_rc_center = bool(rospy.get_param("~require_rc_center", False))
        self.rc_roll_channel_index = int(rospy.get_param("~rc_roll_channel_index", -1))
        self.rc_pitch_channel_index = int(rospy.get_param("~rc_pitch_channel_index", -1))
        self.rc_throttle_channel_index = int(
            rospy.get_param("~rc_throttle_channel_index", -1)
        )
        self.rc_pwm_center = int(rospy.get_param("~rc_pwm_center", 1500))
        self.rc_center_tolerance_pwm = int(
            rospy.get_param("~rc_center_tolerance_pwm", 70)
        )
        self.rc_timeout_s = float(rospy.get_param("~rc_timeout_s", 0.20))
        self.handover_max_tilt_deg = float(
            rospy.get_param("~handover_max_tilt_deg", 10.0)
        )
        self.handover_max_vertical_speed_mps = float(
            rospy.get_param("~handover_max_vertical_speed_mps", 0.20)
        )
        self.odom_twist_in_child_frame = bool(
            rospy.get_param("~odom_twist_in_child_frame", True)
        )
        if self.require_rc_center and len({
            self.rc_roll_channel_index,
            self.rc_pitch_channel_index,
            self.rc_throttle_channel_index,
        }) != 3:
            raise ValueError("RC handover requires three distinct stick channels")
        if self.require_rc_center and min(
            self.rc_roll_channel_index,
            self.rc_pitch_channel_index,
            self.rc_throttle_channel_index,
        ) < 0:
            raise ValueError("RC handover channels must be calibrated before flight")

        self.phase = self.IDLE
        self.selected_mode = None
        self.phase_since = time.monotonic()
        self.stream_since = None
        self.last_request_at = None
        self.request_in_flight = False
        self.takeover_attempt = 0
        self.offboard_request = None
        self.lock_until_neutral = False
        self.owner_seen = False
        self.detach_seen = False
        self.recovery_seen = False
        self.completed = False
        self.fault_seen = False
        self.wall_fault_return_ready = False
        self.wall_failure_reason = ""
        self.wall_return_episode = False
        self.ceiling_approach_timeout_returning = False
        self.wall_approach_timeout_returning = False
        self.wall_timeout_stable_since = self.wall_timeout_last_odom = None
        self.wall_timeout_safe_samples = 0
        self.flight = None
        self.flight_at = None
        self.odom = None
        self.odom_at = None
        self.rc_channels = None
        self.rc_at = None
        self.estimator = None
        self.estimator_at = None
        self.operator = None
        self.operator_at = None
        self.sensor = None
        self.sensor_at = None
        self.output_mode = None
        self.output_at = None
        self.owner = 0
        self.owner_at = None
        self.ceiling = None
        self.ceiling_at = None
        self.wall = None
        self.wall_at = None

        self.prestream_pub = rospy.Publisher(
            "/attachment/prestream", Bool, queue_size=10
        )
        self.phase_pub = rospy.Publisher(
            "/attachment/flight_phase", String, queue_size=10, latch=True
        )
        self.phase_reason_pub = rospy.Publisher(
            "/attachment/flight_phase_reason", String, queue_size=10, latch=True
        )
        self.phase_reason = "initialized"
        rospy.Subscriber("/mavros/state", State, self._on_flight, queue_size=10)
        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self._on_odom, queue_size=10
        )
        # Intent cleanup needs live pilot input even when stick centering is disabled.
        rospy.Subscriber(
            rospy.get_param("~rc_topic", "/mavros/rc/in"),
            RCIn, self._on_rc, queue_size=1,
        )
        rospy.Subscriber(
            "/mavros/estimator_status", EstimatorStatus,
            self._on_estimator, queue_size=10,
        )
        rospy.Subscriber(
            "/operator/command", OperatorCommand, self._on_operator, queue_size=10
        )
        rospy.Subscriber(
            "/sensor/health", SensorHealth, self._on_sensor, queue_size=10
        )
        rospy.Subscriber(
            "/attachment/output_mode", UInt8, self._on_output, queue_size=10
        )
        rospy.Subscriber(
            "/attachment/output_owner", UInt8, self._on_owner, queue_size=10
        )
        rospy.Subscriber(
            "/ceiling_attachment/status",
            CeilingAttachmentStatus,
            self._on_ceiling,
            queue_size=10,
        )
        rospy.Subscriber(
            "/wall_perch/status", WallPerchStatus, self._on_wall, queue_size=10
        )
        # A disabled manager can still own a journaled restoration after restart.
        rospy.Subscriber('/wall_perch/sensor_fusion_status', WallSensorFusionStatus,
                         self._on_wall_fusion_status, queue_size=1)
        rospy.Subscriber('/mavros/px4flow/raw/optical_flow_rad', OpticalFlowRad,
                         self._on_wall_fusion_flow, queue_size=1)
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.range_handover_pub = rospy.Publisher('/wall_perch/handover_readiness', String, queue_size=10)
        self.range_posctl_request = None
        self._init_rc_takeover_manager()
        self.timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)
        if rospy.get_param("~motor_output_telemetry_enabled", False):
            threading.Thread(target=self._motor_telemetry_loop, daemon=True).start()
        if not self.enabled:
            rospy.logwarn("automatic flight mode handover is disabled")

    def _request_motor_telemetry(self):
        # Run only from the telemetry worker, never block the control timer.
        def disarmed():
            flight = self.flight
            return (flight is not None and is_fresh(self.flight_at, self.state_timeout_s, rospy.Time.now())
                    and flight.connected and not flight.armed)
        if not disarmed():
            return False
        from mavros_msgs.srv import MessageInterval
        rospy.wait_for_service('/mavros/set_message_interval', timeout=2)
        if not disarmed():
            return False
        result = rospy.ServiceProxy('/mavros/set_message_interval', MessageInterval)(message_id=36, message_rate=10.0)
        if result.success:
            rospy.loginfo('Requested SERVO_OUTPUT_RAW 10 Hz; reception must be checked in /mavros/rc/out')
        return result.success

    def _motor_telemetry_loop(self):
        # Requests are volatile across FCU reboot. Retry while disarmed;
        # acknowledgement alone does not prove the FCU emits output samples.
        next_attempt = 0.0
        while not rospy.is_shutdown():
            if time.monotonic() >= next_attempt:
                try:
                    if self._request_motor_telemetry():
                        next_attempt = time.monotonic() + 30.0
                except (rospy.ROSException, rospy.ServiceException) as exc:
                    rospy.logwarn_throttle(30, 'Motor output telemetry unavailable: %s', exc)
            time.sleep(2)

    def _on_flight(self, message):
        if (getattr(self, "ceiling_motion_enabled", False)
                and getattr(self, "selected_mode", None) == OperatorCommand.MODE_CEILING):
            now = rospy.Time.now()
            previous = getattr(self, "ceiling_flight_source", None)
            if (not is_fresh_source(message.header.stamp, self.state_timeout_s, now)
                    or (previous is not None and message.header.stamp <= previous)):
                return
            request_at = getattr(self, "ceiling_posctl_request_at", None)
            if message.mode.upper() == 'POSCTL' and request_at is not None and message.header.stamp.to_sec() <= request_at:
                return
            self.ceiling_flight_source = message.header.stamp
        if self._wall_range_mode() and message.connected and message.armed:
            now = rospy.Time.now()
            identity = self._wall_range_identity()
            previous = getattr(self, 'range_flight_source', None)
            if (not is_fresh_source(message.header.stamp, self.state_timeout_s, now)
                    or (previous is not None and previous[0] == identity and message.header.stamp <= previous[1])):
                return
            request = getattr(self, 'range_posctl_request', None)
            if (message.mode.upper() == 'POSCTL' and request is not None and request[0] == identity
                    and message.header.stamp.to_sec() <= request[1]):
                return  # A pre-request cached POSCTL message cannot confirm this RPC.
            self.range_flight_source = (identity, message.header.stamp)
        if message.connected and message.mode.upper() in ("ALTCTL", "POSCTL"):
            self.last_pilot_mode = message.mode.upper()
        self.flight, self.flight_at = message, rospy.Time.now()
        if getattr(self,"wall_approach_timeout_returning",False) and message.mode.upper() != "OFFBOARD":
            self.wall_timeout_stable_since=self.wall_timeout_last_odom=None
            self.wall_timeout_safe_samples=0
        if self.phase == self.CANCELING_ENTRY:
            if message.connected and message.mode.upper() == self._pilot_mode():
                self.cancel_entry_pilot_samples = getattr(self, "cancel_entry_pilot_samples", 0) + 1
            else:
                self.cancel_entry_pilot_samples = 0
                if getattr(self, "offboard_exit_cleanup", False):
                    self.pilot_intent_request_at = None
                    self.cancel_entry_stable_since = None

    def _on_odom(self, message):
        self.odom, self.odom_at = message, rospy.Time.now()

    def _on_rc(self, message):
        self.rc_channels, self.rc_at = tuple(message.channels), rospy.Time.now()

    def _on_estimator(self, message):
        self.estimator, self.estimator_at = message, rospy.Time.now()

    def _on_operator(self, message):
        self.operator, self.operator_at = message, rospy.Time.now()

    def _on_sensor(self, message):
        self.sensor, self.sensor_at = message, rospy.Time.now()

    def _on_output(self, message):
        self.output_mode, self.output_at = int(message.data), rospy.Time.now()
        if self.output_mode == 2:
            self.entry_stream_loss_since = None

    def _on_owner(self, message):
        self.owner, self.owner_at = int(message.data), rospy.Time.now()

    def _on_ceiling(self, message):
        self.ceiling, self.ceiling_at = message, rospy.Time.now()

    def _on_wall(self, message):
        self.wall, self.wall_at = message, rospy.Time.now()

    def _on_wall_fusion_status(self, message):
        self.wall_fusion_status_snapshot = (message, rospy.Time.now())

    def _on_wall_fusion_flow(self, message):
        self.wall_fusion_flow_snapshot = (message, rospy.Time.now())

    def _reset_wall_fusion_gate(self):
        self.wall_fusion_since = self.wall_fusion_last_sample = None
        self.wall_fusion_samples = 0
        # Never reuse pre-restoration stability for a late mode request.
        self.cancel_entry_stable_since = None
        self.wall_timeout_stable_since = self.wall_timeout_last_odom = None
        self.wall_timeout_safe_samples = 0

    def _wall_fusion_required(self):
        snapshot = getattr(self, 'wall_fusion_status_snapshot', None)
        return bool(getattr(self, 'wall_sensor_fusion_enabled', False)
                    or (snapshot is not None
                        and (snapshot[0].enabled or snapshot[0].restore_required)))

    def _wall_fusion_ready(self, now):
        """Require restored parameters and continuous fresh post-buffer feedback."""
        if not self._wall_fusion_required():
            return True
        snapshot = getattr(self, 'wall_fusion_status_snapshot', None)
        status, received = snapshot if snapshot is not None else (None, None)
        valid = bool(status is not None and is_fresh(received, .5, now)
            and is_fresh_source(status.header.stamp, .5, now, future_tolerance_s=0.)
            and status.enabled and status.snapshot_valid and status.parameters_restored
            and status.control_ready and not status.restore_required
            and 0. < status.ready_after.to_sec() <= now.to_sec())
        epoch = (status.episode_id, status.ready_after.to_sec()) if valid else None
        if not valid or epoch != getattr(self, 'wall_fusion_epoch', None):
            self._reset_wall_fusion_gate()
        self.wall_fusion_epoch = epoch
        self.wall_fusion_reason = '等待新鲜的融合参数恢复确认及应用缓冲'
        if not valid:
            return False
        ready = status.ready_after
        for message, at, timeout in (
                (getattr(self, 'odom', None), getattr(self, 'odom_at', None), self.odom_timeout_s),
                (getattr(self, 'estimator', None), getattr(self, 'estimator_at', None), self.estimator_timeout_s)):
            if (message is None or not is_fresh(at, timeout, now) or at <= ready
                    or message.header.stamp <= ready
                    or not is_fresh_source(message.header.stamp, timeout, now)):
                self._reset_wall_fusion_gate()
                self.wall_fusion_reason = '等待参数应用缓冲后的新里程计和估计器样本'
                return False
        flow_snapshot = getattr(self, 'wall_fusion_flow_snapshot', None)
        flow, at = flow_snapshot if flow_snapshot is not None else (None, None)
        if (flow is None or not is_fresh(at, .6, now) or at <= ready
                or not is_fresh_source(flow.header.stamp, .6, now)
                or flow.integration_time_us <= 0 or flow.quality < 1
                or not all(math.isfinite(v) for v in (flow.integrated_x, flow.integrated_y,
                    flow.integrated_xgyro, flow.integrated_ygyro, flow.distance)) or flow.distance <= 0.
                or flow.header.stamp.to_sec() - flow.integration_time_us * 1.e-6 <= ready.to_sec()):
            self._reset_wall_fusion_gate()
            self.wall_fusion_reason = '等待参数应用缓冲后完整采集的新光流样本'
            return False
        sample = flow.header.stamp
        previous = getattr(self, 'wall_fusion_last_sample', None)
        if previous is not None and (sample < previous or (sample - previous).to_sec() > .6):
            self._reset_wall_fusion_gate()
            previous = None
        if previous is None or sample > previous:
            self.wall_fusion_last_sample = sample
            self.wall_fusion_samples = getattr(self, 'wall_fusion_samples', 0) + 1
            if self.wall_fusion_since is None:
                self.wall_fusion_since = sample
        confirmed = bool(self.wall_fusion_samples >= 2 and self.wall_fusion_since is not None
            and (sample - self.wall_fusion_since).to_sec() >= .3)
        self.wall_fusion_reason = '' if confirmed else '等待参数恢复后新光流连续有效0.30秒'
        return confirmed

    def _wall_fusion_handover_ready(self, now):
        snapshot = getattr(self, 'wall_fusion_status_snapshot', None)
        status = snapshot[0] if snapshot is not None else None
        debt = bool(status is not None and status.restore_required)
        # A corrupt journal has unknown originals, so the manager advertises
        # enabled/ERROR even though it cannot assert restore_required. Keep
        # that barrier after canceled-entry cleanup clears selected_mode.
        manager_blocked = bool(status is not None and status.enabled
            and (status.phase == 'ERROR' or not status.snapshot_valid
                 or not status.parameters_restored or not status.control_ready))
        if (getattr(self, 'selected_mode', None) == OperatorCommand.MODE_WALL
                or debt or manager_blocked):
            return self._wall_fusion_ready(now)
        return True

    def _set_phase(self, phase, reason=None):
        if self.phase != phase:
            if reason:
                rospy.loginfo(
                    "attachment flight phase: %s -> %s | reason: %s",
                    self.phase,
                    phase,
                    reason,
                )
            else:
                rospy.loginfo("attachment flight phase: %s -> %s", self.phase, phase)
            self.phase = phase
            self.phase_since = time.monotonic()
        if reason:
            self.phase_reason = reason

    def _requested_mode(self, now):
        if not is_fresh(self.operator_at, self.operator_timeout_s, now):
            return None
        if self.operator is None or not self.operator.enable_control:
            return None
        if self.operator.mode == OperatorCommand.MODE_INCLINE:
            return OperatorCommand.MODE_WALL  # Shared physical output owner.
        if self.operator.mode in (
            OperatorCommand.MODE_CEILING,
            OperatorCommand.MODE_WALL,
        ):
            return self.operator.mode
        return None

    def _on_ceiling_failsafe(self, message):
        self.ceiling_failsafe_ready, self.ceiling_failsafe_at = message.data, rospy.Time.now()

    def _preflight_failure_reason(self, now, selected):
        if (selected == OperatorCommand.MODE_CEILING and getattr(self, 'ceiling_failsafe_guard_required', False)
                and not (getattr(self, 'ceiling_failsafe_ready', False)
                         and is_fresh(getattr(self, 'ceiling_failsafe_at', None), 1., now))):
            if getattr(self, 'ceiling_failsafe_unverified_warning_only', False):
                rospy.logwarn_throttle(5., 'PX4内部失联自动降落参数尚未核验：当前配置仅告警，不因此阻止顶吸接管；实际保护参数以飞控读回为准')
            else:
                return '顶吸接管被阻止：PX4内部失联自动降落参数尚未核验（需连接飞控并解除武装）'
        if self._incline_intent():
            wall = getattr(self, 'wall', None)
            if (wall is None or not is_fresh(getattr(self, 'wall_at', None), self.status_timeout_s, now)
                    or not getattr(wall, 'incline_active', False) or not wall.incline_ready):
                return '斜墙未就绪：请启用模式、标定推力上限并启用自动接近/水平反馈'
        if not self.enabled:
            return "flight mode handover disabled"
        if self.flight is None:
            return "flight state unavailable"
        if not is_fresh(self.flight_at, self.state_timeout_s, now):
            return "flight state timeout"
        if not self.flight.connected:
            return "MAVROS disconnected"
        if not self.flight.armed:
            return "vehicle disarmed"
        if self.odom is None:
            return "local odometry unavailable"
        if not is_fresh(self.odom_at, self.odom_timeout_s, now):
            return "local odometry timeout"
        if not odometry_is_finite(self.odom):
            return "local odometry is not finite"
        if self.estimator is None:
            return "estimator status unavailable"
        if not is_fresh(self.estimator_at, self.estimator_timeout_s, now):
            return "estimator status timeout"
        if not (
            self.estimator.attitude_status_flag
            and self.estimator.velocity_vert_status_flag
        ):
            return "attitude or vertical velocity estimate invalid"
        if getattr(self, "require_rc_center", False) and not self._rc_fresh(now):
            return "RC input unavailable or invalid"
        if selected == OperatorCommand.MODE_WALL and not self._wall_fusion_ready(now):
            return self.wall_fusion_reason
        if selected == OperatorCommand.MODE_WALL and not self._wall_horizontal_ready(now):
            detail=(self.wall.horizontal_equilibrium_reason if self.wall is not None
                    and getattr(self.wall,"horizontal_equilibrium_enabled",False) and not self.wall.horizontal_equilibrium_valid
                    else getattr(self.wall,"horizontal_velocity_reason","") if self.wall else "等待贴墙状态")
            return "水平速度接管条件未就绪：" + detail
        if selected == OperatorCommand.MODE_WALL and (
            getattr(self, "wall_bench_wait_for_ranges", False)
            or getattr(self, "ignore_range_loss_during_attachment", False)
        ):
            return None  # Range faults are diagnostic; automatic takeover does not require ranges.
        if selected == OperatorCommand.MODE_CEILING and not self._ceiling_position_ready(now):
            return "吸顶POSCTL接管需要新鲜有效的水平位置及速度估计"
        if selected == OperatorCommand.MODE_CEILING and getattr(self, "ceiling_distance_gate_enabled", False):
            return None  # Decision pauses on range loss; takeover still requires its valid gate.
        if self.sensor is None:
            return "sensor health unavailable"
        if not is_fresh(self.sensor_at, self.sensor_timeout_s, now):
            return "sensor health timeout"
        if selected == OperatorCommand.MODE_CEILING:
            return None if self.sensor.ceiling_valid else "ceiling sensor invalid"
        if selected == OperatorCommand.MODE_WALL:
            if not self.sensor.front_valid:
                return "front sensor invalid"
            return None
        return "unsupported attachment mode"

    def _pilot_mode(self, selected=None):
        if getattr(self, "phase", None) == "RC_TAKEOVER":
            return self.rc_takeover_return_mode
        if self._incline_intent() or getattr(getattr(self, 'wall', None), 'incline_active', False):
            return 'POSCTL'
        selected = getattr(self,"selected_mode",None) if selected is None else selected
        return getattr(self,"wall_pilot_mode","ALTCTL") if selected == OperatorCommand.MODE_WALL else "POSCTL"

    def _wall_horizontal_ready(self, now, stable=False):
        if not getattr(self,"wall_horizontal_velocity_enabled",False):return True
        wall=getattr(self,"wall",None)
        return bool(wall is not None and is_fresh(self.wall_at,self.status_timeout_s,now)
            and getattr(wall,"horizontal_velocity_enabled",False) and wall.horizontal_velocity_ready
            and (not getattr(wall,"horizontal_equilibrium_enabled",False) or wall.horizontal_equilibrium_valid
                 or (not stable and getattr(self,"phase",None) in (self.IDLE,self.WAITING_WALL)))
            and (not stable or wall.horizontal_velocity_stable)
            and self.estimator is not None and is_fresh(self.estimator_at,self.estimator_timeout_s,now)
            and self.estimator.velocity_horiz_status_flag and not self.estimator.const_pos_mode_status_flag
            and (self._pilot_mode() != "POSCTL" or self.estimator.pos_horiz_rel_status_flag or self.estimator.pos_horiz_abs_status_flag))

    def _preflight_ready(self, now, selected):
        return self._preflight_failure_reason(now, selected) is None

    def _on_wait_cancel(self, message):
        if message.data and self.phase == self.WAITING_WALL:
            self.manual_wait_canceled = True

    def _ceiling_position_ready(self, now):
        e = getattr(self, "estimator", None)
        return bool(e is not None and is_fresh(getattr(self, "estimator_at", None), self.estimator_timeout_s, now)
            and e.attitude_status_flag and e.velocity_vert_status_flag and e.velocity_horiz_status_flag
            and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag) and not e.const_pos_mode_status_flag
            and (not getattr(self, "ceiling_motion_enabled", False)
                 or is_fresh_source(e.header.stamp, self.estimator_timeout_s, now)))

    def _ceiling_takeover_ready(self, now):
        if not getattr(self, "ceiling_distance_gate_enabled", False) or self.selected_mode != OperatorCommand.MODE_CEILING:
            return True
        status = self.ceiling
        if not (status is not None and is_fresh(self.ceiling_at,self.status_timeout_s,now) and status.activation_ready):return False
        if getattr(self, "ceiling_motion_enabled", False) and not (
                status.motion_control_enabled and status.motion_ready and status.baseline_ready
                and is_fresh_source(status.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)
                and self._ceiling_position_ready(now)):
            return False
        if self.odom is None or not is_fresh(self.odom_at,self.odom_timeout_s,now) or not odometry_is_finite(self.odom):return False
        speed=vertical_speed_enu_mps(self.odom,self.odom_twist_in_child_frame)
        return math.isfinite(speed)

    def _ceiling_wait_reason(self,now):
        if self.ceiling is None or not is_fresh(self.ceiling_at,self.status_timeout_s,now):
            return "吸顶请求已接收：等待新鲜吸顶状态；仍由飞手控制POSCTL"
        if not self.ceiling.activation_ready:
            return "吸顶请求已接收：等待有效顶部测距及距离准入；仍由飞手控制POSCTL"
        if getattr(self, "ceiling_motion_enabled", False):
            if not self.ceiling.motion_control_enabled or not self.ceiling.motion_ready:
                return "吸顶请求已接收：" + (self.ceiling.motion_reason or "等待有效光流、位置与速度反馈") + "；仍由飞手控制POSCTL"
            if not self.ceiling.baseline_ready:
                return "吸顶请求已接收：等待稳定POSCTL平衡姿态和推力基准；仍由飞手控制POSCTL"
            if not is_fresh_source(self.ceiling.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.):
                return "吸顶请求已接收：吸顶状态源时间过期；仍由飞手控制POSCTL"
            if not self._ceiling_position_ready(now):
                return "吸顶请求已接收：等待有效水平位置与速度估计；仍由飞手控制POSCTL"
        if self.odom is None or not is_fresh(self.odom_at,self.odom_timeout_s,now) or not odometry_is_finite(self.odom):
            return "吸顶请求已接收：等待新鲜有效里程计；仍由飞手控制POSCTL"
        return "吸顶请求已接收：等待有效竖直速度估计；仍由飞手控制POSCTL"

    def _incline_intent(self):
        return getattr(getattr(self, 'operator', None), 'mode', None) == OperatorCommand.MODE_INCLINE

    def _auto_wall_selected(self):
        return (getattr(self, "wall_auto_approach", False) or self._incline_intent()) and self.selected_mode == OperatorCommand.MODE_WALL

    def _gated_wall_selected(self):
        return self._manual_wall_selected() or self._auto_wall_selected()

    def _wall_takeover_ready(self, now):
        if self._incline_intent():
            wall = getattr(self, 'wall', None)
            return bool(wall is not None and is_fresh(getattr(self, 'wall_at', None), self.status_timeout_s, now)
                and is_fresh_source(wall.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)
                and wall.incline_active and wall.incline_ready and wall.state == WallPerchStatus.STATE_IDLE
                and wall.front_ready and wall.front_range_fresh and wall.top_range_fresh
                and wall.horizontal_equilibrium_valid and wall.entry_height_ready)
        if not self._auto_wall_selected():
            return self._manual_wall_ready(now)
        wall = self.wall
        if not (wall is not None and is_fresh(self.wall_at, self.status_timeout_s, now)
                and wall.state == WallPerchStatus.STATE_IDLE
                and (not getattr(wall, "horizontal_equilibrium_enabled", False)
                     or wall.horizontal_equilibrium_valid)
                and wall.entry_height_ready):
            return False
        # This bypass applies before OFFBOARD as well as during attachment.
        # Do not manufacture front_ready: valid distance still gates the flip.
        if getattr(self, "ignore_range_loss_during_attachment", False):
            return True
        return bool(wall.front_ready and wall.front_range_fresh
                    and wall.sensor_health_fresh
                    and self.sensor is not None and is_fresh(self.sensor_at, self.sensor_timeout_s, now)
                    and self.sensor.front_valid)

    def _manual_wall_selected(self):
        return not self._incline_intent() and getattr(self, "wall_manual_approach", False) and self.selected_mode == OperatorCommand.MODE_WALL

    def _manual_wall_ready(self, now):
        wall = self.wall
        return bool(wall is not None and is_fresh(self.wall_at, self.status_timeout_s, now)
                    and wall.state == WallPerchStatus.STATE_IDLE
                    and wall.flip_ready and wall.front_range_fresh
                    and (not getattr(wall,"horizontal_equilibrium_enabled",False) or wall.horizontal_equilibrium_valid)
                    and wall.entry_height_ready
                    and self.sensor is not None and is_fresh(self.sensor_at, self.sensor_timeout_s, now)
                    and self.sensor.front_valid)

    def _manual_wait_elapsed(self, monotonic_now):
        return monotonic_now - self.manual_wall_wait_started

    def _manual_wait_reason(self, now, monotonic_now):
        remaining = max(0.0, self.manual_wall_wait_timeout_s - self._manual_wait_elapsed(monotonic_now))
        if self._auto_wall_selected():
            wall=getattr(self,"wall",None)
            if wall is not None and getattr(wall,"horizontal_equilibrium_enabled",False) and not wall.horizontal_equilibrium_valid:
                return "等待接管平衡基准：" + wall.horizontal_equilibrium_reason
            if getattr(self, "ignore_range_loss_during_attachment", False):
                return "等待自动接近接管：保持 {}；雷达仅告警，等待高度、平衡基准及预发送就绪，剩余 {:.0f} 秒".format(self._pilot_mode(), remaining)
            return "等待自动接近接管：保持 {}；前距≤0.50 m连续0.40 s且两路雷达有效，等待窗口剩余 {:.0f} 秒".format(self._pilot_mode(),remaining)
        return "等待飞手手动接近：保持 {}；需有效距离连续满足翻转阈值，等待窗口剩余 {:.0f} 秒".format(self._pilot_mode(),remaining)

    def _stream_ready(self, now):
        return bool(
            self.output_mode == 2
            and is_fresh(self.output_at, self.output_timeout_s, now)
            and self.owner == 0
            and is_fresh(self.owner_at, self.output_timeout_s, now)
        )

    def _ceiling_entry_owner_expected(self, monotonic_now):
        return bool(
            self.phase == self.ENTERING
            and self.selected_mode == OperatorCommand.MODE_CEILING
            and self.owner == 1  # AttachmentControlCandidate.SOURCE_CEILING
            and self._current_offboard_request(monotonic_now)
        )

    def _entry_stream_ready(self, now, monotonic_now):
        """Accept prestream or this attempt's confirmed ceiling ownership.

        Keep _stream_ready strict: before entry, only the owner-free prestream
        may authorize an OFFBOARD request. Ownership changes during entry are
        a separate handshake, not proof that the publisher has stopped.
        """
        if self._stream_ready(now):
            return True
        if not self._ceiling_entry_owner_expected(monotonic_now):
            return False
        flight, status = self.flight, getattr(self, "ceiling", None)
        return bool(
            self.output_mode == 2
            and is_fresh(self.output_at, self.output_timeout_s, now)
            and is_fresh(self.owner_at, self.output_timeout_s, now)
            and flight is not None and flight.connected and flight.armed
            and flight.mode.upper() == "OFFBOARD"
            and is_fresh(self.flight_at, self.state_timeout_s, now)
            and is_fresh_source(flight.header.stamp, self.state_timeout_s, now, future_tolerance_s=0.)
            and status is not None
            and is_fresh(getattr(self, "ceiling_at", None), self.status_timeout_s, now)
            and is_fresh_source(status.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)
            and status.state in (CeilingAttachmentStatus.STATE_CEILING_ARMED,
                                 CeilingAttachmentStatus.STATE_APPROACH,
                                 CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
                                 CeilingAttachmentStatus.STATE_SURFACE_HOLD)
            and status.enabled and status.candidate_valid and status.keep_ownership
            and not status.fault_detected and not status.detach_failed
            and not status.approach_timed_out and not status.abort_land_requested
        )

    def _entry_stream_failed(self, now, monotonic_now):
        if self._entry_stream_ready(now, monotonic_now):
            self.entry_stream_loss_since = None
            return False
        # Missing/stale output or an unrelated owner is always a hard failure.
        # Only the current ceiling handoff may wait for independently delivered
        # flight/status callbacks, using the existing bounded dropout grace.
        if (not is_fresh(self.output_at, self.output_timeout_s, now)
                or not is_fresh(self.owner_at, self.output_timeout_s, now)
                or (self.owner != 0 and not self._ceiling_entry_owner_expected(monotonic_now))):
            return True
        if getattr(self, "entry_stream_loss_since", None) is None:
            self.entry_stream_loss_since = monotonic_now
        return monotonic_now - self.entry_stream_loss_since >= getattr(self, "entry_stream_dropout_grace_s", .12)

    def _begin_entry_cancel(self, reason, monotonic_now):
        if self.phase != self.CANCELING_ENTRY:
            self.canceled_entry_mode = self.selected_mode
            self.cancel_entry_started = monotonic_now
            self.cancel_entry_pilot_samples = 0
            self.cancel_entry_stable_since = None
            # Invalidate workers which have not submitted yet; retain selected mode
            # and a tombstone for commands already submitted to the FCU.
            self.takeover_attempt = getattr(self, "takeover_attempt", 0) + 1
            self.offboard_request = None
            self._set_phase(self.CANCELING_ENTRY, reason="取消接管处理中：" + reason)
            self.cancel_pub.publish(String(data="cancel"))
        else:
            self.phase_reason = "取消接管处理中：" + reason

    def _begin_offboard_exit(self, reason, monotonic_now):
        self._begin_entry_cancel(reason, monotonic_now)
        self.offboard_exit_cleanup = True
        self.pilot_intent_request_at = None
        self.last_request_at = None
        self.phase_reason = "OFFBOARD异常退出清理：" + reason + "；禁止新动作"

    def _offboard_exit_request_ready(self, now):
        if not self._wall_fusion_handover_ready(now):return False
        flight = self.flight
        if (flight is None or not flight.connected
                or not is_fresh(self.flight_at, self.state_timeout_s, now)):
            return False
        if flight.mode.upper() not in ("OFFBOARD", self._pilot_mode()):
            return False  # Never override a third mode selected by the pilot/failsafe.
        if not flight.armed:
            return flight.mode.upper() == self._pilot_mode()  # Same-mode only.
        channels = getattr(self, "rc_channels", None)
        if (channels is None or len(channels) < 4
                or not is_fresh(self.rc_at, self.rc_timeout_s, now)
                or not all(800 <= v <= 2200 for v in channels[:4])):
            return False
        if flight.mode.upper() == "OFFBOARD":
            return self._entry_cancel_handover_ready(now)
        e = self.estimator
        if (e is None or not is_fresh(self.estimator_at, self.estimator_timeout_s, now)
                or not e.attitude_status_flag or not e.velocity_vert_status_flag):
            return False
        return self._pilot_mode() != "POSCTL" or bool(e.velocity_horiz_status_flag
            and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)
            and not e.const_pos_mode_status_flag)

    def _tick_offboard_exit(self, now, monotonic_now, flight_fresh, flight_mode):
        pilot = self._pilot_mode()
        if not flight_fresh or self.flight is None or not self.flight.connected:
            self.phase_reason = "OFFBOARD异常退出清理：等待新鲜飞控反馈；禁止新动作"
            return
        if flight_mode not in (pilot, "OFFBOARD"):
            self.phase_reason = "OFFBOARD异常退出清理：尊重当前" + flight_mode + "；等待飞手回到" + pilot
            return
        if flight_mode == "OFFBOARD":
            self.cancel_entry_pilot_samples = 0
            if self._offboard_exit_request_ready(now):
                if self.cancel_entry_stable_since is None:
                    self.cancel_entry_stable_since = monotonic_now
                if monotonic_now - self.cancel_entry_stable_since >= .3:
                    self._request_mode(pilot)
            else:
                self.cancel_entry_stable_since = None
            self.phase_reason = "OFFBOARD异常退出清理：旧模式再次生效，等待安全交回" + pilot + "；禁止新动作"
            return
        self.cancel_entry_stable_since = None
        submitted = getattr(self, "pilot_intent_request_at", None)
        if submitted is None:
            if self._offboard_exit_request_ready(now):
                # A failsafe fallback reports POSCTL but may retain OFFBOARD intention.
                self._request_mode(pilot)
            self.phase_reason = "OFFBOARD异常退出清理：飞控为" + pilot + "，等待明确重申该模式以清除旧OFFBOARD意图"
            return
        anchor = max(submitted, self.cancel_entry_started,
                     getattr(self, "last_offboard_finished_at", None) or self.cancel_entry_started)
        if (not getattr(self, "request_in_flight", False)
                and self.cancel_entry_pilot_samples >= 2
                and monotonic_now - anchor >= self.enter_timeout_s):
            self._reset(lock=True, reason="OFFBOARD退出清理完成：已明确请求并连续确认" + pilot + "；释放旧请求后可重试")
        else:
            self.phase_reason = "OFFBOARD异常退出清理：已请求" + pilot + "，等待后续连续模式反馈及在途请求结束；禁止新动作"

    def _entry_cancel_handover_ready(self, now):
        if not self._wall_fusion_handover_ready(now):return False
        # If a behavior acquired ownership meanwhile, retain its full recovery gates.
        if self.owner != 0:
            return self._handover_ready(now)
        if (self.output_mode != 2 or not is_fresh(self.output_at, self.output_timeout_s, now)
                or not is_fresh(self.owner_at, self.output_timeout_s, now)
                or self.flight is None or not self.flight.connected or not self.flight.armed
                or not is_fresh(self.flight_at, self.state_timeout_s, now)
                or self.flight.mode.upper() != "OFFBOARD" or not self._rc_fresh(now)):
            return False
        e, odom = self.estimator, self.odom
        if (e is None or not is_fresh(self.estimator_at, self.estimator_timeout_s, now)
                or not e.attitude_status_flag or not e.velocity_vert_status_flag
                or odom is None or not is_fresh(self.odom_at, self.odom_timeout_s, now)
                or not odometry_is_finite(odom)):
            return False
        if self._pilot_mode() == "POSCTL" and not (e.velocity_horiz_status_flag
                and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)
                and not e.const_pos_mode_status_flag):
            return False
        q = odom.pose.pose.orientation
        roll, pitch, _ = euler_from_quaternion((q.x, q.y, q.z, q.w))
        angular = odom.twist.twist.angular
        return bool(math.cos(roll)*math.cos(pitch) >= math.cos(math.radians(5.))
            and abs(vertical_speed_enu_mps(odom, self.odom_twist_in_child_frame)) <= self.handover_max_vertical_speed_mps
            and all(abs(v) <= .3 for v in (angular.x, angular.y, angular.z)))

    def _tick_entry_cancel(self, now, monotonic_now, flight_fresh, flight_mode):
        if getattr(self, "offboard_exit_cleanup", False):
            return self._tick_offboard_exit(now, monotonic_now, flight_fresh, flight_mode)
        if not flight_fresh or self.flight is None or not self.flight.connected:
            self.phase_reason = "取消接管处理中：等待新鲜飞控模式反馈；禁止新动作"
            return
        if flight_mode == "OFFBOARD":
            self.cancel_entry_pilot_samples = 0
            if self._entry_cancel_handover_ready(now):
                if getattr(self, "cancel_entry_stable_since", None) is None:
                    self.cancel_entry_stable_since = monotonic_now
                if monotonic_now - self.cancel_entry_stable_since >= .3:
                    self._request_mode(self._pilot_mode())
            else:
                self.cancel_entry_stable_since = None
            self.phase_reason = "取消接管处理中：已检测到OFFBOARD，进行有界竖直闭环稳定，等待安全交回" + self._pilot_mode()
        elif flight_mode == self._pilot_mode():
            self.cancel_entry_stable_since = None
            # Two new FCU heartbeats, quiescent service worker and a full entry
            # timeout window prevent a stale pilot-mode sample ending cleanup.
            finished = getattr(self, "last_offboard_finished_at", None)
            anchor = max(self.cancel_entry_started, finished or self.cancel_entry_started)
            if (not getattr(self, "request_in_flight", False)
                    and getattr(self, "cancel_entry_pilot_samples", 0) >= 2
                    and monotonic_now - anchor >= getattr(self, "enter_timeout_s", 6.)):
                self._reset(lock=True, reason="取消接管已完成；飞控确认" + self._pilot_mode() + "，释放旧请求后可重试")
            else:
                self.phase_reason = "取消接管处理中：飞控为" + self._pilot_mode() + "，等待在途请求结束及连续模式确认"
        else:
            # Respect a pilot-selected third mode; keep the tombstone to catch a
            # late OFFBOARD, but never force a mode change from another pilot mode.
            self.phase_reason = "取消接管处理中：飞控为" + flight_mode + "；等待飞手回到" + self._pilot_mode()

    def _rc_fresh(self, now):
        # Frame validity is independent of optional calibrated centering channels.
        channels = getattr(self, "rc_channels", None)
        if (channels is None
                or not is_fresh(getattr(self, "rc_at", None), self.rc_timeout_s, now)):
            return False
        if not getattr(self, "require_rc_center", False):
            return bool(len(channels) >= 4
                        and all(800 <= value <= 2200 for value in channels[:4]))
        indices = (self.rc_roll_channel_index, self.rc_pitch_channel_index,
                   self.rc_throttle_channel_index)
        return bool(len(set(indices)) == 3
                    and all(0 <= index < len(channels) for index in indices)
                    and all(800 <= channels[index] <= 2200 for index in indices))

    def _ceiling_handover_confirmed(self, now):
        status, operator = getattr(self, 'ceiling', None), getattr(self, 'operator', None)
        if status is None or operator is None or not status.recovery_attitude_locked:
            return False
        event, locked = operator.pilot_handover_request_time, status.recovery_lock_time
        return bool(operator.mode == OperatorCommand.MODE_CEILING and operator.enable_control
            and operator.source == 'terminal' and locked.to_sec() > 0
            and locked < event <= operator.header.stamp <= now
            and locked <= status.header.stamp <= now
            and is_fresh(getattr(self, 'operator_at', None), self.operator_timeout_s, now)
            and is_fresh_source(operator.header.stamp, self.operator_timeout_s, now, future_tolerance_s=0.))

    def _ceiling_baro_handover_ready(self, now):
        status = getattr(self, "ceiling", None)
        return bool(getattr(self, "ceiling_baro_handover_enabled", False)
                    and self.selected_mode == OperatorCommand.MODE_CEILING
                    and status is not None and is_fresh(self.ceiling_at, self.status_timeout_s, now)
                    and status.state == CeilingAttachmentStatus.STATE_RECOVERY_HOVER
                    and status.recovery_ready and self._ceiling_handover_confirmed(now)
                    and self._ceiling_position_ready(now)
                    and (not getattr(self, "ceiling_motion_enabled", False) or (
                        status.motion_control_enabled and status.motion_ready and not status.motion_paused
                        and is_fresh_source(status.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)))
                    and status.detach_height_ready
                    and ((status.height_feedback_valid and status.height_reference_ready
                          and self.estimator is not None and self.estimator.pos_vert_abs_status_flag)
                         if status.height_source == 'px4_fused_hold_relative'
                         else status.baro_fresh and status.baro_reference_ready)
                    and not status.detach_failed
                    and not status.fault_detected)

    def _ceiling_approach_timeout_requested(self, now):
        status = getattr(self, "ceiling", None)
        return bool(
            self.selected_mode == OperatorCommand.MODE_CEILING
            and status is not None
            and is_fresh(self.ceiling_at, self.status_timeout_s, now)
            and status.state == CeilingAttachmentStatus.STATE_ABORT
            and not getattr(status, "abort_land_requested", False)
            and (getattr(status, "approach_timed_out", False)
                 or getattr(status, "motion_control_enabled", False))
        )

    def _wall_approach_timeout_requested(self, now):
        wall=getattr(self, "wall", None)
        return bool(self.selected_mode == OperatorCommand.MODE_WALL
            and wall is not None and is_fresh(self.wall_at,self.status_timeout_s,now)
            and wall.state == WallPerchStatus.STATE_ABORT
            and str(wall.reason).startswith("自动接近超时"))

    def _wall_timeout_handover_failure(self, now):
        if not self._wall_fusion_handover_ready(now):return self.wall_fusion_reason
        if not kill_clear(self,now):return "急停生效或反馈无效"
        wall=getattr(self,"wall",None)
        recovered=bool(getattr(self,"wall_approach_timeout_returning",False)
            and self.selected_mode == OperatorCommand.MODE_WALL and wall is not None
            and is_fresh(self.wall_at,self.status_timeout_s,now)
            and wall.state == WallPerchStatus.STATE_EXIT
            and getattr(wall, "fault_recovery_completed", False))
        if not self._wall_approach_timeout_requested(now) and not recovered:
            return "等待新鲜的接近超时/恢复完成状态"
        f=self.flight
        if (f is None or not f.connected or not f.armed
                or not is_fresh(self.flight_at,self.state_timeout_s,now)
                or f.mode.upper() != "OFFBOARD"):
            return "等待新鲜、已解锁的OFFBOARD反馈"
        if (self.owner != 2 or not is_fresh(self.owner_at,self.output_timeout_s,now)
                or self.output_mode != 2 or not is_fresh(self.output_at,self.output_timeout_s,now)):
            return "等待连续受限姿态输出"
        channels=getattr(self,"rc_channels",None)
        if (channels is None or len(channels)<4
                or not is_fresh(self.rc_at,self.rc_timeout_s,now)
                or not all(800<=v<=2200 for v in channels[:4])):
            return "遥控反馈缺失或过期"
        e=self.estimator
        if (e is None or not is_fresh(self.estimator_at,self.estimator_timeout_s,now)
                or not e.attitude_status_flag or not e.velocity_vert_status_flag):
            return "PX4姿态或竖直速度估计未有效"
        if (self._pilot_mode() == "POSCTL" and not (e.velocity_horiz_status_flag
                and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag)
                and not e.const_pos_mode_status_flag)):
            return "PX4位置/水平速度估计未有效，不能自动交回POSCTL"
        odom=self.odom
        if (odom is None or not is_fresh(self.odom_at,self.odom_timeout_s,now)
                or not odometry_is_finite(odom)):
            return "里程计缺失或无效"
        q=odom.pose.pose.orientation
        if not .5<=sum(v*v for v in (q.x,q.y,q.z,q.w))<=1.5:return "姿态四元数无效"
        roll,pitch,_=euler_from_quaternion((q.x,q.y,q.z,q.w))
        if math.cos(roll)*math.cos(pitch)<math.cos(math.radians(5.)):
            return "等待总倾角≤5度"
        a=odom.twist.twist.angular
        if math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)>math.radians(10.):return "等待角速度≤10度/秒"
        speed=vertical_speed_enu_mps(odom,self.odom_twist_in_child_frame)
        if not math.isfinite(speed) or abs(speed)>self.handover_max_vertical_speed_mps:
            return "等待竖直速度进入原有交接范围"
        return None

    def _wall_timeout_handover_ready(self, now):
        if self._wall_timeout_handover_failure(now) is not None:
            self.wall_timeout_stable_since=self.wall_timeout_last_odom=None
            self.wall_timeout_safe_samples=0
            return False
        last=getattr(self,"wall_timeout_last_odom",None)
        if last is not None and not 0 <= (self.odom_at-last).to_sec() <= self.odom_timeout_s:
            self.wall_timeout_stable_since=None;self.wall_timeout_safe_samples=0
        if self.odom_at != last:
            self.wall_timeout_last_odom=self.odom_at
            self.wall_timeout_safe_samples=getattr(self,"wall_timeout_safe_samples",0)+1
            if getattr(self,"wall_timeout_stable_since",None) is None:
                self.wall_timeout_stable_since=self.odom_at
        return bool(self.wall_timeout_safe_samples>=3 and self.wall_timeout_stable_since is not None
            and (self.odom_at-self.wall_timeout_stable_since).to_sec()>=.3)

    def _wall_return_requires_exit(self):
        return bool(getattr(self, "selected_mode", None) == OperatorCommand.MODE_WALL
                    and (getattr(self, "wall_return_episode", False)
                         or getattr(self, "detach_seen", False)
                         or getattr(self, "recovery_seen", False)
                         or getattr(self, "completed", False)
                         or getattr(self, "wall_fault_return_ready", False)))

    def _wall_range_mode(self):
        s = getattr(self, 'wall', None)
        return bool(getattr(self, 'selected_mode', None) == OperatorCommand.MODE_WALL and s is not None
                    and getattr(s, 'detach_range_height_enabled', False)
                    and getattr(s, 'detach_range_target_locked', False)
                    and getattr(s, 'detach_vertical_active', False))

    def _wall_range_identity(self):
        s = getattr(self, 'wall', None)
        if not self._wall_range_mode():
            return None
        return (s.detach_range_maneuver_id, s.detach_range_episode, s.detach_range_target_revision)

    def _captured_recovery(self):
        return bool(getattr(getattr(self, 'wall', None), 'recover_attitude_locked', False))

    def _wall_range_feedback_ready(self, now):
        s = getattr(self, 'wall', None)
        if not self._wall_range_mode() or not is_fresh(getattr(self, 'wall_at', None), self.status_timeout_s, now):
            return False
        if not is_fresh_source(s.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.):
            return False
        if self._captured_recovery():
            # Captured RECOVER no longer waits for range/velocity convergence.
            return bool(s.recover_stable)
        try:
            config = json.loads(s.detach_range_config_json)
            timeout = float(config['timeout_s'])
            speed_limit = float(config['speed_tolerance_mps'])
            height_limit = float(config['height_tolerance_m'])
            future = float(config['future_tolerance_s'])
        except (ValueError, TypeError, KeyError):
            return False
        return bool(s.detach_range_valid and s.detach_range_velocity_valid
            and math.isfinite(s.detach_range_velocity_mps)
            and abs(s.detach_range_velocity_mps) <= min(speed_limit, self.handover_max_vertical_speed_mps)
            and math.isfinite(s.detach_range_target_m) and math.isfinite(s.detach_range_height_m)
            and abs(s.detach_range_target_m-s.detach_range_height_m) <= height_limit
            and is_fresh_source(s.detach_range_source_time, timeout, now, future_tolerance_s=future)
            and is_fresh(s.detach_range_received_time, timeout, now))

    def _range_px4_rc_failure(self, now):
        """Independent FCU takeover readiness; never uses task Z or fused vz."""
        if not getattr(self, 'enabled', True):
            return '模式管理器未启用自动交回'
        if getattr(self, 'wall_pilot_mode', 'POSCTL') != 'POSCTL':
            return '测距闭环要求交回POSCTL，请核对模式管理器配置'
        if not kill_clear(self, now):
            return '急停或权限状态无效'
        e = getattr(self, 'estimator', None)
        if e is None or not is_fresh(getattr(self, 'estimator_at', None), self.estimator_timeout_s, now):
            return 'PX4估计就绪标志过期'
        if not (e.attitude_status_flag and e.velocity_vert_status_flag and e.pos_vert_abs_status_flag
                and e.velocity_horiz_status_flag and (e.pos_horiz_abs_status_flag or e.pos_horiz_rel_status_flag)):
            return 'PX4接管所需位置/速度估计未就绪'
        if e.const_pos_mode_status_flag:
            return 'PX4仍处于恒位置推算模式'
        if not self._rc_fresh(now):
            return '等待有效遥控通道'
        if getattr(self, 'require_rc_center', False) and not all(
                abs(self.rc_channels[i]-self.rc_pwm_center) <= self.rc_center_tolerance_pwm
                for i in (self.rc_roll_channel_index, self.rc_pitch_channel_index, self.rc_throttle_channel_index)):
            return '等待遥控摇杆回中'
        return ''

    def _publish_range_handover_readiness(self, now):
        pub = getattr(self, 'range_handover_pub', None)
        s = getattr(self, 'wall', None)
        if pub is None or s is None:
            return
        reason = self._range_px4_rc_failure(now)
        if not is_fresh(getattr(self, 'wall_at', None), self.status_timeout_s, now):
            reason = '侧吸状态过期'
        request = getattr(self, 'range_posctl_request', None)
        identity = (getattr(s, 'detach_range_maneuver_id', ''), getattr(s, 'detach_range_episode', 0),
                    getattr(s, 'detach_range_target_revision', 0))
        pub.publish(String(data=json.dumps(dict(episode=identity[1], maneuver_id=identity[0],
            target_revision=identity[2],
            source_time=now.to_sec(), ready=not bool(reason), reason=reason,
            posctl_request_time=request[1] if request is not None and request[0] == identity else 0.))))

    def _wall_return_exit_ready(self, now):
        status = getattr(self, "wall", None)
        if (status is None or not is_fresh(getattr(self, "wall_at", None), self.status_timeout_s, now)
                or status.state != WallPerchStatus.STATE_EXIT):
            return False
        if self._wall_range_mode():
            return bool(self._wall_range_feedback_ready(now)
                and (getattr(status, 'fault_recovery_completed', False)
                     or (status.detach_task_phase_name == 'HANDOVER_READY'
                         and status.detach_range_handover_ready
                         and not status.fault_detected and not getattr(self, 'fault_seen', False))))
        return bool(getattr(status, "fault_recovery_completed", False)
                    or (getattr(status, "detach_completed", False)
                        and getattr(status, "recovery_completed", False)
                        and not status.fault_detected and not getattr(self, "fault_seen", False)))


    def _captured_recovery_handover_ready(self, now):
        from px4_wall_ceiling_control.domain.geometry import rotate_vector
        from px4_wall_ceiling_control.domain.simple_recovery import attitude_within_margins
        if (not self._wall_return_exit_ready(now) or not getattr(self.wall, 'recover_stable', False)
                or self._range_px4_rc_failure(now) or not self._wall_fusion_handover_ready(now)
                or self.output_mode != 2 or not is_fresh(self.output_at, self.output_timeout_s, now)
                or self.odom is None or not is_fresh(self.odom_at, self.odom_timeout_s, now)
                or not is_fresh_source(self.odom.header.stamp, self.odom_timeout_s, now)
                or not odometry_attitude_horizontal_is_finite(self.odom, self.odom_twist_in_child_frame)):
            return False
        q = self.odom.pose.pose.orientation
        tilt = math.acos(max(-1., min(1., rotate_vector((q.x, q.y, q.z, q.w), (0., 0., 1.))[2])))
        a = self.odom.twist.twist.angular
        return attitude_within_margins(tilt, math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z))

    def _handover_ready(self, now):
        if self._captured_recovery():
            return self._captured_recovery_handover_ready(now)
        if self._wall_range_mode() and self._range_px4_rc_failure(now):
            return False
        if not self._wall_fusion_handover_ready(now):return False
        if getattr(self,"wall_approach_timeout_returning",False):
            return self._wall_timeout_handover_ready(now)
        # A normal EXIT can be revoked while waiting for POSCTL. Read the live
        # terminal status again here, including the service worker's recheck.
        if self._wall_return_requires_exit() and not self._wall_return_exit_ready(now):
            return False
        if (getattr(self,"selected_mode",None) == OperatorCommand.MODE_WALL and not self._wall_horizontal_ready(now,stable=True)):
            return False
        if self.output_mode != 2 or not is_fresh(
            self.output_at, self.output_timeout_s, now
        ):
            return False
        if self.estimator is None or not is_fresh(
            self.estimator_at, self.estimator_timeout_s, now
        ) or not (
            self.estimator.attitude_status_flag
            and self.estimator.velocity_vert_status_flag
        ):
            return False
        finite = (odometry_attitude_horizontal_is_finite(self.odom, self.odom_twist_in_child_frame)
                  if self._wall_range_mode() else odometry_is_finite(self.odom))
        if self.odom is None or not is_fresh(
            self.odom_at, self.odom_timeout_s, now
        ) or not finite:
            return False
        if getattr(self, "selected_mode", None) == OperatorCommand.MODE_CEILING and not self._ceiling_position_ready(now):
            return False
        if getattr(self, "ceiling_approach_timeout_returning", False):
            # This is an aborted approach, so there is no HOLD pressure baseline.
            # Keep the FAULT stream until a live POSCTL response. Valid RC input
            # is required, but descent speed/centering must not prevent return.
            return bool(
                self._ceiling_approach_timeout_requested(now)
                and self.flight is not None
                and is_fresh(self.flight_at, self.state_timeout_s, now)
                and self.flight.connected and self.flight.armed
                and self.flight.mode.upper() == "OFFBOARD"
                and self.owner == 1
                and is_fresh(self.owner_at, self.output_timeout_s, now)
                and self._rc_fresh(now)
            )
        if getattr(self, "ceiling_baro_handover_enabled", False) and self.selected_mode == OperatorCommand.MODE_CEILING:
            return self._ceiling_baro_handover_ready(now)  # No legacy speed/settle fallback.
        orientation = self.odom.pose.pose.orientation
        roll, pitch, _yaw = euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        limit = math.radians(self.handover_max_tilt_deg)
        if self._wall_range_mode():
            if not self._wall_range_feedback_ready(now):
                return False
            vertical_speed = self.wall.detach_range_velocity_mps
        else:
            vertical_speed = vertical_speed_enu_mps(self.odom, self.odom_twist_in_child_frame)
        angular = self.odom.twist.twist.angular
        if any(not math.isfinite(v) or abs(v) > 0.3 for v in (angular.x, angular.y, angular.z)):
            return False
        if (
            abs(roll) > limit or abs(pitch) > limit
            or not math.isfinite(vertical_speed)
            or (not getattr(self, "wall_bench_wait_for_ranges", False)
                and abs(vertical_speed) > self.handover_max_vertical_speed_mps)
        ):
            return False
        if not self._rc_fresh(now):
            return False
        if not getattr(self, "require_rc_center", False):
            return True
        return all(
            abs(self.rc_channels[index] - self.rc_pwm_center)
            <= self.rc_center_tolerance_pwm
            for index in (
                self.rc_roll_channel_index,
                self.rc_pitch_channel_index,
                self.rc_throttle_channel_index,
            )
        )

    def _observe_completion(self, now):
        selected_owner = (
            1 if self.selected_mode == OperatorCommand.MODE_CEILING else 2
        )
        if self.owner == selected_owner and is_fresh(
            self.owner_at, self.output_timeout_s, now
        ):
            self.owner_seen = True
        if self.selected_mode == OperatorCommand.MODE_CEILING:
            status, received_at = self.ceiling, self.ceiling_at
            if status is None or not is_fresh(received_at, self.status_timeout_s, now):
                return
            if status.state == CeilingAttachmentStatus.STATE_ABORT or status.detach_failed:
                self.fault_seen = True
            if status.state == CeilingAttachmentStatus.STATE_DETACH:
                self.detach_seen = True
                if self._ceiling_baro_handover_ready(now):
                    self.completed = True
            elif self.detach_seen and status.state == CeilingAttachmentStatus.STATE_RECOVERY_HOVER:
                self.recovery_seen = True
                if self._ceiling_baro_handover_ready(now):
                    self.completed = True
            elif (
                self.recovery_seen
                and status.state == CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
                and not status.detach_failed
                and not status.fault_detected
                and not self.fault_seen
            ):
                self.completed = True
        elif self.selected_mode == OperatorCommand.MODE_WALL:
            status, received_at = self.wall, self.wall_at
            self.completed = False
            self.wall_fault_return_ready = False
            if status is None or not is_fresh(received_at, self.status_timeout_s, now):
                return
            if (status.state in (WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER)
                    or getattr(status, "recovery_completed", False)
                    or getattr(status, "fault_recovery_completed", False)):
                self.wall_return_episode = True
            if status.state == WallPerchStatus.STATE_ABORT or status.fault_detected:
                self.fault_seen = True
                if status.state == WallPerchStatus.STATE_ABORT or not getattr(self, "wall_failure_reason", ""):
                    self.wall_failure_reason = str(status.reason) or "侧吸故障"
                self.completed = False
            if status.state == WallPerchStatus.STATE_DETACH_ROTATE:
                self.detach_seen = True
            elif self.detach_seen and status.state == WallPerchStatus.STATE_RECOVER:
                self.recovery_seen = True
            if status.state == WallPerchStatus.STATE_EXIT:
                if getattr(status, "fault_recovery_completed", False):
                    # Safe leveling after ABORT authorizes a fault handback,
                    # never a successful detach/recover result. Explicit flags
                    # also cover a manager that missed intermediate samples.
                    self.fault_seen = True
                    self.completed = False
                    self.wall_fault_return_ready = True
                    if not getattr(self, "wall_failure_reason", ""):
                        self.wall_failure_reason = str(status.reason) or "侧吸故障"
                elif ((getattr(status, "detach_completed", False)
                      and getattr(status, "recovery_completed", False)
                      or (self._wall_range_mode() and self._wall_return_exit_ready(now)))
                      and not status.fault_detected and not self.fault_seen):
                    self.completed = True

    def _wall_return_reason(self, confirmed=False):
        if self._wall_range_mode() and not confirmed and not getattr(self, 'wall_fault_return_ready', False):
            return ('姿态已稳定，准备交回POSCTL；等待新鲜飞控模式确认' if self._captured_recovery()
                    else '测距闭环已稳定，准备交回POSCTL；等待新鲜飞控模式确认')
        if getattr(self, "wall_fault_return_ready", False):
            result = "故障恢复后已安全交回" if confirmed else "故障恢复完成，准备安全交回"
            return result + self._pilot_mode() + "；本次侧吸未正常完成：" + getattr(self, "wall_failure_reason", "侧吸故障")
        result = "正常脱离及恢复完成，已交回" if confirmed else "正常脱离及恢复完成，准备交回"
        return result + self._pilot_mode()

    def _wall_approach_aborted(self, now):
        return bool(self.selected_mode == OperatorCommand.MODE_WALL
                    and self.wall is not None and is_fresh(self.wall_at, self.status_timeout_s, now)
                    and self.wall.state == WallPerchStatus.STATE_ABORT
                    and str(self.wall.reason).startswith("approach front distance "))

    def _wall_flip_started(self, now):
        return bool(
            self.wall is not None
            and is_fresh(self.wall_at, self.status_timeout_s, now)
            and self.wall.state == WallPerchStatus.STATE_FLIP_TO_WALL
            and self.wall.candidate_valid and not self.wall.fault_detected
        )

    def _current_offboard_request(self, monotonic_now):
        request = getattr(self, "offboard_request", None)
        return bool(
            request is not None
            and request[0] == getattr(self, "takeover_attempt", 0)
            and request[1] == self.selected_mode
            and 0.0 <= monotonic_now - request[2] <= self.enter_timeout_s
        )

    def _acquire_failure_reason(self, now):
        if self.selected_mode == OperatorCommand.MODE_CEILING:
            status, received_at, label = self.ceiling, self.ceiling_at, "ceiling"
        elif self.selected_mode == OperatorCommand.MODE_WALL:
            status, received_at, label = self.wall, self.wall_at, "wall"
        else:
            return "attachment mode is not selected"
        if status is None:
            return "{} status unavailable".format(label)
        if not is_fresh(received_at, self.status_timeout_s, now):
            return "{} status timeout".format(label)
        detail = str(getattr(status, "reason", "")).strip()
        return detail or "{} candidate did not request ownership".format(label)

    def _request_mode(self, mode):
        if getattr(self, "rc_takeover_requested", False) and (
                self.phase != "RC_TAKEOVER" or mode != self._pilot_mode()
                or not self._rc_takeover_handover_ready(rospy.Time.now())):
            return
        now = time.monotonic()
        if self.request_in_flight or (
            self.last_request_at is not None
            and now - self.last_request_at < self.retry_s
        ):
            return
        self.last_request_at = now
        self.request_in_flight = True
        threading.Thread(
            target=self._send_mode_request, args=(mode, getattr(self, "takeover_attempt", 0), self._wall_range_identity()), daemon=True
        ).start()

    def _send_mode_request(self, mode, attempt=None, range_identity=None):
        try:
            if mode == 'AUTO.LAND':
                rospy.wait_for_service('/mavros/set_mode', timeout=.2)
                ceiling_landing = self._ceiling_abort_land_live(rospy.Time.now(), attempt)
                if not ceiling_landing and not self._wall_abort_land_live(rospy.Time.now(), attempt):
                    return
                request = SetModeRequest()
                request.custom_mode = 'AUTO.LAND'
                request_stamp = 'ceiling_land_request_at' if ceiling_landing else 'wall_land_request_at'
                if getattr(self, request_stamp, None) is None:
                    setattr(self, request_stamp, rospy.Time.now().to_sec())
                response = self.set_mode(request)
                if not response.mode_sent:
                    rospy.logwarn('PX4 did not accept AUTO.LAND mode request')
                return
            if mode not in ("OFFBOARD", self._pilot_mode()):
                return
            rospy.wait_for_service("/mavros/set_mode", timeout=0.2)
            if not kill_clear(self, rospy.Time.now()):
                return
            if getattr(self, "rc_takeover_requested", False) and (
                    self.phase != "RC_TAKEOVER" or mode != self._pilot_mode()
                    or not self._rc_takeover_handover_ready(rospy.Time.now())):
                return
            if mode == "OFFBOARD":
                if (
                    self.phase != self.ENTERING
                    or self._requested_mode(rospy.Time.now()) != self.selected_mode
                    or not self._preflight_ready(rospy.Time.now(), self.selected_mode)
                    or not self._stream_ready(rospy.Time.now())
                    or (self._gated_wall_selected() and not self._wall_takeover_ready(rospy.Time.now()))
                    or not self._ceiling_takeover_ready(rospy.Time.now())
                ):
                    return
            elif mode == self._pilot_mode():
                if self.phase == "RC_TAKEOVER":
                    if not self._rc_takeover_handover_ready(rospy.Time.now()):
                        return
                elif self.phase == self.CANCELING_ENTRY:
                    ready = (self._offboard_exit_request_ready(rospy.Time.now())
                             if getattr(self, "offboard_exit_cleanup", False)
                             else self._entry_cancel_handover_ready(rospy.Time.now()))
                    if not ready:
                        return
                elif self.phase != self.RETURNING or not self._handover_ready(rospy.Time.now()):
                    return
            # Recheck immediately before the RPC as restoration status can change
            # while this worker is waiting for MAVROS or another eligibility gate.
            if mode != 'OFFBOARD' and not self._wall_fusion_handover_ready(rospy.Time.now()):
                return
            if attempt is not None and attempt != getattr(self, "takeover_attempt", 0):
                return  # A canceled/previous attempt must not send a late mode change.
            if mode != "OFFBOARD" and self.phase == self.RETURNING:
                if range_identity is not None and range_identity != self._wall_range_identity():
                    return
                # A flight callback can arrive while this worker waits for the
                # service, before the timer retires RETURNING. Never override a
                # pilot-selected mode (or a disconnected/disarmed FCU) using
                # still-fresh EXIT and output feedback from the old mode.
                flight = self.flight
                if (flight is None or not flight.connected or not flight.armed
                        or flight.mode.upper() != "OFFBOARD"
                        or not is_fresh(self.flight_at, self.state_timeout_s, rospy.Time.now())):
                    return
            if mode == "OFFBOARD":
                self.offboard_request = (getattr(self, "takeover_attempt", 0),
                                         self.selected_mode, time.monotonic())
            request = SetModeRequest()
            request.custom_mode = mode
            if mode != 'OFFBOARD' and self.phase == self.RETURNING and self._wall_range_mode():
                if not self._handover_ready(rospy.Time.now()):
                    return
                self.range_posctl_request = (self._wall_range_identity(), rospy.Time.now().to_sec())
            if mode == 'POSCTL' and getattr(self, "selected_mode", None) == OperatorCommand.MODE_CEILING:
                # Retain the first request epoch across retries, so heartbeats are not starved.
                if getattr(self, "ceiling_posctl_request_at", None) is None:
                    self.ceiling_posctl_request_at = rospy.Time.now().to_sec()
            response = self.set_mode(request)
            if not response.mode_sent:
                rospy.logwarn("PX4 did not accept %s mode request", mode)
            elif (mode != "OFFBOARD" and self.phase == self.CANCELING_ENTRY
                  and getattr(self, "offboard_exit_cleanup", False)
                  and (attempt is None or attempt == self.takeover_attempt)):
                # mode_sent means submitted, not confirmed. Require NEW heartbeats.
                self.cancel_entry_pilot_samples = 0
                self.pilot_intent_request_at = time.monotonic()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("MAVROS set_mode %s failed: %s", mode, exc)
        finally:
            if mode == "OFFBOARD":
                self.last_offboard_finished_at = time.monotonic()
            self.request_in_flight = False

    def _reset(self, lock=False, reason=None):
        self.entry_lock_reason = reason if lock else ""
        self.ceiling_detach_pending_at = None
        self.ceiling_local_land_fault = ''
        self.ceiling_local_land_id = ''
        self.ceiling_posctl_request_at = None
        self.ceiling_flight_source = None
        self.wall_approach_timeout_returning = False
        self.wall_timeout_stable_since = self.wall_timeout_last_odom = None
        self.wall_timeout_safe_samples = 0
        self.offboard_exit_cleanup = False
        self.pilot_intent_request_at = None
        self.takeover_attempt = getattr(self, "takeover_attempt", 0) + 1
        self.offboard_request = None
        self.selected_mode = None
        self.manual_wall_wait_started = None
        self.manual_wait_canceled = False
        self.stream_since = None
        self.owner_seen = False
        self.detach_seen = False
        self.recovery_seen = False
        self.completed = False
        self.fault_seen = False
        self.wall_fault_return_ready = False
        self.wall_failure_reason = ""
        self.wall_return_episode = False
        self.lock_until_neutral = lock
        self.ceiling_approach_timeout_returning = False
        self._set_phase(self.LOCKED if lock else self.IDLE, reason=reason)

    def _wall_land_status_identity(self):
        s = getattr(self, 'wall', None)
        return (getattr(s, 'detach_range_maneuver_id', ''), getattr(s, 'detach_range_episode', 0))

    def _wall_abort_land_live(self, now, attempt=None):
        s, flight = getattr(self, 'wall', None), getattr(self, 'flight', None)
        return bool(self.phase == 'WALL_LANDING'
            and (attempt is None or attempt == getattr(self, 'takeover_attempt', 0))
            and not getattr(self, 'rc_takeover_requested', False)
            and kill_clear(self, now)
            and getattr(self, 'wall_land_identity', None) == self._wall_land_status_identity()
            and s is not None and s.state == WallPerchStatus.STATE_ABORT
            and getattr(s, 'abort_land_requested', False)
            and is_fresh(getattr(self, 'wall_at', None), self.status_timeout_s, now)
            and is_fresh_source(s.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)
            and flight is not None and flight.connected and flight.armed
            and flight.mode.upper() == 'OFFBOARD'
            and is_fresh(self.flight_at, self.state_timeout_s, now)
            and is_fresh_source(flight.header.stamp, self.state_timeout_s, now, future_tolerance_s=0.))

    def _tick_wall_abort_land(self, now):
        s = getattr(self, 'wall', None)
        identity = self._wall_land_status_identity()
        if self.phase != 'WALL_LANDING':
            if (getattr(self, 'selected_mode', None) != OperatorCommand.MODE_WALL
                    or s is None or s.state != WallPerchStatus.STATE_ABORT
                    or not getattr(s, 'abort_land_requested', False)
                    or identity == getattr(self, 'wall_land_retired_identity', None)
                    or not is_fresh(getattr(self, 'wall_at', None), self.status_timeout_s, now)
                    or not is_fresh_source(s.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)):
                return False
            self.wall_land_identity = identity
            self.wall_land_request_at = None
            self.wall_land_confirmed = False
            self.last_request_at = None
            self._set_phase('WALL_LANDING', reason='detach最终俯仰为负，立即请求PX4 AUTO.LAND')
        flight = getattr(self, 'flight', None)
        fresh = (flight is not None and is_fresh(self.flight_at, self.state_timeout_s, now)
                 and is_fresh_source(flight.header.stamp, self.state_timeout_s, now, future_tolerance_s=0.))
        if fresh and (not flight.connected or not flight.armed):
            self.wall_land_retired_identity = self.wall_land_identity
            self._reset(lock=True, reason='降落分支结束：飞控断连或已解除武装')
        elif fresh and flight.mode.upper() != 'OFFBOARD':
            request_at = self.wall_land_request_at
            if (flight.mode.upper() == 'AUTO.LAND' and request_at is not None
                    and flight.header.stamp.to_sec() > request_at and self.flight_at.to_sec() > request_at):
                self.wall_land_confirmed = True
                self.phase_reason = 'PX4 AUTO.LAND新鲜反馈已确认，降落由飞控执行'
            elif flight.mode.upper() != 'AUTO.LAND':
                self.wall_land_retired_identity = self.wall_land_identity
                self._reset(lock=True, reason='飞手/外部模式接管，停止自动降落请求')
        elif fresh and getattr(self, 'wall_land_confirmed', False):
            self.wall_land_retired_identity = self.wall_land_identity
            self._reset(lock=True, reason='已确认降落后的模式改变，禁止再次自动接管')
        elif self._wall_abort_land_live(now):
            self._request_mode('AUTO.LAND')
        self.prestream_pub.publish(Bool(data=False))
        self.phase_pub.publish(String(data=self.phase))
        self.phase_reason_pub.publish(String(data=self.phase_reason))
        return True

    def _ceiling_hold_command_fault(self, now):
        if not (getattr(self, 'ceiling_explicit_detach', False)
                and self.phase == self.ACTIVE and self.selected_mode == OperatorCommand.MODE_CEILING):
            self.ceiling_detach_pending_at = None
            return ''
        s = getattr(self, 'ceiling', None)
        if s is None or s.state != CeilingAttachmentStatus.STATE_SURFACE_HOLD:
            self.ceiling_detach_pending_at = None
            return ''
        if (not is_fresh(getattr(self, 'ceiling_at', None), self.status_timeout_s, now)
                or not is_fresh_source(s.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)):
            return '顶吸保持/脱离状态反馈失效'
        op = getattr(self, 'operator', None)
        if (op is None or not is_fresh(getattr(self, 'operator_at', None), self.operator_timeout_s, now)
                or not is_fresh_source(op.header.stamp, self.operator_timeout_s, now, future_tolerance_s=0.)
                or op.mode != OperatorCommand.MODE_CEILING or not op.enable_control):
            return '顶吸保持阶段指令反馈失效'
        if op.source == 'terminal' and op.reason == 'detach pending':
            if getattr(self, 'ceiling_detach_pending_at', None) is None:
                self.ceiling_detach_pending_at = now
            elif (now-self.ceiling_detach_pending_at).to_sec() >= 1.0:
                return '脱离请求已发出但1秒内未进入DETACH'
        elif getattr(self, 'ceiling_detach_pending_at', None) is not None:
            return '脱离请求反馈在进入DETACH前丢失'
        return ''

    def _ceiling_land_status_identity(self):
        if self.phase == 'CEILING_LANDING' and getattr(self, 'ceiling_local_land_fault', ''):
            return self.ceiling_local_land_id
        return getattr(getattr(self, 'ceiling', None), 'abort_maneuver_id', '')

    def _ceiling_abort_land_live(self, now, attempt=None):
        s, flight = getattr(self, 'ceiling', None), getattr(self, 'flight', None)
        return bool(self.phase == 'CEILING_LANDING'
            and (attempt is None or attempt == getattr(self, 'takeover_attempt', 0))
            and not getattr(self, 'rc_takeover_requested', False)
            and kill_clear(self, now)
            and getattr(self, 'ceiling_land_identity', None) == self._ceiling_land_status_identity()
            and (bool(getattr(self, 'ceiling_local_land_fault', '')) or (
                s is not None and s.state == CeilingAttachmentStatus.STATE_ABORT
                and getattr(s, 'abort_land_requested', False)
                and is_fresh(getattr(self, 'ceiling_at', None), self.status_timeout_s, now)
                and is_fresh_source(s.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)))
            and flight is not None and flight.connected and flight.armed
            and (flight.mode.upper() == 'OFFBOARD' or (getattr(self, 'ceiling_land_from_entry', False) and flight.mode.upper() == 'POSCTL'))
            and is_fresh(self.flight_at, self.state_timeout_s, now)
            and is_fresh_source(flight.header.stamp, self.state_timeout_s, now, future_tolerance_s=0.))

    def _tick_ceiling_abort_land(self, now, local_fault=''):
        s = getattr(self, 'ceiling', None)
        identity = self._ceiling_land_status_identity()
        if self.phase != 'CEILING_LANDING':
            if local_fault:
                self.ceiling_local_land_fault = local_fault
                self.ceiling_local_land_id = 'hold-feedback-%d-%d' % (getattr(self, 'takeover_attempt', 0), now.to_nsec())
                identity = self.ceiling_local_land_id
                self.cancel_pub.publish(String(data='cancel'))
            elif (getattr(self, 'selected_mode', None) != OperatorCommand.MODE_CEILING
                    or s is None or s.state != CeilingAttachmentStatus.STATE_ABORT
                    or not getattr(s, 'abort_land_requested', False)
                    or not identity
                    or identity == getattr(self, 'ceiling_land_retired_identity', None)
                    or not is_fresh(getattr(self, 'ceiling_at', None), self.status_timeout_s, now)
                    or not is_fresh_source(s.header.stamp, self.status_timeout_s, now, future_tolerance_s=0.)):
                return False
            self.ceiling_land_from_entry = self.phase in (self.PRESTREAM, self.ENTERING)
            self.ceiling_land_started_at = now.to_sec()
            self.ceiling_land_identity = identity
            self.ceiling_land_request_at = None
            self.ceiling_land_confirmed = False
            self.last_request_at = None
            self._set_phase('CEILING_LANDING', reason='顶吸安全中止，立即请求PX4 AUTO.LAND：' + (local_fault or getattr(s, 'reason', '')))
        flight = getattr(self, 'flight', None)
        fresh = (flight is not None and is_fresh(self.flight_at, self.state_timeout_s, now)
                 and is_fresh_source(flight.header.stamp, self.state_timeout_s, now, future_tolerance_s=0.))
        if fresh and flight.connected and flight.mode.upper() == 'OFFBOARD':
            self.ceiling_land_from_entry = False  # A later POSCTL change is a pilot override.
        if not fresh or not flight.connected:
            self.phase_reason = 'ABORT降落意图已锁存；飞控通信不可用，无法发送或确认AUTO.LAND；依靠PX4内部失联保护'
        elif not flight.armed:
            self.ceiling_land_retired_identity = self.ceiling_land_identity
            self._reset(lock=True, reason='降落分支结束：飞控已确认解除武装')
        elif flight.mode.upper() != 'OFFBOARD' and not (getattr(self, 'ceiling_land_from_entry', False) and flight.mode.upper() == 'POSCTL'):
            request_at = self.ceiling_land_request_at
            confirm_after = request_at if request_at is not None else self.ceiling_land_started_at
            if (flight.mode.upper() == 'AUTO.LAND'
                    and flight.header.stamp.to_sec() > confirm_after and self.flight_at.to_sec() > confirm_after):
                self.ceiling_land_confirmed = True
                self.phase_reason = 'PX4 AUTO.LAND新鲜反馈已确认，降落由飞控执行'
            elif flight.mode.upper() != 'AUTO.LAND':
                self.ceiling_land_retired_identity = self.ceiling_land_identity
                self._reset(lock=True, reason='飞手/外部模式接管，停止自动降落请求')
        elif fresh and getattr(self, 'ceiling_land_confirmed', False):
            self.ceiling_land_retired_identity = self.ceiling_land_identity
            self._reset(lock=True, reason='已确认降落后的模式改变，禁止再次自动接管')
        elif self._ceiling_abort_land_live(now):
            self._request_mode('AUTO.LAND')
        self.prestream_pub.publish(Bool(data=False))
        self.phase_pub.publish(String(data=self.phase))
        self.phase_reason_pub.publish(String(data=self.phase_reason))
        return True

    def _on_timer(self, _event):
        now = rospy.Time.now()
        self._publish_range_handover_readiness(now)
        if self._tick_rc_takeover_manager(now):
            return
        if not kill_clear(self, now):
            if self.phase == self.RETURNING and getattr(self,"wall_approach_timeout_returning",False):
                self._begin_offboard_exit("急停生效或反馈无效；输出立即停止",time.monotonic())
            elif self.phase in (self.ENTERING, self.CANCELING_ENTRY):
                self._begin_entry_cancel("急停生效或反馈无效；输出立即停止", time.monotonic())
            else:
                self._reset(lock=True, reason="kill switch active or unavailable")
            self.prestream_pub.publish(Bool(data=False))
            self.phase_pub.publish(String(data=self.phase))
            self.phase_reason_pub.publish(String(data=self.phase_reason))
            return
        if self._tick_ceiling_abort_land(now, self._ceiling_hold_command_fault(now)):
            return
        if self._tick_wall_abort_land(now):
            return
        monotonic_now = time.monotonic()
        requested = self._requested_mode(now)
        flight_mode = self.flight.mode.upper() if self.flight else ""
        flight_fresh = is_fresh(self.flight_at, self.state_timeout_s, now)
        if (self.phase == self.RETURNING and flight_mode == "OFFBOARD"
                and getattr(self, "selected_mode", None) == OperatorCommand.MODE_WALL
                and not getattr(self, "wall_approach_timeout_returning", False)):
            self._observe_completion(now)

        if (self.phase in (self.IDLE, self.LOCKED) and flight_fresh and flight_mode == "OFFBOARD"
                and getattr(self, "canceled_entry_mode", None) is not None):
            self.selected_mode = self.canceled_entry_mode
            self._begin_offboard_exit("检测到已取消请求的迟到OFFBOARD反馈", monotonic_now)
        if self.phase == self.LOCKED and requested is None:
            self._reset(
                lock=False,
                reason="operator request released; retry enabled",
            )
        elif self.phase == self.LOCKED and requested == OperatorCommand.MODE_CEILING:
            original = getattr(self, "entry_lock_reason", None) or "此前接管被中止"
            self.phase_reason = ("启动请求仍锁定：" + original
                + "；当前急停检查已通过。请先在网页取消启动请求，确认待命后重新拨CH12或点击启动；CH12回低位不会取消已提交请求")
        if self.phase == self.IDLE:
            if (
                requested is not None
                and flight_mode == self._pilot_mode(requested)
                and self._preflight_ready(now, requested)
            ):
                self.takeover_attempt = getattr(self, "takeover_attempt", 0) + 1
                self.offboard_request = None
                self.selected_mode = requested
                if self._gated_wall_selected():
                    self.manual_wall_wait_started = monotonic_now
                    self._set_phase(self.WAITING_WALL, reason=self._manual_wait_reason(now, monotonic_now))
                else:
                    self._set_phase(self.PRESTREAM)
            elif requested == OperatorCommand.MODE_CEILING:
                detail = ("飞控需处于" + self._pilot_mode(requested)
                          if flight_mode != self._pilot_mode(requested)
                          else self._preflight_failure_reason(now, requested))
                self.phase_reason = "启动请求已收到，尚未接管：" + (detail or "等待有效接管条件")
        elif self.phase == self.WAITING_WALL:
            if getattr(self, "manual_wait_canceled", False):
                self._reset(lock=True, reason="手动接近等待已取消；仍由飞手控制 ALTCTL")
                self.cancel_pub.publish(String(data="cancel"))
            elif requested != self.selected_mode:
                self._reset(lock=requested is not None, reason="manual wall request canceled")
            elif self._wall_approach_aborted(now):
                self._reset(lock=True, reason=self.wall.reason)
                self.cancel_pub.publish(String(data="cancel"))
            elif flight_fresh and flight_mode == "OFFBOARD":
                # Recognize this attempt's handover despite delayed phase feedback.
                # An unrelated/stale OFFBOARD change must never start a flip.
                if (self._current_offboard_request(monotonic_now)
                    and self._preflight_ready(now, self.selected_mode)
                    and self._stream_ready(now)):
                    if self._wall_flip_started(now):
                        self._set_phase(self.ACTIVE, reason="本次接管已确认：PX4 OFFBOARD，贴墙节点已开始翻转")
                    else:
                        self._set_phase(self.ENTERING, reason="本次 OFFBOARD 请求已确认；等待贴墙节点及有效距离")
                else:
                    reason = (self._preflight_failure_reason(now, self.selected_mode)
                              or "OFFBOARD 不属于本次有效接管，或预发送反馈已过期")
                    self._begin_entry_cancel(reason, monotonic_now)
            elif flight_mode != self._pilot_mode() or not self._preflight_ready(now, self.selected_mode):
                self._reset(lock=True, reason=self._preflight_failure_reason(now, self.selected_mode) or "manual approach requires " + self._pilot_mode())
                self.cancel_pub.publish(String(data="cancel"))
            elif self._manual_wait_elapsed(monotonic_now) >= self.manual_wall_wait_timeout_s:
                self._reset(lock=True, reason="手动接近等待超时，贴墙请求已取消；仍由飞手控制 ALTCTL")
                self.cancel_pub.publish(String(data="cancel"))
            else:
                # Warm the OFFBOARD stream while the pilot approaches in ALTCTL.
                # Distance changes do not restart this independent stream timer.
                if not self._stream_ready(now):
                    self.stream_since = None
                elif self.stream_since is None:
                    self.stream_since = monotonic_now
                stream_warm = (self.stream_since is not None
                               and monotonic_now - self.stream_since >= self.prestream_s)
                if stream_warm and self._wall_takeover_ready(now):
                    self._set_phase(self.ENTERING)
                    self._request_mode("OFFBOARD")
                elif self._wall_takeover_ready(now):
                    self.phase_reason = "距离条件已满足；等待并行预发送完成，仍由飞手控制 " + self._pilot_mode()
                else:
                    self.phase_reason = self._manual_wait_reason(now, monotonic_now)
        elif self.phase == self.PRESTREAM:
            failure_reason = None
            if requested != self.selected_mode:
                failure_reason = "operator request changed or was released"
            elif flight_mode != self._pilot_mode():
                failure_reason = "flight mode changed from {} to {}".format(
                    self._pilot_mode(), flight_mode or "unknown"
                )
            elif not self._preflight_ready(now, self.selected_mode):
                failure_reason = self._preflight_failure_reason(
                    now, self.selected_mode
                )
            if failure_reason is not None:
                rospy.logwarn("OFFBOARD prestream stopped: %s", failure_reason)
                self._reset(
                    lock=requested is not None,
                    reason=failure_reason,
                )
            elif self._stream_ready(now):
                if self.stream_since is None:
                    self.stream_since = monotonic_now
                elif monotonic_now - self.stream_since >= self.prestream_s:
                    if self._ceiling_takeover_ready(now):
                        self._set_phase(self.ENTERING)
                        self._request_mode("OFFBOARD")
                    else:
                        self.phase_reason = self._ceiling_wait_reason(now)
            else:
                self.stream_since = None
        elif self.phase == self.ENTERING:
            failure_reason = None
            if requested != self.selected_mode:
                failure_reason = "operator request changed or was released"
            elif self._wall_approach_aborted(now):
                failure_reason = self.wall.reason
            elif not self._preflight_ready(now, self.selected_mode):
                failure_reason = self._preflight_failure_reason(now, self.selected_mode) or "preflight safety lost"
            elif self._entry_stream_failed(now, monotonic_now):
                failure_reason = "OFFBOARD setpoint stream lost beyond entry grace"
            elif monotonic_now - self.phase_since > self.enter_timeout_s:
                failure_reason = "PX4 OFFBOARD entry timed out"
            if failure_reason is not None:
                self._begin_entry_cancel(failure_reason, monotonic_now)
            elif flight_fresh and flight_mode == "OFFBOARD":
                if not self._current_offboard_request(monotonic_now):
                    self._begin_entry_cancel("未确认本次OFFBOARD请求", monotonic_now)
                elif not self._entry_stream_ready(now, monotonic_now):
                    self.phase_reason = "本次OFFBOARD已反馈；等待短暂输出间断恢复，禁止开始侧吸"
                elif (self._manual_wall_selected() and not self._manual_wall_ready(now)
                        and not self._wall_flip_started(now)):
                    if getattr(self, "ignore_range_loss_during_attachment", False):
                        self.phase_reason = "接管等待：距离条件失效，保持中性输出，等待有效距离后翻转"
                    else:
                        self._begin_entry_cancel("OFFBOARD确认时距离条件失效", monotonic_now)
                else:
                    self.canceled_entry_mode = None
                    self._set_phase(self.ACTIVE, reason="PX4 OFFBOARD接管已确认；开始受限自动接近" if self._auto_wall_selected() else "PX4 entered OFFBOARD")
            elif self._stream_ready(now):
                self._request_mode("OFFBOARD")
        elif self.phase == self.CANCELING_ENTRY:
            self._tick_entry_cancel(now, monotonic_now, flight_fresh, flight_mode)
        elif self.phase == self.ACTIVE:
            if flight_fresh and flight_mode != "OFFBOARD":
                failure_reason = "PX4 left OFFBOARD for {}".format(
                    flight_mode or "unknown mode"
                )
                rospy.logwarn("%s; automatic reentry inhibited", failure_reason)
                self._begin_offboard_exit(failure_reason, monotonic_now)
            else:
                self._observe_completion(now)
                if self.selected_mode == OperatorCommand.MODE_CEILING and getattr(self, "ceiling", None) is not None and is_fresh(self.ceiling_at, self.status_timeout_s, now):
                    self.phase_reason = self.ceiling.reason
                if (self._auto_wall_selected() and self.wall is not None
                        and is_fresh(self.wall_at, self.status_timeout_s, now)
                        and self.wall.state == WallPerchStatus.STATE_SLOW_APPROACH):
                    self.phase_reason = self.wall.reason
                if self._wall_approach_timeout_requested(now):
                    self.wall_approach_timeout_returning = True
                    self.wall_timeout_stable_since=self.wall_timeout_last_odom=None
                    self.wall_timeout_safe_samples=0
                    self.cancel_pub.publish(String(data="cancel"))
                    self._set_phase(self.RETURNING, reason="接近超时：取消前进，受控交回" + self._pilot_mode())
                    if self._handover_ready(now):self._request_mode(self._pilot_mode())
                elif self._ceiling_approach_timeout_requested(now):
                    self.ceiling_approach_timeout_returning = True
                    self._set_phase(self.RETURNING, reason="吸顶故障退出：FAULT锁存，等待POSCTL确认后交回遥控器")
                    if self._handover_ready(now):
                        self._request_mode(self._pilot_mode())
                elif (self.selected_mode == OperatorCommand.MODE_CEILING
                        and getattr(self, "ceiling", None) is not None and is_fresh(self.ceiling_at, self.status_timeout_s, now)
                        and self.ceiling.recovery_attitude_locked and not self._ceiling_handover_confirmed(now)):
                    self.phase_reason = ("顶吸持续XY闭环及高度范围保持；" + (
                        "等待CH12确认交回POSCTL" if self.ceiling.recovery_ready
                        else "实时稳定条件未满足，暂不可交回POSCTL"))
                elif self._ceiling_baro_handover_ready(now):
                    self._set_phase(self.RETURNING, reason=self.ceiling.reason)
                    if self._handover_ready(now):
                        self._request_mode(self._pilot_mode())
                elif ((self.completed or (self.selected_mode == OperatorCommand.MODE_WALL
                                         and getattr(self, "wall_fault_return_ready", False)))
                    and self.owner_seen
                    and (self.owner == 0 or (self.selected_mode == OperatorCommand.MODE_WALL and self.owner == 2))
                    and is_fresh(self.owner_at, self.output_timeout_s, now)):
                    self._set_phase(self.RETURNING, reason=(self._wall_return_reason()
                        if self.selected_mode == OperatorCommand.MODE_WALL else None))
                elif (
                    (not self.owner_seen or (
                        self.selected_mode == OperatorCommand.MODE_CEILING
                        and self.ceiling is not None
                        and is_fresh(self.ceiling_at, self.status_timeout_s, now)
                        and self.ceiling.state in (CeilingAttachmentStatus.STATE_CEILING_ARMED,
                                                   CeilingAttachmentStatus.STATE_NORMAL_FLIGHT)))
                    and monotonic_now - self.phase_since > self.acquire_timeout_s
                ):
                    detail = self._acquire_failure_reason(now)
                    failure_reason = "attachment owner was not acquired: {}".format(
                        detail
                    )
                    rospy.logerr("%s; canceling takeover", failure_reason)
                    if self.selected_mode == OperatorCommand.MODE_CEILING:
                        self._begin_entry_cancel(failure_reason, monotonic_now)
                    else:
                        self._set_phase(self.RETURNING, reason=failure_reason)
        elif self.phase == self.RETURNING:
            if ((getattr(self, "ceiling_approach_timeout_returning", False)
                    or getattr(self,"wall_approach_timeout_returning",False))
                    and (not flight_fresh or self.flight is None or not self.flight.connected)):
                self.phase_reason = "waiting for fresh connected PX4 mode confirmation"
            elif (getattr(self,"wall_approach_timeout_returning",False) and flight_fresh
                    and self.flight is not None and self.flight.connected and flight_mode == self._pilot_mode()):
                self._begin_offboard_exit("接近超时已交回" + self._pilot_mode() + "，清理旧OFFBOARD意图",monotonic_now)
            elif flight_fresh and self.flight.connected and flight_mode == self._pilot_mode():
                self._reset(
                    lock=True,
                    reason=(self._wall_return_reason(confirmed=True) + "；释放请求后可重试"
                        if self.selected_mode == OperatorCommand.MODE_WALL
                        and (self.completed or getattr(self, "wall_fault_return_ready", False))
                        else "returned to {}; release command before retry".format(self._pilot_mode())),
                )
            elif flight_fresh and flight_mode == "OFFBOARD" and self._handover_ready(now):
                self._request_mode(self._pilot_mode())
            elif flight_fresh and flight_mode == "OFFBOARD":
                if getattr(self,"wall_approach_timeout_returning",False):
                    detail=self._wall_timeout_handover_failure(now) or "等待姿态及竖直速度连续安全0.30秒"
                    self.phase_reason="接近超时：受控交回" + self._pilot_mode() + "；" + detail
                elif getattr(self, "ceiling_approach_timeout_returning", False):
                    self.phase_reason = "吸顶故障退出：保持FAULT，等待有效位置/速度估计、遥控输入及连续输出后请求POSCTL"
                elif self._wall_return_requires_exit() and not self._wall_return_exit_ready(now):
                    self.phase_reason = (('姿态稳定条件重新确认中：保持受控输出，等待有效EXIT后交回'
                        if self._captured_recovery() else '离墙条件重新确认中：保持受控输出，等待新的有效EXIT后交回')
                        + self._pilot_mode())
                else:
                    prefix = (self._wall_return_reason() + "；"
                        if getattr(self, "selected_mode", None) == OperatorCommand.MODE_WALL
                        and (self.completed or getattr(self, "wall_fault_return_ready", False)) else "")
                    self.phase_reason = prefix + ('等待姿态持续稳定、有效遥控及新鲜飞控估计后交回'
                        if self._captured_recovery() else '等待水平速度、姿态持续稳定和新鲜有效估计后交回') + self._pilot_mode()
            elif flight_fresh:
                failure_reason = "unexpected flight mode during return: {}".format(
                    flight_mode or "unknown"
                )
                rospy.logwarn("%s", failure_reason)
                self._reset(lock=True, reason=failure_reason)

        self.prestream_pub.publish(
            Bool(data=self.phase in (self.WAITING_WALL, self.PRESTREAM, self.ENTERING, self.CANCELING_ENTRY, self.ACTIVE, self.RETURNING)
                 and (not getattr(self, "offboard_exit_cleanup", False) or flight_mode == "OFFBOARD"))
        )
        self.phase_pub.publish(String(data=self.phase))
        phase_reason_pub = getattr(self, "phase_reason_pub", None)
        if phase_reason_pub is not None:
            phase_reason_pub.publish(
                String(data=getattr(self, "phase_reason", "initialized"))
            )


if __name__ == "__main__":
    rospy.init_node("flight_mode_manager_node")
    from px4_wall_ceiling_control.adapters.async_logging import install_ros_async_logging
    install_ros_async_logging(rospy)
    FlightModeManagerNode()
    rospy.spin()
