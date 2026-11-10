#!/usr/bin/env python3
"""Single MAVROS attitude publisher with bounded, direct RC stick trim."""

import math
import json
import time

import rospy
from geometry_msgs.msg import Quaternion, Vector3Stamped
from mavros_msgs.msg import AttitudeTarget, RCIn, State
from mavros_msgs.srv import ParamGet
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, UInt8, String
from px4_wall_ceiling_control.adapters.kill_switch import install_kill_guard, kill_clear

from px4_wall_ceiling_control.adapters.mavros_setpoint import MavrosSetpointBuilder
from px4_wall_ceiling_control.adapters.odometry import (
    odometry_is_finite,
    odometry_attitude_horizontal_is_finite,
    vertical_speed_enu_mps,
)
from px4_wall_ceiling_control.domain.geometry import (
    euler_from_quaternion, quaternion_multiply, quaternion_normalize,
    quaternion_angle, quaternion_slerp,
)
from px4_wall_ceiling_control.domain.owner_arbiter import (
    CandidateIntent,
    OWNER_CEILING,
    OWNER_NONE,
    OWNER_WALL,
    OwnerArbiter,
)
from px4_wall_ceiling_control.domain.rc_attitude_assist import (
    slew,
    stick_fraction,
    trim_attitude,
)
from px4_wall_ceiling_control.domain.recovery_attitude_limits import (
    limit_recovery_attitude, axis_yaw, attitude_from_axis_yaw)
from px4_wall_ceiling_control.domain.ceiling_motion import CeilingMotionSettings, limit_ceiling_attitude
from px4_wall_ceiling_control.domain.incline_attachment import InclineSettings, InclineAttachment
from px4_wall_ceiling_control.domain.safety import flight_state_ready
from px4_wall_ceiling_control.freshness import is_fresh_receipt as is_fresh, is_fresh_source
from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    CeilingAttachmentStatus,
    CeilingRcAssist,
    OperatorCommand,
    WallPerchStatus,
)


from px4_wall_ceiling_control.adapters.rc_takeover import RCTakeoverOutput
from px4_wall_ceiling_control.adapters.incline_approach_output import InclineApproachOutput


class AttachmentOutputMuxNode(InclineApproachOutput, RCTakeoverOutput):
    OWNER_NONE = OWNER_NONE
    OWNER_CEILING = OWNER_CEILING
    OWNER_WALL = OWNER_WALL

    OUTPUT_NONE = 0
    OUTPUT_ATTITUDE_THRUST = 2

    ATTITUDE_AND_THRUST_MASK = MavrosSetpointBuilder.ATTITUDE_AND_THRUST_MASK

    def __init__(self):
        install_kill_guard(self, rospy, String)
        self.output_enabled = bool(rospy.get_param("~output_enabled", False))
        self.ceiling_output_enabled = bool(
            rospy.get_param("~ceiling_output_enabled", False)
        )
        self.wall_output_enabled = bool(
            rospy.get_param("~wall_output_enabled", False)
        )
        self.require_connected = bool(rospy.get_param("~require_connected", True))
        self.require_armed = bool(rospy.get_param("~require_armed", True))
        self.require_offboard = bool(rospy.get_param("~require_offboard", True))
        self.ceiling_candidate_timeout_s = float(
            rospy.get_param("~ceiling_candidate_timeout_s", 0.20)
        )
        self.wall_candidate_timeout_s = float(
            rospy.get_param("~wall_candidate_timeout_s", 0.20)
        )
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 1.50))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.operator_timeout_s = float(
            rospy.get_param("~operator_timeout_s", 0.50)
        )
        self.handoff_dwell_s = float(
            rospy.get_param("~handoff_dwell_s", 0.50)
        )
        self.max_velocity_xy_mps = float(
            rospy.get_param("~max_velocity_xy_mps", 0.30)
        )
        self.max_velocity_z_mps = float(
            rospy.get_param("~max_velocity_z_mps", 0.30)
        )
        self.min_thrust = float(rospy.get_param("~min_thrust", 0.10))
        self.max_thrust = float(rospy.get_param("~max_thrust", 0.85))
        self.local_frame_id = rospy.get_param("~local_frame_id", "map")
        self.hover_thrust = float(rospy.get_param("~hover_thrust", 0.49))
        self.vertical_speed_kp = float(rospy.get_param("~vertical_speed_kp", 0.15))
        self.odom_twist_in_child_frame = bool(
            rospy.get_param("~odom_twist_in_child_frame", True)
        )
        self.rc_assist_enabled = bool(rospy.get_param("~rc_assist_enabled", False))
        self.rc_roll_channel_index = int(rospy.get_param("~rc_roll_channel_index", -1))
        self.rc_pitch_channel_index = int(rospy.get_param("~rc_pitch_channel_index", -1))
        self.rc_pwm_low = int(rospy.get_param("~rc_pwm_low", 1000))
        self.rc_pwm_center = int(rospy.get_param("~rc_pwm_center", 1500))
        self.rc_pwm_high = int(rospy.get_param("~rc_pwm_high", 2000))
        self.rc_deadzone = float(rospy.get_param("~rc_deadzone", 0.06))
        self.rc_roll_reverse = bool(rospy.get_param("~rc_roll_reverse", False))
        self.rc_pitch_reverse = bool(rospy.get_param("~rc_pitch_reverse", False))
        self.rc_timeout_s = float(rospy.get_param("~rc_timeout_s", 0.20))
        self.rc_slew_deg_s = float(rospy.get_param("~rc_slew_deg_s", 90.0))
        self.rc_free_max_angle_deg = float(
            rospy.get_param("~rc_free_max_angle_deg", 8.0)
        )
        self.rc_contact_max_angle_deg = float(
            rospy.get_param("~rc_contact_max_angle_deg", 2.0)
        )
        self.rc_wall_detach_roll_deg = float(
            rospy.get_param("~rc_wall_detach_roll_deg", 2.0)
        )
        self.rc_wall_detach_pitch_deg = float(
            rospy.get_param("~rc_wall_detach_pitch_deg", 0.0)
        )
        self.rc_roll_yaw_enabled = bool(rospy.get_param("~rc_roll_yaw_enabled", False))
        self.rc_ceiling_roll_pitch_enabled = bool(
            rospy.get_param("~rc_ceiling_roll_pitch_enabled", False)
        )
        self.rc_yaw_max_deg = float(rospy.get_param("~rc_yaw_max_deg", 15.0))
        self.rc_yaw_contact_deg = float(rospy.get_param("~rc_yaw_contact_deg", 3.0))
        self.rc_yaw_detach_deg = float(rospy.get_param("~rc_yaw_detach_deg", 5.0))
        self.rc_yaw_slew_deg_s = float(rospy.get_param("~rc_yaw_slew_deg_s", 20.0))
        self.recovery_attitude_limits_enabled = bool(rospy.get_param("~recovery_attitude_limits_enabled", False))
        self.recovery_max_tilt_deg = float(rospy.get_param("~recovery_max_tilt_deg", 5.0))
        self.recovery_tilt_slew_deg_s = float(rospy.get_param("~recovery_tilt_slew_deg_s", 10.0))
        self.recovery_yaw_slew_deg_s = float(rospy.get_param("~recovery_yaw_slew_deg_s", 20.0))
        self.recovery_final_thrust_cap = float(rospy.get_param("~recovery_final_thrust_cap", 0.60))
        if not (0 < self.recovery_max_tilt_deg < 45 and 0 < self.recovery_tilt_slew_deg_s <= 90
                and 0 < self.recovery_yaw_slew_deg_s <= 90 and 0.1 <= self.recovery_final_thrust_cap <= 1.0):
            raise ValueError("invalid final recovery attitude limits")
        self.approach_attitude_limits_enabled = bool(rospy.get_param("~approach_attitude_limits_enabled", False))
        self.approach_max_tilt_deg = float(rospy.get_param("~approach_max_tilt_deg", 5.0))
        self.approach_tilt_slew_deg_s = float(rospy.get_param("~approach_tilt_slew_deg_s", 10.0))
        self.approach_yaw_slew_deg_s = float(rospy.get_param("~approach_yaw_slew_deg_s", 20.0))
        if not (0 < self.approach_max_tilt_deg <= 5 and 0 < self.approach_tilt_slew_deg_s <= 10
                and 0 < self.approach_yaw_slew_deg_s <= 20):
            raise ValueError("invalid final automatic approach attitude limits")
        self.approach_previous_target = self.approach_previous_at = None
        self.approach_target_publisher = rospy.Publisher("/attachment/approach_target_deg", Vector3Stamped, queue_size=10)
        self.recovery_previous_target = None
        self.recovery_previous_at = None
        self.recovery_target_publisher = rospy.Publisher("/attachment/recovery_target_deg", Vector3Stamped, queue_size=10)
        self.recovery_horizontal_publisher = rospy.Publisher("/attachment/recovery_horizontal_command", Float32, queue_size=10)
        self.preflip_right_roll_deg = float(rospy.get_param("~preflip_right_roll_deg", 0.0))
        self.preflip_roll_slew_deg_s = float(rospy.get_param("~preflip_roll_slew_deg_s", 5.0))
        if not (math.isfinite(self.preflip_right_roll_deg) and 0 <= self.preflip_right_roll_deg <= 5
                and math.isfinite(self.preflip_roll_slew_deg_s) and 0 < self.preflip_roll_slew_deg_s <= 10):
            raise ValueError("invalid preflip right-roll compensation")
        self.preflip_bias_rad = 0.0
        self.preflip_bias_at = None
        self.preflip_bias_publisher = rospy.Publisher("/attachment/preflip_roll_bias_deg", Float32, queue_size=10)
        self.rc_yaw_rad = 0.0
        self.rc_calibration = {}
        self.ceiling_rc_feedback_publisher = rospy.Publisher('/attachment/ceiling_rc_assist', CeilingRcAssist, queue_size=1)
        if self.rc_roll_yaw_enabled or self.rc_ceiling_roll_pitch_enabled:
            if not self.rc_assist_enabled:
                raise ValueError("Calibrated RC correction requires rc_assist_enabled")
            if not (0 <= self.rc_yaw_contact_deg <= self.rc_yaw_detach_deg <= self.rc_yaw_max_deg <= 30
                    and 0 < self.rc_yaw_slew_deg_s <= 45):
                raise ValueError("RC yaw correction limits are invalid")
            self._load_rc_calibration()
        self.status_timeout_s = float(rospy.get_param("~status_timeout_s", 0.30))
        self.match_fcu_thrust_on_handover = bool(
            rospy.get_param("~match_fcu_thrust_on_handover", False)
        )
        self.fcu_target_timeout_s = float(
            rospy.get_param("~fcu_target_timeout_s", 0.20)
        )
        self.thrust_handover_ramp_s = float(
            rospy.get_param("~thrust_handover_ramp_s", 1.0)
        )
        if self.rc_assist_enabled and (
            self.rc_roll_channel_index < 0
            or self.rc_pitch_channel_index < 0
            or self.rc_roll_channel_index == self.rc_pitch_channel_index
            or not self.rc_pwm_low < self.rc_pwm_center < self.rc_pwm_high
            or not 0.0 <= self.rc_deadzone < 1.0
            or self.rc_slew_deg_s <= 0.0
            or not 0.0 <= self.rc_contact_max_angle_deg <= self.rc_free_max_angle_deg <= 15.0
            or not 0.0 <= self.rc_wall_detach_roll_deg <= self.rc_free_max_angle_deg
            or not 0.0 <= self.rc_wall_detach_pitch_deg <= self.rc_free_max_angle_deg
        ):
            raise ValueError("RC assist channels or limits are invalid")
        if not self.min_thrust <= self.hover_thrust <= self.max_thrust:
            raise ValueError("hover_thrust must lie within output thrust limits")
        if self.vertical_speed_kp < 0.0:
            raise ValueError("vertical_speed_kp must be nonnegative")
        if self.fcu_target_timeout_s <= 0.0:
            raise ValueError("fcu_target_timeout_s must be positive")
        if self.thrust_handover_ramp_s <= 0.0:
            raise ValueError("thrust_handover_ramp_s must be positive")

        self.ceiling_motion_enabled = bool(rospy.get_param("/ceiling_attachment_decision_node/motion_control_enabled", False))
        self.ceiling_motion_settings = CeilingMotionSettings.read(
            lambda name, default: rospy.get_param("/ceiling_attachment_decision_node/"+name[1:], default))
        self.ceiling_candidate_snapshot = None
        self.ceiling_last_source = None
        self.ceiling_last_sequence = 0
        self.ceiling_maneuver_id = None
        self.ceiling_retired_maneuvers = set()
        self.ceiling_prestream_attitude = None
        self.ceiling_entry_source = None
        self.ceiling_candidate = None
        self.ceiling_candidate_at = None
        self.wall_candidate = None
        self.wall_candidate_at = None
        self.wall_candidate_snapshot = None
        self.wall_maneuver_id = None
        self.wall_maneuver_closed = False
        self.wall_last_sequence = 0
        self.wall_last_source_stamp = None
        self.wall_retired_maneuvers = set()
        self.wall_candidate_rejection = ""
        self.last_wall_command = None
        # Bound final publication too: the decision timer may run normally
        # while this node alone stalls. Use the same flip thrust slew and the
        # peak angular speed of its quintic trajectory, with a shared dt cap.
        flip_time = float(rospy.get_param("/wall_perch_decision_node/flip_time_s", 1.0))
        flip_angle = abs(float(rospy.get_param("/wall_perch_decision_node/wall_angle_deg", 90.0)))
        self.wall_maneuver_rate_rad_s = math.radians(1.875 * flip_angle / max(.01, flip_time))
        self.wall_maneuver_thrust_slew_per_s = float(rospy.get_param(
            "/wall_perch_decision_node/flip_thrust_slew_per_s", .50))
        detach_time = float(rospy.get_param("/wall_perch_decision_node/detach_time_s", 3.0))
        self.wall_detach_rate_rad_s = math.radians(1.875 * flip_angle / max(.01, detach_time))
        self.wall_return_thrust_slew_per_s = float(rospy.get_param(
            "/wall_perch_decision_node/return_thrust_slew_per_s", .50))
        if not (math.isfinite(self.wall_maneuver_rate_rad_s) and 0 < self.wall_maneuver_rate_rad_s
                and math.isfinite(self.wall_maneuver_thrust_slew_per_s)
                and 0 < self.wall_maneuver_thrust_slew_per_s <= 1.0
                and math.isfinite(self.wall_detach_rate_rad_s) and self.wall_detach_rate_rad_s > 0
                and math.isfinite(self.wall_return_thrust_slew_per_s)
                and 0 < self.wall_return_thrust_slew_per_s <= 1.0):
            raise ValueError("invalid wall final publication slew bounds")
        self.last_published_control = None
        self.wall_publication_slew = None
        self.flight_state = None
        self.flight_state_at = None
        self.odom = None
        self.odom_at = None
        self.rc_channels = None
        self.rc_at = None
        self.rc_roll_rad = 0.0
        self.rc_pitch_rad = 0.0
        self.rc_trim_at = None
        self.ceiling_status = None
        self.ceiling_status_at = None
        self.wall_status = None
        self.wall_status_at = None
        self.operator = None
        self.operator_at = None
        self.prestream_requested = False
        self.prestream_at = None
        self.prestream_timeout_s = float(rospy.get_param("~prestream_timeout_s", 0.30))
        self.fcu_attitude_target = None
        self.fcu_attitude_target_at = None
        self.wall_continuity_fcu_snapshot = None
        self.handover_start_thrust = None
        self.handover_started_at = None
        self.manager_phase = ""
        self.manager_phase_at = None
        self.cancel_vertical_latched = False
        self.cancel_vertical_reference = None
        self.cancel_vertical_telemetry = None
        self.owner = self.OWNER_NONE
        self.interlock_active = False
        self.interlock_clear_since = None
        self.owner_arbiter = OwnerArbiter(self.handoff_dwell_s)
        self.setpoint_builder = MavrosSetpointBuilder(
            self.max_velocity_z_mps,
            self.min_thrust,
            self.max_thrust,
            self.local_frame_id,
        )

        self.attitude_publisher = rospy.Publisher(
            rospy.get_param(
                "~attitude_setpoint_topic", "/mavros/setpoint_raw/attitude"
            ),
            AttitudeTarget,
            queue_size=10,
        )
        self.owner_publisher = rospy.Publisher(
            rospy.get_param("~owner_topic", "/attachment/output_owner"),
            UInt8,
            queue_size=10,
            latch=True,
        )
        self.mode_publisher = rospy.Publisher(
            rospy.get_param("~output_mode_topic", "/attachment/output_mode"),
            UInt8,
            queue_size=10,
            latch=True,
        )
        self.rc_age_publisher = rospy.Publisher(
            rospy.get_param("~rc_age_topic", "/attachment/rc_input_age_ms"),
            Float32,
            queue_size=1,
        )
        self.rc_preview_publisher = rospy.Publisher(
            rospy.get_param("~rc_preview_topic", "/attachment/rc_preview_deg"),
            Vector3Stamped,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~ceiling_candidate_topic",
                "/ceiling_attachment/control_candidate",
            ),
            AttachmentControlCandidate,
            self._on_ceiling_candidate,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~operator_topic", "/operator/command"),
            OperatorCommand,
            self._on_operator,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~wall_candidate_topic", "/wall_perch/control_candidate"
            ),
            AttachmentControlCandidate,
            self._on_wall_candidate,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~state_topic", "/mavros/state"),
            State,
            self._on_state,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/mavros/local_position/odom"),
            Odometry,
            self._on_odom,
            queue_size=10,
        )
        if self.match_fcu_thrust_on_handover:
            rospy.Subscriber(
                rospy.get_param(
                    "~fcu_attitude_target_topic",
                    "/mavros/setpoint_raw/target_attitude",
                ),
                AttitudeTarget,
                self._on_fcu_attitude_target,
                queue_size=10,
            )
        # Subscribe independently of optional attitude trim for explicit takeover.
        rospy.Subscriber(
            rospy.get_param("~rc_topic", "/mavros/rc/in"),
            RCIn,
            self._on_rc,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~ceiling_status_topic", "/ceiling_attachment/status"),
            CeilingAttachmentStatus,
            self._on_ceiling_status,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~wall_status_topic", "/wall_perch/status"),
            WallPerchStatus,
            self._on_wall_status,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~prestream_topic", "/attachment/prestream"),
            Bool,
            self._on_prestream,
            queue_size=10,
        )
        rospy.Subscriber("/attachment/flight_phase", String,
                         self._on_manager_phase, queue_size=1)
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.output_health_publisher = rospy.Publisher("/attachment/output_health", String, queue_size=10)
        self.output_health_last_key = None
        self.output_health_last_at = None
        self._init_rc_takeover()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 2.0)), self._on_timer
        )

        if not self.output_enabled:
            rospy.logwarn("combined attachment MAVROS output is disabled")

    def _on_ceiling_candidate(self, message):
        if message.source != AttachmentControlCandidate.SOURCE_CEILING:
            rospy.logerr_throttle(1.0, "invalid source on ceiling candidate topic")
            return
        now = rospy.Time.now()
        if getattr(self, "ceiling_motion_enabled", False):
            if not is_fresh_source(message.header.stamp, self.ceiling_candidate_timeout_s, now, future_tolerance_s=0.):
                return
            if message.keep_ownership:
                if (not message.phase_valid or not message.maneuver_id
                        or message.maneuver_id in self.ceiling_retired_maneuvers
                        or not math.isfinite(message.ceiling_yaw_reference_rad)):
                    return
                if self.ceiling_maneuver_id != message.maneuver_id:
                    if self.ceiling_maneuver_id:
                        self.ceiling_retired_maneuvers.add(self.ceiling_maneuver_id)
                    self.ceiling_maneuver_id = message.maneuver_id
                    self.ceiling_last_sequence = 0
                if (message.control_sequence <= self.ceiling_last_sequence
                        or (self.ceiling_last_source is not None and message.header.stamp <= self.ceiling_last_source)):
                    return
                self.ceiling_last_sequence = message.control_sequence
                self.ceiling_last_source = message.header.stamp
            elif self.ceiling_maneuver_id:
                if (message.maneuver_id != self.ceiling_maneuver_id
                        or message.control_sequence <= self.ceiling_last_sequence
                        or not (message.control_phase == CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
                                or (message.control_phase == CeilingAttachmentStatus.STATE_ABORT
                                    and message.ceiling_abort_land_requested))):
                    return
                self.ceiling_retired_maneuvers.add(self.ceiling_maneuver_id)
                self.ceiling_maneuver_id = None
        self.ceiling_candidate = message
        self.ceiling_candidate_at = now
        self.ceiling_candidate_snapshot = (message, now)

    def _wall_candidate_problem(self, message, now):
        if message is not None and getattr(message, 'incline_active', False):
            settings = self._incline_output_settings()
            if settings.error(self.max_thrust):
                return 'incline_configuration_not_ready'
            if (message.kind != AttachmentControlCandidate.KIND_ATTITUDE_THRUST
                    or not math.isfinite(message.thrust_normalized)
                    or message.thrust_normalized > settings.flip_thrust_max + 1e-6):
                return 'invalid_incline_output'
        operator = getattr(self, 'operator', None)
        if message is not None and message.acquire_request and operator is not None:
            if (operator.mode == OperatorCommand.MODE_INCLINE) != bool(getattr(message, 'incline_active', False)):
                return 'incline_profile_acquisition_mismatch'
        if message is None or message.source != AttachmentControlCandidate.SOURCE_WALL:
            return "wrong_source"
        if not (getattr(message, "phase_valid", False)
                and WallPerchStatus.STATE_IDLE <= message.control_phase <= WallPerchStatus.STATE_ABORT
                and bool(getattr(message, "maneuver_id", ""))
                and len(message.maneuver_id) <= 128
                and getattr(message, "control_sequence", 0) > 0):
            return "missing_or_invalid_control_metadata"
        if not is_fresh_source(message.header.stamp,
                               getattr(self, "wall_candidate_timeout_s", .20),
                               now, future_tolerance_s=0.0):
            return "source_stamp_stale_zero_or_future"
        if getattr(message, "recovery_support_valid", False):
            cap = getattr(message, "recovery_thrust_limit", float("nan"))
            if (not math.isfinite(cap)
                    or not self.min_thrust <= cap <= self.max_thrust + 1e-6):
                return "invalid_recovery_support_limit"
        if getattr(message, 'range_height_control', False) and (
                message.kind != AttachmentControlCandidate.KIND_ATTITUDE_THRUST
                or message.detach_range_episode <= 0
                or message.detach_task_phase not in (2, 3, 4)
                or message.control_phase not in (WallPerchStatus.STATE_DETACH_ROTATE,
                    WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_EXIT, WallPerchStatus.STATE_ABORT)):
            return 'range_phase_payload_mismatch'
        if getattr(message, 'departure_support_only', False) and (
                message.kind != AttachmentControlCandidate.KIND_ATTITUDE_THRUST
                or message.detach_range_episode <= 0
                or message.detach_task_phase not in (1, 4)
                or getattr(message, 'range_height_control', False)
                or message.control_phase not in (WallPerchStatus.STATE_DETACH_ROTATE,
                    WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_EXIT, WallPerchStatus.STATE_ABORT)):
            return 'departure_support_payload_mismatch'
        return ""

    def _retire_wall_maneuver(self):
        maneuver = getattr(self, "wall_maneuver_id", None)
        retired = getattr(self, "wall_retired_maneuvers", set())
        if maneuver:
            retired.add(maneuver)
        self.wall_retired_maneuvers = retired
        self.wall_maneuver_id = None
        self.wall_maneuver_closed = False
        self.wall_last_sequence = 0
        self.wall_last_source_stamp = None
        self.wall_candidate_snapshot = None
        self.wall_candidate = self.wall_candidate_at = None
        self.last_published_control = None
        self.wall_continuity_fcu_snapshot = None

    def _on_wall_candidate(self, message):
        now = rospy.Time.now()
        reason = self._wall_candidate_problem(message, now)
        maneuver = getattr(message, "maneuver_id", "")
        current = getattr(self, "wall_maneuver_id", None)
        retired = getattr(self, "wall_retired_maneuvers", set())
        if not reason and maneuver in retired:
            reason = "retired_maneuver"
        if not reason and maneuver == current:
            last_stamp = getattr(self, "wall_last_source_stamp", None)
            if (message.control_sequence <= getattr(self, "wall_last_sequence", 0)
                    or (last_stamp is not None and message.header.stamp <= last_stamp)):
                reason = "repeated_or_reordered_candidate"
            elif getattr(self, "wall_maneuver_closed", False) and message.keep_ownership:
                reason = "closed_maneuver_cannot_reacquire"
        if not reason and maneuver != current:
            previous = getattr(self, "wall_candidate", None)
            active = (getattr(self, "owner", self.OWNER_NONE) == self.OWNER_WALL
                      or (previous is not None and previous.keep_ownership
                          and not getattr(self, "wall_maneuver_closed", False)))
            acquisition = (message.acquire_request and message.control_phase in (
                WallPerchStatus.STATE_FRONT_WALL_DETECT, WallPerchStatus.STATE_STABILIZE_HOVER,
                WallPerchStatus.STATE_SLOW_APPROACH, WallPerchStatus.STATE_FLIP_TO_WALL))
            idle = (message.control_phase == WallPerchStatus.STATE_IDLE
                    and not message.acquire_request and not message.keep_ownership)
            if active:
                reason = "new_maneuver_while_previous_active"
            elif not (acquisition or idle):
                reason = "new_maneuver_without_acquisition"
            else:
                if current:
                    retired.add(current)
                self.wall_retired_maneuvers = retired
                self.wall_maneuver_id = maneuver
                self.wall_maneuver_closed = False
        self.wall_candidate_rejection = reason
        if reason:
            rospy.logwarn_throttle(1.0, "wall candidate rejected: %s", reason)
            return
        self.wall_last_sequence = message.control_sequence
        self.wall_last_source_stamp = message.header.stamp
        if not message.keep_ownership:
            self.wall_maneuver_closed = True
        self.wall_candidate = message
        self.wall_candidate_at = now
        # Publish an atomic pair: timer and phase helpers must never pair a
        # newly received message with an earlier message's receipt timestamp.
        self.wall_candidate_snapshot = (message, now)

    def _wall_control(self, now, candidate=None):
        if candidate is None:
            snapshot = getattr(self, "wall_candidate_snapshot", None)
            candidate = snapshot[0] if snapshot is not None else getattr(self, "wall_candidate", None)
        return candidate if not self._wall_candidate_problem(candidate, now) else None

    def _on_state(self, message):
        now = rospy.Time.now()
        previous_mode = (
            self.flight_state.mode.upper() if self.flight_state is not None else ""
        )
        next_mode = message.mode.upper()
        previous = getattr(self, "flight_state", None)
        if previous is not None and ((previous.mode.upper() == "OFFBOARD" and next_mode != "OFFBOARD")
                                     or (previous.armed and not message.armed)):
            self._retire_wall_maneuver()
        if next_mode != "OFFBOARD" or not message.armed or previous_mode != next_mode:
            self._reset_cancel_vertical()
        if previous_mode != next_mode and "OFFBOARD" in (previous_mode, next_mode):
            self.rc_roll_rad = self.rc_pitch_rad = self.rc_yaw_rad = 0.0
            self.rc_trim_at = now
            self.rc_trim_axes = ()
        if getattr(self, "match_fcu_thrust_on_handover", False):
            if previous_mode != "OFFBOARD" and next_mode == "OFFBOARD":
                entry = getattr(self, "ceiling_entry_source", None)
                if (getattr(self, "ceiling_motion_enabled", False) and entry is not None
                        and self._operator_mode(now) == OperatorCommand.MODE_CEILING
                        and is_fresh(entry[2], self.ceiling_motion_settings.motion_capture_timeout_s, now)):
                    self.handover_start_thrust = entry[1]
                    self.handover_started_at = None
                elif self._handover_source_ready(now):
                    self.handover_start_thrust = float(
                        self.fcu_attitude_target.thrust
                    )
                    # Start the ramp only when the behavior candidate first owns
                    # the output. Until then prestream holds this exact thrust.
                    self.handover_started_at = None
                    rospy.loginfo(
                        "captured pilot-mode handover thrust %.3f",
                        self.handover_start_thrust,
                    )
                else:
                    self.handover_start_thrust = None
                    self.handover_started_at = None
                    rospy.logerr(
                        "OFFBOARD entered without a fresh bounded FCU thrust target"
                    )
            elif next_mode != "OFFBOARD":
                self.handover_start_thrust = None
                self.handover_started_at = None
        self.flight_state = message
        self.flight_state_at = now

    def _on_fcu_attitude_target(self, message):
        # Once OFFBOARD is active this topic reflects the external command.
        # Keep that echo separate from the pilot-mode handover baseline. It is
        # used only to resume bounded publication after an output interruption.
        if self.flight_state is not None and self.flight_state.mode.upper() == "OFFBOARD":
            self.wall_continuity_fcu_snapshot = (message, rospy.Time.now())
            return
        if (getattr(self, "ceiling_motion_enabled", False) and getattr(self, "ceiling_entry_source", None) is not None
                and getattr(self, "manager_phase", '') in ('ENTERING', 'ACTIVE', 'RETURNING')):
            return
        self.fcu_attitude_target = message
        self.fcu_attitude_target_at = rospy.Time.now()

    def _on_odom(self, message):
        self.odom = message
        self.odom_at = rospy.Time.now()

    def _load_rc_calibration(self):
        # Fail startup rather than guess stick channels or direction.
        rospy.wait_for_service("/mavros/param/get", timeout=5.0)
        get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
        deadline = time.monotonic() + 20.0
        def read(name, integer=False):
            while not rospy.is_shutdown():
                response = get(param_id=name)
                if response.success:
                    return response.value.integer if integer else response.value.real
                if time.monotonic() >= deadline:
                    raise ValueError("RC calibration unavailable after parameter-cache wait: " + name)
                rospy.logwarn_throttle(2.0, "Waiting for MAVROS RC parameter cache: %s", name)
                rospy.sleep(.25)
            raise ValueError("RC calibration canceled during shutdown")
        axes = ["roll"]
        if getattr(self, "rc_roll_yaw_enabled", False):
            axes.append("yaw")
        if getattr(self, "rc_ceiling_roll_pitch_enabled", False):
            axes.append("pitch")
        for axis in axes:
            channel = int(read("RC_MAP_" + axis.upper(), True))
            if channel <= 0:
                raise ValueError("RC " + axis + " channel is not mapped")
            low, center, high, reverse = (read("RC%d_%s" % (channel, suffix))
                                         for suffix in ("MIN", "TRIM", "MAX", "REV"))
            if not (all(math.isfinite(v) for v in (low, center, high, reverse))
                    and low < center < high and reverse in (-1, 1)):
                raise ValueError("Invalid RC " + axis + " calibration")
            self.rc_calibration[axis] = (channel - 1, low, center, high, reverse < 0)
        if len({self.rc_calibration[axis][0] for axis in axes}) != len(axes):
            raise ValueError("Calibrated RC axes require distinct channels")
        self.rc_roll_channel_index = self.rc_calibration["roll"][0]
        # Keep legacy validation valid, even when only roll/yaw assist is used.
        second = "pitch" if "pitch" in axes else "yaw"
        self.rc_pitch_channel_index = self.rc_calibration[second][0]
        rospy.loginfo("Calibrated RC correction channels: %s", ", ".join(
            "%s=%d" % (axis, self.rc_calibration[axis][0] + 1) for axis in axes
        ))

    def _calibrated_rc_axes(self, now, source):
        ceiling = source == AttachmentControlCandidate.SOURCE_CEILING
        if source == AttachmentControlCandidate.SOURCE_NONE:
            ceiling = self._operator_mode(now) == OperatorCommand.MODE_CEILING
        if ceiling and getattr(self, "rc_ceiling_roll_pitch_enabled", False):
            return ("roll", "pitch")
        if getattr(self, "rc_roll_yaw_enabled", False):
            return ("roll", "yaw")
        return ()

    def _calibrated_rc_fraction(self, axis):
        index, low, center, high, reverse = self.rc_calibration[axis]
        value = stick_fraction(self.rc_channels, index, low, center, high, self.rc_deadzone, reverse)
        # PX4 yaw is clockwise in NED; ROS heading is counterclockwise in ENU.
        # Calibrated positive pitch stick means forward/nose-down: positive
        # body-FLU pitch, so pitch needs no additional inversion after RCn_REV.
        return -value if axis == "yaw" and value is not None else value

    def _roll_yaw_limits(self, now, source, candidate=None):
        roll = math.radians(self.rc_free_max_angle_deg)
        yaw = math.radians(self.rc_yaw_max_deg)
        if source == AttachmentControlCandidate.SOURCE_WALL:
            control = self._wall_control(now, candidate)
            if control is None:
                return 0.0, 0.0
            state = control.control_phase
            if state in (WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE,
                         WallPerchStatus.STATE_WALL_HOLD, WallPerchStatus.STATE_WALL_PIN,
                         WallPerchStatus.STATE_DETACH_ROTATE):
                # The wall maneuver owns these axes; no RC trim during flip-to-detach.
                return 0.0, 0.0
        return roll, yaw

    def _calibrated_rc_attitude(self, now, quaternion, source, candidate=None):
        axes = self._calibrated_rc_axes(now, source)
        # Do not change ALTCTL prestream targets or retain a trim across handovers.
        flight = getattr(self, "flight_state", None)
        if (flight is None or not is_fresh(self.flight_state_at, self.state_timeout_s, now)
                or flight.mode.upper() != "OFFBOARD"):
            self.rc_roll_rad = self.rc_pitch_rad = self.rc_yaw_rad = 0.0
            self.rc_trim_at = now
            self.rc_trim_axes = axes
            return quaternion
        if getattr(self, "rc_trim_axes", axes) != axes:
            # Never carry a wall yaw trim into ceiling flight (or pitch back to wall).
            self.rc_roll_rad = self.rc_pitch_rad = self.rc_yaw_rad = 0.0
            self.rc_trim_at = now
        self.rc_trim_axes = axes
        if axes == ("roll", "pitch"):
            limits = self._rc_limits(now, AttachmentControlCandidate.SOURCE_CEILING, candidate)
        else:
            limits = self._roll_yaw_limits(now, source, candidate)
        ready = self._rc_ready(now, source)
        if not ready:
            rospy.logwarn_throttle(1.0, "Calibrated RC unavailable; correction returning to zero")
        self.rc_age_publisher.publish(Float32(data=(now-self.rc_at).to_sec()*1000 if ready else -1.0))
        elapsed = max(0.0, min((now-self.rc_trim_at).to_sec() if self.rc_trim_at is not None else .02, .1))
        applied = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        for axis, limit in zip(axes, limits):
            desired = self._calibrated_rc_fraction(axis) * limit if ready else 0.0
            previous = getattr(self, "rc_" + axis + "_rad")
            rate = self.rc_yaw_slew_deg_s if axis == "yaw" else self.rc_slew_deg_s
            applied[axis] = slew(max(-limit, min(previous, limit)), desired,
                                 math.radians(rate) * elapsed)
        self.rc_roll_rad, self.rc_pitch_rad, self.rc_yaw_rad = (
            applied["roll"], applied["pitch"], applied["yaw"]
        )
        self.rc_trim_at = now
        # Reuse the body-axis trim; world-up yaw is used only for wall assistance.
        yaw_q = (0, 0, math.sin(self.rc_yaw_rad/2), math.cos(self.rc_yaw_rad/2))
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        values = quaternion_normalize(quaternion_multiply(
            yaw_q, trim_attitude(values, self.rc_roll_rad, self.rc_pitch_rad)
        ))
        preview = Vector3Stamped()
        preview.header.stamp = now
        preview.vector.x = math.degrees(self.rc_roll_rad)
        preview.vector.y = math.degrees(self.rc_pitch_rad)
        preview.vector.z = math.degrees(self.rc_yaw_rad)
        self.rc_preview_publisher.publish(preview)
        return Quaternion(x=values[0], y=values[1], z=values[2], w=values[3])

    def _on_rc(self, message):
        self.rc_source_stamp = message.header.stamp
        self.rc_channels = tuple(message.channels)
        self.rc_at = rospy.Time.now()
        if (getattr(self, "rc_roll_yaw_enabled", False)
                or getattr(self, "rc_ceiling_roll_pitch_enabled", False)):
            return  # Publish applied correction from the output path, not raw sticks.
        roll = stick_fraction(
            self.rc_channels, self.rc_roll_channel_index, self.rc_pwm_low,
            self.rc_pwm_center, self.rc_pwm_high, self.rc_deadzone,
            self.rc_roll_reverse,
        )
        pitch = stick_fraction(
            self.rc_channels, self.rc_pitch_channel_index, self.rc_pwm_low,
            self.rc_pwm_center, self.rc_pwm_high, self.rc_deadzone,
            self.rc_pitch_reverse,
        )
        if roll is not None and pitch is not None:
            preview = Vector3Stamped()
            preview.header.stamp = self.rc_at
            preview.vector.x = roll * self.rc_free_max_angle_deg
            preview.vector.y = pitch * self.rc_free_max_angle_deg
            self.rc_preview_publisher.publish(preview)

    def _on_ceiling_status(self, message):
        self.ceiling_status = message
        self.ceiling_status_at = rospy.Time.now()

    def _on_wall_status(self, message):
        self.wall_status = message
        self.wall_status_at = rospy.Time.now()

    def _on_operator(self, message):
        self.operator = message
        self.operator_at = rospy.Time.now()

    def _on_prestream(self, message):
        self.prestream_requested = bool(message.data)
        self.prestream_at = rospy.Time.now()

    def _ceiling_fresh(self, now):
        return bool(
            getattr(self, "ceiling_output_enabled", False)
            and is_fresh(
                getattr(self, "ceiling_candidate_at", None),
                getattr(self, "ceiling_candidate_timeout_s", 0.20),
                now,
            )
            and getattr(self, "ceiling_candidate", None) is not None
            and (not getattr(self, "ceiling_motion_enabled", False) or is_fresh_source(
                self.ceiling_candidate.header.stamp, self.ceiling_candidate_timeout_s, now, future_tolerance_s=0.))
        )

    def _wall_snapshot_fresh(self, snapshot, now):
        candidate, received = snapshot if snapshot is not None else (None, None)
        return bool(getattr(self, "wall_output_enabled", False)
                    and is_fresh(received, getattr(self, "wall_candidate_timeout_s", .20), now)
                    and not self._wall_candidate_problem(candidate, now))

    def _wall_fresh(self, now):
        return self._wall_snapshot_fresh(getattr(self, "wall_candidate_snapshot", None), now)

    def _operator_mode(self, now):
        if not is_fresh(
            getattr(self, "operator_at", None),
            getattr(self, "operator_timeout_s", 0.50),
            now,
        ):
            return OperatorCommand.MODE_ABORT
        operator = getattr(self, "operator", None)
        if operator is None or not operator.enable_control:
            return OperatorCommand.MODE_STANDBY
        return OperatorCommand.MODE_WALL if operator.mode == OperatorCommand.MODE_INCLINE else operator.mode

    def _operator_neutral(self, now):
        return self._operator_mode(now) in (
            OperatorCommand.MODE_MANUAL,
            OperatorCommand.MODE_STANDBY,
        )

    @staticmethod
    def _time_to_sec(value):
        if value is None:
            return None
        return value.to_sec() if hasattr(value, "to_sec") else float(value)

    def _ensure_owner_arbiter(self):
        arbiter = getattr(self, "owner_arbiter", None)
        handoff_dwell_s = getattr(self, "handoff_dwell_s", 0.50)
        if arbiter is None:
            arbiter = OwnerArbiter(handoff_dwell_s)
            self.owner_arbiter = arbiter
        arbiter.handoff_dwell_s = max(0.0, float(handoff_dwell_s))
        arbiter.restore(
            self.owner,
            self.interlock_active,
            self._time_to_sec(self.interlock_clear_since),
        )
        return arbiter

    def _sync_arbiter_state(self, arbiter):
        self.owner = arbiter.owner
        self.interlock_active = arbiter.interlock_active
        self.interlock_clear_since = (
            None
            if arbiter.interlock_clear_since_s is None
            else rospy.Time.from_sec(arbiter.interlock_clear_since_s)
        )

    def _candidate_intent(self, candidate, fresh, enabled, required_mode):
        return CandidateIntent(
            enabled=enabled,
            fresh=fresh,
            acquire_request=bool(candidate and candidate.acquire_request),
            keep_ownership=bool(candidate and candidate.keep_ownership),
            candidate_valid=bool(candidate and candidate.valid),
            required_operator_mode=required_mode,
        )

    def _update_owner(self, now):
        arbiter = self._ensure_owner_arbiter()
        operator_mode = self._operator_mode(now)
        wall_snapshot = getattr(self, "wall_candidate_snapshot", None)
        event = arbiter.update(
            self._time_to_sec(now),
            operator_mode,
            self._operator_neutral(now),
            self._candidate_intent(
                getattr(self, "ceiling_candidate", None),
                self._ceiling_fresh(now),
                getattr(self, "ceiling_output_enabled", False),
                OperatorCommand.MODE_CEILING,
            ),
            self._candidate_intent(
                wall_snapshot[0] if wall_snapshot is not None else None,
                self._wall_snapshot_fresh(wall_snapshot, now),
                getattr(self, "wall_output_enabled", False),
                OperatorCommand.MODE_WALL,
            ),
        )
        self._sync_arbiter_state(arbiter)
        if event.conflict:
            rospy.logerr_throttle(
                1.0, "attachment output conflict: both decisions request ownership"
            )
        if event.interlock_started:
            rospy.logwarn("attachment owner interlock: %s", event.reason)
        if event.interlock_cleared:
            rospy.loginfo("attachment owner interlock cleared")

    def _flight_ready(self, now):
        if not kill_clear(self, now):
            return False
        ready, _reason = flight_state_ready(
            self.flight_state,
            is_fresh(self.flight_state_at, self.state_timeout_s, now),
            self.require_connected,
            self.require_armed,
            self.require_offboard,
        )
        return ready

    def _odom_fresh(self, now):
        if self.odom is None or not is_fresh(
            self.odom_at, self.odom_timeout_s, now
        ):
            return False
        if getattr(self, "owner", self.OWNER_NONE) == self.OWNER_WALL and not is_fresh_source(
                self.odom.header.stamp, self.odom_timeout_s, now):
            return False
        snapshot = getattr(self, 'wall_candidate_snapshot', None)
        candidate = snapshot[0] if snapshot is not None else None
        if (getattr(self, 'owner', self.OWNER_NONE) == self.OWNER_WALL
                and candidate is not None and (getattr(candidate, 'range_height_control', False)
                                               or getattr(candidate, 'departure_support_only', False))
                and self._wall_fresh(now)):
            return odometry_attitude_horizontal_is_finite(self.odom, self.odom_twist_in_child_frame)
        return odometry_is_finite(self.odom)

    def _get_setpoint_builder(self):
        builder = getattr(self, "setpoint_builder", None)
        if builder is None:
            builder = MavrosSetpointBuilder(
                self.max_velocity_z_mps,
                self.min_thrust,
                self.max_thrust,
                self.local_frame_id,
            )
            self.setpoint_builder = builder
        return builder

    def _rc_ready(self, now, source=AttachmentControlCandidate.SOURCE_NONE):
        if not getattr(self, "rc_assist_enabled", False):
            return True
        if not is_fresh(self.rc_at, self.rc_timeout_s, now) or self.rc_channels is None:
            return False
        axes = self._calibrated_rc_axes(now, source)
        if axes:
            return all(self._calibrated_rc_fraction(axis) is not None for axis in axes)
        return all(
            stick_fraction(
                self.rc_channels, index, self.rc_pwm_low, self.rc_pwm_center,
                self.rc_pwm_high, self.rc_deadzone, reverse,
            ) is not None
            for index, reverse in (
                (self.rc_roll_channel_index, self.rc_roll_reverse),
                (self.rc_pitch_channel_index, self.rc_pitch_reverse),
            )
        )

    def _rc_limits(self, now, source, candidate=None):
        free = math.radians(self.rc_free_max_angle_deg)
        contact = math.radians(self.rc_contact_max_angle_deg)
        if source == AttachmentControlCandidate.SOURCE_CEILING:
            if candidate is not None and candidate.phase_valid:
                state = candidate.control_phase
                landing_abort = candidate.ceiling_abort_land_requested
            else:
                if not is_fresh(self.ceiling_status_at, self.status_timeout_s, now):
                    return 0.0, 0.0
                state = self.ceiling_status.state
                landing_abort = self.ceiling_status.abort_land_requested
            if state in (
                CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
                CeilingAttachmentStatus.STATE_SURFACE_HOLD,
            ) or (state == CeilingAttachmentStatus.STATE_ABORT and landing_abort):
                return contact, contact
            if state in (
                CeilingAttachmentStatus.STATE_APPROACH,
                CeilingAttachmentStatus.STATE_DETACH,
                CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
            ):
                return free, free
            return 0.0, 0.0
        if source == AttachmentControlCandidate.SOURCE_WALL:
            control = self._wall_control(now, candidate)
            if control is None:
                return 0.0, 0.0
            state = control.control_phase
            if state in (
                WallPerchStatus.STATE_FRONT_WALL_DETECT,
                WallPerchStatus.STATE_STABILIZE_HOVER,
                WallPerchStatus.STATE_RECOVER,
            ):
                return free, free
            if state == WallPerchStatus.STATE_SLOW_APPROACH:
                return free, contact
            if state == WallPerchStatus.STATE_DETACH_ROTATE:
                return math.radians(self.rc_wall_detach_roll_deg), math.radians(
                    self.rc_wall_detach_pitch_deg
                )
            return contact, contact
        return free, free

    def _assisted_attitude(self, now, quaternion, source, candidate=None):
        if (source == AttachmentControlCandidate.SOURCE_CEILING and candidate is not None
                and candidate.ceiling_rc_assist_blocked):
            self.rc_roll_rad = self.rc_pitch_rad = self.rc_yaw_rad = 0.
            self.rc_trim_at = now
            return quaternion
        if not getattr(self, "rc_assist_enabled", False):
            return self._preflip_bias_attitude(now, quaternion, source, candidate)
        if self._calibrated_rc_axes(now, source):
            return self._preflip_bias_attitude(
                now, self._calibrated_rc_attitude(now, quaternion, source, candidate), source, candidate
            )
        limits = self._rc_limits(now, source, candidate)
        if not self._rc_ready(now):
            rospy.logwarn_throttle(1.0, "RC attitude assist unavailable; trim returning to zero")
            self.rc_age_publisher.publish(Float32(data=-1.0))
            desired_roll = desired_pitch = 0.0
        else:
            channels = self.rc_channels
            desired_roll = stick_fraction(
                channels, self.rc_roll_channel_index, self.rc_pwm_low,
                self.rc_pwm_center, self.rc_pwm_high, self.rc_deadzone,
                self.rc_roll_reverse,
            ) * limits[0]
            desired_pitch = stick_fraction(
                channels, self.rc_pitch_channel_index, self.rc_pwm_low,
                self.rc_pwm_center, self.rc_pwm_high, self.rc_deadzone,
                self.rc_pitch_reverse,
            ) * limits[1]
            self.rc_age_publisher.publish(
                Float32(data=(now - self.rc_at).to_sec() * 1000.0)
            )
        elapsed = (
            (now - self.rc_trim_at).to_sec()
            if self.rc_trim_at is not None else 0.02
        )
        step = math.radians(self.rc_slew_deg_s) * max(0.0, min(elapsed, 0.1))
        # A phase change may tighten the allowed trim immediately.
        self.rc_roll_rad = max(-limits[0], min(self.rc_roll_rad, limits[0]))
        self.rc_pitch_rad = max(-limits[1], min(self.rc_pitch_rad, limits[1]))
        self.rc_roll_rad = slew(self.rc_roll_rad, desired_roll, step)
        self.rc_pitch_rad = slew(self.rc_pitch_rad, desired_pitch, step)
        self.rc_trim_at = now
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        x, y, z, w = trim_attitude(values, self.rc_roll_rad, self.rc_pitch_rad)
        return self._preflip_bias_attitude(now, Quaternion(x=x, y=y, z=z, w=w), source, candidate)

    def _preflip_bias_attitude(self, now, quaternion, source, candidate=None):
        # ROS body FLU: positive roll tilts upward thrust toward body right (-Y).
        flight = getattr(self, "flight_state", None)
        status = getattr(self, "wall_status", None)
        control = self._wall_control(now, candidate) if source == AttachmentControlCandidate.SOURCE_WALL else None
        state = control.control_phase if control is not None else None
        metadata_fresh = control is not None
        horizontal_enabled = bool(control and control.horizontal_velocity_enabled)
        paused = bool(control and control.approach_paused)
        if source == AttachmentControlCandidate.SOURCE_NONE:
            metadata_fresh = status is not None and is_fresh(self.wall_status_at, self.status_timeout_s, now)
            state = status.state if metadata_fresh else None
            horizontal_enabled = bool(status and getattr(status, "horizontal_velocity_enabled", False))
            paused = bool(status and getattr(status, "approach_paused", False))
        allowed = bool(
            getattr(self, "preflip_right_roll_deg", 0.0) > 0
            and source in (AttachmentControlCandidate.SOURCE_NONE, AttachmentControlCandidate.SOURCE_WALL)
            and flight is not None and flight.connected and flight.armed and flight.mode.upper() == "OFFBOARD"
            and is_fresh(self.flight_state_at, self.state_timeout_s, now)
            and self._operator_mode(now) == OperatorCommand.MODE_WALL
            and metadata_fresh and state in (WallPerchStatus.STATE_IDLE, WallPerchStatus.STATE_FRONT_WALL_DETECT,
                                            WallPerchStatus.STATE_STABILIZE_HOVER, WallPerchStatus.STATE_SLOW_APPROACH)
            and not horizontal_enabled)
        if getattr(self, "approach_attitude_limits_enabled", False):
            # Pause metadata is from the same candidate, including neutral hold.
            allowed = allowed and state == WallPerchStatus.STATE_SLOW_APPROACH and not paused
        applied = 0.0
        if allowed:
            previous_at = getattr(self, "preflip_bias_at", None)
            dt = max(0.0, min((now-previous_at).to_sec() if previous_at is not None else .02, .1))
            bias = slew(getattr(self, "preflip_bias_rad", 0.0), math.radians(self.preflip_right_roll_deg),
                        math.radians(self.preflip_roll_slew_deg_s)*dt)
            values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            roll = euler_from_quaternion(values)[0]
            bound = math.radians(min(5.0, getattr(self, "rc_free_max_angle_deg", 5.0)))
            applied = bias if getattr(self, "approach_attitude_limits_enabled", False) else max(-bound, min(bound, roll+bias))-roll
            values = trim_attitude(values, applied, 0.0)
            quaternion = Quaternion(x=values[0], y=values[1], z=values[2], w=values[3])
            self.preflip_bias_rad = bias
        else:
            # Clear immediately on phase loss; never carry this bias into flip or recovery.
            self.preflip_bias_rad = 0.0
        self.preflip_bias_at = now
        publisher = getattr(self, "preflip_bias_publisher", None)
        if publisher is not None:
            publisher.publish(Float32(data=math.degrees(applied)))
        return quaternion

    def _level_attitude(self):
        orientation = self.odom.pose.pose.orientation
        yaw = euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )[2]
        return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def _handover_source_ready(self, now):
        if not getattr(self, "match_fcu_thrust_on_handover", False):
            return True
        target = getattr(self, "fcu_attitude_target", None)
        received_at = getattr(self, "fcu_attitude_target_at", None)
        if target is None or not is_fresh(
            received_at, getattr(self, "fcu_target_timeout_s", 0.20), now
        ):
            return False
        if target.type_mask & AttitudeTarget.IGNORE_THRUST:
            return False
        thrust = float(target.thrust)
        # The takeover sample is allowed across MAVROS' complete normalized
        # multicopter thrust range. Behavior output returns to max_thrust after
        # the bumpless ramp completes.
        return bool(math.isfinite(thrust) and 0.0 <= thrust <= 1.0)

    def _on_manager_phase(self, message):
        if (getattr(self, "ceiling_motion_enabled", False) and message.data == 'ENTERING'
                and getattr(self, 'manager_phase', '') != 'ENTERING'
                and self._operator_mode(rospy.Time.now()) == OperatorCommand.MODE_CEILING):
            now = rospy.Time.now()
            if self._handover_source_ready(now):
                self.ceiling_entry_source = (self.fcu_attitude_target.orientation, float(self.fcu_attitude_target.thrust), now)
        if message.data in ('IDLE', 'LOCKED'):
            self.ceiling_entry_source = None
        self.manager_phase = message.data
        self.manager_phase_at = rospy.Time.now()

    def _reset_cancel_vertical(self):
        self.cancel_vertical_latched = False
        self.cancel_vertical_reference = None
        self.cancel_vertical_telemetry = None
        self.cancel_vertical_previous = self.cancel_vertical_previous_at = None

    def _cancel_vertical_active(self, now):
        flight = getattr(self, "flight_state", None)
        if (getattr(self, "owner", self.OWNER_NONE) != self.OWNER_NONE or flight is None
                or not flight.armed or flight.mode.upper() != "OFFBOARD"):
            self._reset_cancel_vertical()
            return False
        if (getattr(self, "manager_phase", "") == "CANCELING_ENTRY"
                and is_fresh(getattr(self, "manager_phase_at", None), 1.5, now)):
            self.cancel_vertical_latched = True
        # Once canceled, stale phase feedback must never restore fixed thrust.
        return getattr(self, "cancel_vertical_latched", False)

    def _cancel_vertical_target(self, now):
        if not self._odom_fresh(now):
            return None
        odom = self.odom
        z = odom.pose.pose.position.z
        vz = vertical_speed_enu_mps(odom, self.odom_twist_in_child_frame)
        start = getattr(self, "handover_start_thrust", None)
        if start is None or not math.isfinite(start) or not 0.0 <= start <= 1.0:
            return None
        if not (math.isfinite(z) and math.isfinite(vz)):
            return None
        if getattr(self, "cancel_vertical_reference", None) is None:
            measured = odom.pose.pose.orientation
            q = tuple(getattr(measured, k) for k in "xyzw")
            axis, _ = axis_yaw(q)
            # Capture a vertical feedforward once; never learn our OFFBOARD echo.
            self.cancel_vertical_reference = (z, start * max(0.0, axis[2]), self._level_attitude())
            rospy.logwarn("Canceled OFFBOARD entry: vertical feedback active at z=%.3f, base=%.3f", z, start * max(0.0, axis[2]))
        reference, base, heading = self.cancel_vertical_reference
        vz_target = max(-.15, min(.10, .50 * (reference-z)))
        # This caps ALL positive feedback, including braking a descent.
        correction = min(.08, .30 * (vz_target-vz))
        orientation = self._assisted_attitude(now, heading, AttachmentControlCandidate.SOURCE_NONE)
        target = self._attitude_target(now, orientation, base+correction, apply_handover=False)
        if target is None:
            return None
        previous = getattr(self, "cancel_vertical_previous", None)
        previous_at = getattr(self, "cancel_vertical_previous_at", None)
        if previous is None or not is_fresh(previous_at, self.odom_timeout_s, now):
            measured = odom.pose.pose.orientation
            previous = tuple(getattr(measured, k) for k in "xyzw")
        dt = max(0., min((now-previous_at).to_sec(), .1)) if previous_at is not None else .02
        q, _ = limit_recovery_attitude(tuple(getattr(target.orientation, k) for k in "xyzw"),
            previous, dt, math.radians(5.), math.radians(10.), math.radians(20.))
        target.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        axis, _ = axis_yaw(q)
        target.thrust = min(.60, self.max_thrust, max(self.min_thrust,
                            max(0., base+correction) / max(math.cos(math.radians(5.)), axis[2])))
        self.cancel_vertical_previous, self.cancel_vertical_previous_at = q, now
        self.cancel_vertical_telemetry = {"active": True, "height_reference_m": reference,
            "height_m": z, "vertical_speed_mps": vz, "target_vertical_speed_mps": vz_target,
            "base_vertical_thrust": base, "correction": correction, "final_thrust": target.thrust}
        return target

    def _prestream_attitude_target(self, now):
        if self._cancel_vertical_active(now):
            return self._cancel_vertical_target(now)
        if not getattr(self, "match_fcu_thrust_on_handover", False):
            return self._vertical_attitude_target(now)

        entry = getattr(self, "ceiling_entry_source", None)
        if (getattr(self, "ceiling_motion_enabled", False) and entry is not None
                and self._operator_mode(now) == OperatorCommand.MODE_CEILING
                and is_fresh(entry[2], self.ceiling_motion_settings.motion_capture_timeout_s, now)):
            self.ceiling_prestream_attitude = entry[0]
            return self._attitude_target(now, entry[0], entry[1], apply_handover=False)
        flight_mode = (
            self.flight_state.mode.upper()
            if getattr(self, "flight_state", None) is not None
            else ""
        )
        if flight_mode == "OFFBOARD":
            # FCU target feedback is deliberately ignored in OFFBOARD to avoid
            # reading our own command back as the ALTCTL takeover source. Keep
            # publishing the value captured on the ALTCTL -> OFFBOARD edge
            # until an attachment candidate takes ownership and starts the
            # configured bumpless ramp.
            thrust = getattr(self, "handover_start_thrust", None)
            if thrust is None:
                rospy.logerr_throttle(
                    1.0,
                    "OFFBOARD prestream inhibited: captured ALTCTL thrust unavailable",
                )
                return None
        else:
            if not self._handover_source_ready(now):
                rospy.logwarn_throttle(
                    1.0,
                    "OFFBOARD prestream inhibited: FCU thrust target missing, stale or invalid",
                )
                return None
            thrust = float(self.fcu_attitude_target.thrust)

        orientation=self._level_attitude()
        if (getattr(self, "ceiling_motion_enabled", False)
                and self._operator_mode(now) == OperatorCommand.MODE_CEILING):
            if flight_mode == "OFFBOARD":
                orientation = self.ceiling_prestream_attitude
                if orientation is None:
                    return None
            else:
                orientation = self.fcu_attitude_target.orientation
                self.ceiling_prestream_attitude = orientation
        wall=getattr(self,"wall_status",None)
        if (wall is not None and getattr(wall,"horizontal_equilibrium_enabled",False)
                and (self._operator_mode(now)==OperatorCommand.MODE_WALL or (
                    flight_mode=="OFFBOARD" and wall.horizontal_equilibrium_frozen and wall.state==WallPerchStatus.STATE_IDLE))):
            if not is_fresh(self.wall_status_at,self.status_timeout_s,now):
                return None
            if wall.horizontal_prestream_valid:
                orientation=wall.horizontal_prestream_attitude
            elif wall.horizontal_equilibrium_valid:
                orientation=wall.horizontal_equilibrium_attitude
            elif flight_mode != "OFFBOARD":
                # Waiting in POSCTL: follow its fresh attitude, never replace
                # the pilot controller's horizontal equilibrium with Euler zero.
                orientation=self.fcu_attitude_target.orientation
            else:
                rospy.logerr_throttle(1.,"OFFBOARD prestream inhibited: wall equilibrium unavailable")
                return None
        orientation = self._assisted_attitude(now, orientation, AttachmentControlCandidate.SOURCE_NONE)
        return self._attitude_target(
            now,
            orientation,
            thrust,
            apply_handover=False,
        )

    def _apply_thrust_handover(self, now, desired_thrust):
        if not getattr(self, "match_fcu_thrust_on_handover", False):
            return desired_thrust
        start = getattr(self, "handover_start_thrust", None)
        if start is None:
            return desired_thrust
        if getattr(self, "handover_started_at", None) is None:
            self.handover_started_at = now
        elapsed = max(0.0, (now - self.handover_started_at).to_sec())
        duration = getattr(self, "thrust_handover_ramp_s", 1.0)
        fraction = min(1.0, elapsed / duration)
        if fraction >= 1.0:
            self.handover_start_thrust = None
            self.handover_started_at = None
            return desired_thrust
        # Cubic smoothstep has zero slope at both ends.
        blend = fraction * fraction * (3.0 - 2.0 * fraction)
        return start + blend * (desired_thrust - start)

    def _vertical_attitude_target(self, now, velocity_z=0.0, source=0, candidate=None):
        if not self._odom_fresh(now) or not math.isfinite(velocity_z):
            return None
        measured = vertical_speed_enu_mps(
            self.odom, getattr(self, "odom_twist_in_child_frame", True)
        )
        if not math.isfinite(measured):
            return None
        thrust = self.hover_thrust + self.vertical_speed_kp * (velocity_z - measured)
        orientation = self._assisted_attitude(now, self._level_attitude(), source, candidate)
        return self._attitude_target(now, orientation, thrust)

    def _attitude_target(self, now, quaternion, thrust, apply_handover=True):
        full_range_handover = bool(
            getattr(self, "match_fcu_thrust_on_handover", False)
            and not apply_handover
        )
        if apply_handover:
            thrust = self._apply_thrust_handover(now, thrust)
            full_range_handover = getattr(
                self, "handover_start_thrust", None
            ) is not None
        target = self._get_setpoint_builder().attitude_thrust(
            now, quaternion, thrust
        )
        if target is not None and full_range_handover:
            # MavrosSetpointBuilder enforces the normal behavior cap. Only the
            # captured handover sample and its ramp may temporarily bypass that
            # cap so the first OFFBOARD command exactly matches ALTCTL.
            target.thrust = min(1.0, max(0.0, float(thrust)))
        return target

    def _limit_final_approach_target(self, now, target, source, candidate=None):
        status = getattr(self, 'wall_status', None)
        if ((candidate is not None and getattr(candidate, 'incline_active', False))
                or (source == AttachmentControlCandidate.SOURCE_NONE and status is not None
                    and getattr(status, 'incline_active', False))):
            return self._limit_incline_approach_target(now, target, source, candidate)
        if not getattr(self, "approach_attitude_limits_enabled", False):
            return target
        status = getattr(self, "wall_status", None)
        flight = getattr(self, "flight_state", None)
        prestream = bool(source == AttachmentControlCandidate.SOURCE_NONE
            and status is not None and getattr(status,"horizontal_equilibrium_enabled",False)
            and status.state in (WallPerchStatus.STATE_IDLE,WallPerchStatus.STATE_FRONT_WALL_DETECT,
                                 WallPerchStatus.STATE_STABILIZE_HOVER,WallPerchStatus.STATE_SLOW_APPROACH)
            and flight is not None and flight.connected and flight.armed
            and flight.mode.upper() in ("POSCTL","OFFBOARD")
            and (self._operator_mode(now)==OperatorCommand.MODE_WALL
                 or (flight.mode.upper()=="OFFBOARD" and status.horizontal_equilibrium_frozen)))
        control = self._wall_control(now, candidate) if source == AttachmentControlCandidate.SOURCE_WALL else None
        active = bool(flight is not None and is_fresh(self.flight_state_at, self.state_timeout_s, now)
            and ((prestream and is_fresh(self.wall_status_at, self.status_timeout_s, now))
                 or (control is not None and control.control_phase == WallPerchStatus.STATE_SLOW_APPROACH
                     and flight.mode.upper() == "OFFBOARD")))
        q = tuple(getattr(target.orientation, k) for k in "xyzw")
        previous = getattr(self, "approach_previous_target", None)
        previous_at = getattr(self, "approach_previous_at", None)
        if active:
            dt = max(0.0, min((now-previous_at).to_sec(), .1)) if previous_at is not None else .02
            if previous is None or not is_fresh(previous_at, self.status_timeout_s, now):
                measured = self.odom.pose.pose.orientation
                previous = (measured.x, measured.y, measured.z, measured.w)
                dt = .02
            q, projected = limit_recovery_attitude(q, previous, dt,
                math.radians(self.approach_max_tilt_deg), math.radians(self.approach_tilt_slew_deg_s),
                math.radians(self.approach_yaw_slew_deg_s))
            if projected:
                rospy.logwarn_throttle(1.0, "Approach entry target outside cone: immediate projection before slew limiting")
            target.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
            roll,pitch,_ = euler_from_quaternion(q)
            axis,_ = axis_yaw(q)
            telemetry = Vector3Stamped();telemetry.header.stamp = now
            telemetry.vector.x = math.degrees(roll);telemetry.vector.y = math.degrees(pitch)
            telemetry.vector.z = math.degrees(math.acos(max(-1.,min(1.,axis[2]))))
            self.approach_target_publisher.publish(telemetry)
        return target

    def _recovery_output_active(self, control):
        return bool(getattr(self, "recovery_attitude_limits_enabled", False)
                    and control is not None
                    and (control.control_phase in (WallPerchStatus.STATE_RECOVER,
                                                   WallPerchStatus.STATE_EXIT,
                                                   WallPerchStatus.STATE_ABORT)
                         or (control.control_phase == WallPerchStatus.STATE_DETACH_ROTATE
                             and control.horizontal_velocity_enabled
                             and control.horizontal_velocity_active)))

    def _recovery_tilt_limit_deg(self, control):
        from px4_wall_ceiling_control.domain.simple_recovery import MAX_TILT_DEG
        return MAX_TILT_DEG if getattr(control, 'recover_attitude_locked', False) else self.recovery_max_tilt_deg

    def _limit_final_recovery_target(self, now, target, source, candidate=None):
        if candidate is not None and getattr(candidate, 'incline_active', False):
            return target  # Incline has its own continuous publication envelope.
        # Project only the desired axis here. Continuity is applied later from
        # the last actual publication, never from an unsent projected target.
        control = self._wall_control(now, candidate) if source == AttachmentControlCandidate.SOURCE_WALL else None
        flight = getattr(self, "flight_state", None)
        if (not self._recovery_output_active(control) or flight is None
                or flight.mode.upper() != "OFFBOARD"
                or not is_fresh(self.flight_state_at, self.state_timeout_s, now)):
            return target
        q = tuple(getattr(target.orientation, k) for k in "xyzw")
        q, _ = limit_recovery_attitude(q, None, 0.,
            math.radians(self._recovery_tilt_limit_deg(control)), 0., 0.)
        target.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        cap = self.recovery_final_thrust_cap
        if getattr(control, "recovery_support_valid", False):
            # This per-maneuver allowance follows verified vertical support;
            # it does not change hover thrust or the global hard maximum.
            cap = min(self.max_thrust, float(control.recovery_thrust_limit))
        target.thrust = min(target.thrust, cap, self.max_thrust)
        return target

    def _publish_recovery_preview(self, now, target, candidate):
        if not self._recovery_output_active(candidate):
            return
        q = tuple(getattr(target.orientation, key) for key in "xyzw")
        axis, yaw = axis_yaw(q)
        preview = Vector3Stamped(); preview.header.stamp = now
        preview.vector.x = math.degrees(math.acos(max(-1., min(1., axis[2]))))
        preview.vector.y = math.degrees(yaw); preview.vector.z = target.thrust
        self.recovery_target_publisher.publish(preview)
        self.recovery_horizontal_publisher.publish(Float32(data=target.thrust*math.hypot(axis[0], axis[1])))

    def _remember_publication(self, target, source, published_at):
        orientation = getattr(target, "orientation", None)
        if orientation is None:
            return
        q = tuple(getattr(orientation, key) for key in "xyzw")
        thrust = float(target.thrust)
        if not (all(math.isfinite(value) for value in q) and sum(value*value for value in q) > 1e-9
                and math.isfinite(thrust) and 0 <= thrust <= 1):
            return
        q = quaternion_normalize(q)
        self.last_published_control = (q, thrust, published_at, source)
        if getattr(getattr(self, 'operator', None), 'mode', None) == OperatorCommand.MODE_INCLINE:
            self.incline_approach_previous_target, self.incline_approach_previous_at = q, published_at
        else:
            self.approach_previous_target, self.approach_previous_at = q, published_at
        self.recovery_previous_target, self.recovery_previous_at = q, published_at

    def _wall_publication_seed(self, now):
        previous = getattr(self, "last_published_control", None)
        if (previous is not None and previous[3] in (AttachmentControlCandidate.SOURCE_NONE,
                                                    AttachmentControlCandidate.SOURCE_WALL)
                and is_fresh(previous[2], max(.50, self.odom_timeout_s), now)):
            return previous
        # A fresh measured attitude plus a fresh FCU thrust command is the
        # only fallback. Never seed from a nominal hover/pressure guess.
        echo = getattr(self, "wall_continuity_fcu_snapshot", None)
        flight = getattr(self, "flight_state", None)
        if echo is not None and flight is not None and flight.mode.upper() == "OFFBOARD":
            fcu, fcu_received = echo
            timeout = .35  # Match the departure controller's echo continuation window.
        else:
            # At the initial OFFBOARD edge, a still-fresh pilot-mode sample is
            # also valid; it remains the original handover value, not an echo.
            fcu = getattr(self, "fcu_attitude_target", None)
            fcu_received = getattr(self, "fcu_attitude_target_at", None)
            timeout = getattr(self, "fcu_target_timeout_s", .20)
        if (not self._odom_fresh(now)
                or not is_fresh_source(self.odom.header.stamp, self.odom_timeout_s, now)
                or fcu is None or not is_fresh(fcu_received, timeout, now)
                or not is_fresh_source(fcu.header.stamp, timeout, now)
                or fcu.type_mask & AttitudeTarget.IGNORE_THRUST
                or not math.isfinite(fcu.thrust)
                or not self.min_thrust-1e-6 <= fcu.thrust <= self.max_thrust+1e-6):
            return None
        q = self.odom.pose.pose.orientation
        thrust = max(self.min_thrust, min(self.max_thrust, float(fcu.thrust)))
        return (quaternion_normalize((q.x, q.y, q.z, q.w)), thrust, now,
                AttachmentControlCandidate.SOURCE_NONE)

    def _limit_wall_publication(self, now, target, candidate):
        if getattr(candidate, 'incline_active', False):
            return self._limit_incline_publication(now, target, candidate)
        self.wall_publication_slew = None
        phase = candidate.control_phase
        abort = phase == WallPerchStatus.STATE_ABORT
        returning = phase in (WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER,
                              WallPerchStatus.STATE_EXIT, WallPerchStatus.STATE_ABORT)
        if not returning and phase not in (
                WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE,
                WallPerchStatus.STATE_WALL_HOLD, WallPerchStatus.STATE_WALL_PIN):
            return target
        previous = self._wall_publication_seed(now)
        if previous is None:
            if abort:
                # Lack of continuity history must not suppress a validated
                # emergency level command. Hard output bounds still apply.
                target.thrust = max(self.min_thrust, min(self.max_thrust, target.thrust))
                self.wall_publication_slew = {"seed_available": False, "abort_level_immediate": True}
                return target
            self.wall_candidate_rejection = "final_slew_seed_unavailable"
            return None
        q_previous, thrust_previous, previous_at, _source = previous
        if not self.min_thrust <= thrust_previous <= self.max_thrust:
            if not abort:
                self.wall_candidate_rejection = "final_slew_seed_outside_bounds"
                return None
            thrust_previous = max(self.min_thrust, min(self.max_thrust, thrust_previous))
        dt = max(0.0, min((now-previous_at).to_sec(), .05))
        q = tuple(getattr(target.orientation, key) for key in "xyzw")
        desired_q = q
        requested_angle = quaternion_angle(q_previous, q)
        requested_thrust = target.thrust
        abort_level_immediate = False
        if phase in (WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_EXIT):
            prev_axis, _ = axis_yaw(q_previous)
            if math.acos(max(-1., min(1., prev_axis[2]))) > math.radians(self._recovery_tilt_limit_deg(candidate)) + 1e-6:
                # Normal RECOVER is admitted only after both measured and sent
                # axes have settled in the cone. A phase error cannot project
                # an actually published large tilt instantaneously to 5 deg.
                self.wall_candidate_rejection = "recovery_entry_published_axis_outside_cone"
                return None
            q, _ = limit_recovery_attitude(q, q_previous, dt,
                math.radians(self._recovery_tilt_limit_deg(candidate)), math.radians(self.recovery_tilt_slew_deg_s),
                math.radians(self.recovery_yaw_slew_deg_s))
        elif abort:
            # ABORT is also a long-lived near-level recovery state. Ordinary
            # velocity feedback must not inherit the emergency leveling bypass.
            max_tilt = math.radians(self.recovery_max_tilt_deg)
            prev_axis, _ = axis_yaw(q_previous)
            measured = self.odom.pose.pose.orientation if self._odom_fresh(now) else None
            measured_q = tuple(getattr(measured, k) for k in 'xyzw') if measured is not None else ()
            measured_valid = (len(measured_q) == 4 and all(math.isfinite(v) for v in measured_q)
                              and abs(sum(v*v for v in measured_q)-1.) <= .01)
            near_level = bool(measured_valid and prev_axis[2] >= math.cos(max_tilt)-1.e-9
                              and axis_yaw(measured_q)[0][2] >= math.cos(max_tilt)-1.e-9)
            bounded_q, _ = limit_recovery_attitude(q, q_previous, dt, max_tilt,
                math.radians(self.recovery_tilt_slew_deg_s), math.radians(self.recovery_yaw_slew_deg_s))
            if near_level:
                q = bounded_q
            else:
                # Large/unknown measured attitude or a large actual output axis:
                # preserve immediate safe-axis recovery; yaw is never leveling.
                q = attitude_from_axis_yaw(axis_yaw(q)[0], axis_yaw(bounded_q)[1])
                abort_level_immediate = True
        elif not abort:
            rate = (getattr(self, "wall_detach_rate_rad_s", math.radians(56.25)) if returning
                    else getattr(self, "wall_maneuver_rate_rad_s", math.radians(84.375)))
            angle_step = rate * dt
            if requested_angle > angle_step and requested_angle > 1e-9:
                q = quaternion_slerp(q_previous, q, angle_step/requested_angle)
        # Emergency ABORT keeps immediate safe leveling. Its thrust still
        # follows actual support; loss of a soft allowance cannot cause a step.
        slew_rate = (getattr(self, "wall_return_thrust_slew_per_s", .50) if returning
                     else getattr(self, "wall_maneuver_thrust_slew_per_s", .50))
        thrust_step = slew_rate * dt
        target.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        target.thrust = max(self.min_thrust, min(self.max_thrust,
            max(thrust_previous-thrust_step, min(thrust_previous+thrust_step, requested_thrust))))
        self.wall_publication_slew = {
            "seed_available": True, "dt_s": dt,
            "attitude_step_deg": math.degrees(quaternion_angle(q_previous, q)),
            "thrust_step": target.thrust-thrust_previous,
            "abort_level_immediate": abort_level_immediate,
            "abort_near_level_slew": abort and not abort_level_immediate,
            "support_allowance_valid": bool(getattr(candidate, "recovery_support_valid", False)),
            "requested_thrust": requested_thrust,
            "limited": quaternion_angle(q, desired_q) > 1e-9
                       or abs(target.thrust-requested_thrust) > 1e-9,
        }
        return target

    def _incline_output_settings(self):
        settings = getattr(self, 'incline_output_settings', None)
        if settings is None:
            settings = InclineSettings.read(lambda key, default: rospy.get_param('/incline_attachment/' + key, default))
            self.incline_output_settings = settings
        return settings

    def _limit_incline_publication(self, now, target, candidate):
        self.wall_publication_slew = None
        settings = self._incline_output_settings()
        if settings.error(self.max_thrust):
            return None
        if candidate.control_phase == WallPerchStatus.STATE_SLOW_APPROACH:
            # Wall approach already has the shared final attitude envelope.
            # Flip search slew must never throttle its height-control output.
            target.thrust = max(self.min_thrust, min(self.max_thrust, settings.flip_thrust_max, target.thrust))
            return target
        previous = self._wall_publication_seed(now)
        if previous is None:
            self.wall_candidate_rejection = 'incline_continuity_seed_unavailable'
            return None  # No emergency instantaneous leveling bypass.
        seed, thrust, stamp, _source = previous
        dt = min(.05, max(0., (now-stamp).to_sec()))
        q = tuple(getattr(target.orientation, key) for key in 'xyzw')
        if InclineAttachment.tilt(q) > math.radians(settings.flip_max_tilt_deg) + 1e-6:
            self.wall_candidate_rejection = 'incline_target_tilt_exceeds_cap'
            return None
        returning = candidate.control_phase in (WallPerchStatus.STATE_DETACH_ROTATE,
            WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_EXIT, WallPerchStatus.STATE_ABORT)
        rate = settings.detach_max_rate_deg_s if returning else settings.flip_pitch_rate_deg_s
        distance = quaternion_angle(seed, q)
        if distance > 1e-9:
            q = quaternion_slerp(seed, q, min(1., math.radians(rate)*dt/distance))
        # Search pressure and anti-descent support have separate envelopes.
        # Only the incline flip phase receives the vertical-support slew.
        slew = (settings.return_thrust_slew_per_s if returning else
                settings.flip_support_slew_per_s
                if candidate.control_phase == WallPerchStatus.STATE_FLIP_TO_WALL
                else settings.flip_thrust_ramp_per_s)
        value = max(thrust-slew*dt, min(thrust+slew*dt, target.thrust))
        # A newly configured cap cannot silently step down an old live output.
        if thrust > settings.flip_thrust_max + 1e-6:
            self.wall_candidate_rejection = 'incline_seed_thrust_outside_cap'
            return None
        target.orientation = Quaternion(*q)
        target.thrust = max(self.min_thrust, min(self.max_thrust, settings.flip_thrust_max, value))
        return target

    def _publish_candidate(self, now, candidate):
        wall = candidate.source == AttachmentControlCandidate.SOURCE_WALL
        if wall:
            snapshot = getattr(self, "wall_candidate_snapshot", None)
            # Candidate admission and use are both checked; a fresh receipt can
            # never make an old source command fresh again.
            if (snapshot is None or candidate is not snapshot[0]
                    or not self._wall_fresh(now) or not candidate.valid
                    or candidate.maneuver_id != getattr(self, "wall_maneuver_id", None)):
                return self.OUTPUT_NONE
        if candidate.kind == AttachmentControlCandidate.KIND_VERTICAL_VELOCITY:
            target = self._vertical_attitude_target(
                now, candidate.velocity_z_enu_mps, candidate.source, candidate if wall else None)
        elif candidate.kind == AttachmentControlCandidate.KIND_ATTITUDE_THRUST:
            if getattr(candidate, 'incline_active', False):
                if candidate.control_phase == WallPerchStatus.STATE_SLOW_APPROACH:
                    orientation = self._incline_approach_assisted_attitude(now, candidate.attitude_setpoint)
                else:
                    self.incline_rc_roll_rad = self.incline_rc_yaw_rad = 0.
                    self.incline_rc_at = now
                    orientation = candidate.attitude_setpoint
            else:
                orientation = self._assisted_attitude(now, candidate.attitude_setpoint, candidate.source, candidate)
            target = self._attitude_target(now, orientation, candidate.thrust_normalized)
        else:
            return self.OUTPUT_NONE
        if target is None:
            return self.OUTPUT_NONE
        target = self._limit_final_recovery_target(now, target, candidate.source, candidate if wall else None)
        target = self._limit_final_approach_target(now, target, candidate.source, candidate if wall else None)
        if not wall and getattr(self, "ceiling_motion_enabled", False):
            if candidate.kind != AttachmentControlCandidate.KIND_ATTITUDE_THRUST:
                return self.OUTPUT_NONE
            seed = getattr(self, "last_published_control", None)
            previous = seed[0] if seed is not None and is_fresh(seed[2], .30, now) else None
            dt = min(.05, max(0., (now-seed[2]).to_sec())) if previous is not None else 0.
            contact = (candidate.control_phase in (CeilingAttachmentStatus.STATE_ATTACH_CONTROL, CeilingAttachmentStatus.STATE_SURFACE_HOLD)
                       or (candidate.control_phase == CeilingAttachmentStatus.STATE_ABORT and candidate.ceiling_abort_land_requested))
            q = limit_ceiling_attitude(tuple(getattr(target.orientation, k) for k in "xyzw"), previous,
                dt, candidate.ceiling_yaw_reference_rad, self.ceiling_motion_settings, contact,
                follow_hold_yaw=candidate.control_phase == CeilingAttachmentStatus.STATE_SURFACE_HOLD)
            target.orientation = Quaternion(*q)
            snapshot = self.ceiling_candidate_snapshot
            published = rospy.Time.now()
            if (snapshot is None or snapshot[0] is not candidate or not candidate.phase_valid
                    or not is_fresh(snapshot[1], self.ceiling_candidate_timeout_s, published)
                    or not is_fresh_source(candidate.header.stamp, self.ceiling_candidate_timeout_s, published, future_tolerance_s=0.)):
                return self.OUTPUT_NONE
            target.header.stamp = published
        if wall:
            # The previous seed is stamped at actual publication, not at the
            # start of its timer. Use the current time at the matching boundary:
            # otherwise processing time is subtracted from every slew interval,
            # making configured thrust/rate limits slower as CPU work increases.
            # The limiter retains its 50 ms cap; final freshness is checked below.
            target = self._limit_wall_publication(rospy.Time.now(), target, candidate)
            if target is None:
                return self.OUTPUT_NONE
        if wall:
            # Recheck immediately before publication: expensive callbacks must
            # not turn an initially fresh command into a stale outgoing command.
            published_at = rospy.Time.now()
            if (self._wall_candidate_problem(candidate, published_at)
                    or not is_fresh(snapshot[1], self.wall_candidate_timeout_s, published_at)
                    or candidate.maneuver_id != getattr(self, "wall_maneuver_id", None)):
                self.wall_candidate_rejection = "candidate_expired_before_publication"
                return self.OUTPUT_NONE
            # MAVROS header remains the transport publication time. The original
            # generation time is retained separately with the same sequence.
            target.header.stamp = published_at
            command_diagnostic = {
                "maneuver_id": candidate.maneuver_id,
                "control_sequence": candidate.control_sequence,
                "control_phase": candidate.control_phase,
                "detach_task_phase": candidate.detach_task_phase,
                "detach_range_episode": candidate.detach_range_episode,
                "range_height_control": candidate.range_height_control,
                "departure_support_only": getattr(candidate, 'departure_support_only', False),
                "range_feedback_valid": candidate.range_feedback_valid,
                "generated_at_s": candidate.header.stamp.to_sec(),
                "received_at_s": snapshot[1].to_sec(),
                "published_at_s": published_at.to_sec(),
                "source_age_s": (published_at-candidate.header.stamp).to_sec(),
                "receipt_age_s": (published_at-snapshot[1]).to_sec(),
                "publication_slew": getattr(self, "wall_publication_slew", None),
                "attitude_xyzw": [getattr(target.orientation, key) for key in "xyzw"],
                "thrust": target.thrust,
            }
        self.attitude_publisher.publish(target)
        self._remember_publication(target, candidate.source, published_at if wall else now)
        if not wall and getattr(self, 'ceiling_motion_enabled', False):
            self._publish_ceiling_rc_feedback(candidate, target)
        if wall:
            self.last_wall_command = command_diagnostic
            self._publish_recovery_preview(published_at, target, candidate)
        return self.OUTPUT_ATTITUDE_THRUST

    def _publish_ceiling_rc_feedback(self, candidate, target):
        publisher = getattr(self, 'ceiling_rc_feedback_publisher', None)
        if publisher is None or not candidate.phase_valid:
            return
        feedback = CeilingRcAssist()
        feedback.header.stamp = target.header.stamp
        feedback.candidate_stamp = candidate.header.stamp
        feedback.maneuver_id = candidate.maneuver_id
        feedback.control_sequence = candidate.control_sequence
        feedback.control_phase = candidate.control_phase
        calibrated = self._calibrated_rc_axes(target.header.stamp, candidate.source) == ('roll', 'pitch')
        enabled = getattr(self, 'rc_assist_enabled', False)
        ready = not enabled or self._rc_ready(target.header.stamp, candidate.source)
        feedback.input_valid = ready
        if enabled and ready and not candidate.ceiling_rc_assist_blocked:
            fractions = ([self._calibrated_rc_fraction(axis) for axis in ('roll', 'pitch')] if calibrated
                else [stick_fraction(self.rc_channels, index, self.rc_pwm_low, self.rc_pwm_center,
                    self.rc_pwm_high, self.rc_deadzone, reverse) for index, reverse in (
                        (self.rc_roll_channel_index, self.rc_roll_reverse), (self.rc_pitch_channel_index, self.rc_pitch_reverse))])
            # An RC callback may replace the channel frame after _rc_ready.
            feedback.input_valid = all(value is not None for value in fractions)
            feedback.stick_active = feedback.input_valid and any(abs(value) > 0. for value in fractions)
        feedback.roll_trim_rad = self.rc_roll_rad if enabled and not candidate.ceiling_rc_assist_blocked else 0.
        feedback.pitch_trim_rad = self.rc_pitch_rad if enabled and not candidate.ceiling_rc_assist_blocked else 0.
        publisher.publish(feedback)

    def _publish_output_health(self, now, output_mode, reason, checks):
        publisher = getattr(self, "output_health_publisher", None)
        if publisher is None:
            return
        key = (self.owner, output_mode, reason)
        previous = getattr(self, "output_health_last_at", None)
        if key == getattr(self, "output_health_last_key", None) and previous is not None and (now-previous).to_sec() < .1:
            return
        # Report the exact evaluated gates plus a separate post-decision snapshot.
        sampled_at = rospy.Time.now()
        ages = {}
        for name in ("prestream_at", "odom_at", "flight_state_at", "rc_at", "fcu_attitude_target_at", "kill_status_at"):
            stamp = getattr(self, name, None)
            ages[name] = (sampled_at-stamp).to_sec() if stamp is not None else None
        data = {"decision_at_s": now.to_sec(), "sampled_at_s": sampled_at.to_sec(),
                "owner": self.owner, "output_mode": output_mode, "pause_reason": reason,
                "gates": checks, "receipt_ages_s": ages,
                "cancel_vertical_control": getattr(self, "cancel_vertical_telemetry", None),
                "last_wall_command": getattr(self, "last_wall_command", None),
                "wall_candidate_rejection": getattr(self, "wall_candidate_rejection", "")}
        publisher.publish(String(data=json.dumps(data, ensure_ascii=False, allow_nan=False)))
        self.output_health_last_key, self.output_health_last_at = key, now

    def _on_timer(self, _event):
        now = rospy.Time.now()
        if self._tick_rc_takeover(now):
            return
        previous_owner = self.owner
        self._update_owner(now)
        if self.owner != self.OWNER_NONE:
            self._reset_cancel_vertical()
        self.owner_publisher.publish(UInt8(data=self.owner))
        if self.owner != previous_owner:
            rospy.loginfo("attachment output owner=%d", self.owner)

        output_mode = self.OUTPUT_NONE
        clear = kill_clear(self, now)
        checks = {"enabled": self.output_enabled, "kill_clear": clear}
        reason = "output_disabled" if not self.output_enabled else "kill_active_or_stale" if not clear else "idle"
        if self.output_enabled and clear and self.owner == self.OWNER_NONE:
            checks.update({
                "prestream_requested": self.prestream_requested,
                "prestream_fresh": is_fresh(self.prestream_at, self.prestream_timeout_s, now),
                "odom_fresh": self._odom_fresh(now),
                "flight_fresh": is_fresh(self.flight_state_at, self.state_timeout_s, now),
                "flight_connected": self.flight_state is not None and self.flight_state.connected,
                "flight_armed": self.flight_state is not None and self.flight_state.armed,
            })
            reason = next((key for key, value in checks.items() if not value), "")
            if not reason:
                checks["rc_ready"] = self._rc_ready(now)
                if not checks["rc_ready"]:
                    reason = "rc_not_ready"
                else:
                    target = self._prestream_attitude_target(now)
                    if target is not None:
                        if not getattr(self, "cancel_vertical_latched", False):
                            target = self._limit_final_approach_target(now, target, AttachmentControlCandidate.SOURCE_NONE)
                        self.attitude_publisher.publish(target)
                        self._remember_publication(target, AttachmentControlCandidate.SOURCE_NONE, rospy.Time.now())
                        output_mode = self.OUTPUT_ATTITUDE_THRUST
                    else:
                        reason = "handover_thrust_or_equilibrium_missing_stale_or_invalid"
        elif self.output_enabled and self.owner != self.OWNER_NONE:
            # _flight_ready includes the independent emergency stop guard.
            checks.update({"flight_ready": self._flight_ready(now), "odom_fresh": self._odom_fresh(now)})
            if not checks["flight_ready"]:
                reason = "owned_flight_not_ready"
                rospy.logwarn_throttle(1.0, "attachment output inhibited: flight not ready")
            elif not checks["odom_fresh"]:
                reason = "owned_odometry_stale"
                rospy.logwarn_throttle(1.0, "attachment output inhibited: odometry stale")
            elif self.owner in (self.OWNER_CEILING, self.OWNER_WALL):
                snapshot = getattr(self, "wall_candidate_snapshot", None)
                candidate = (self.ceiling_candidate if self.owner == self.OWNER_CEILING
                             else snapshot[0] if snapshot is not None else self.wall_candidate)
                if candidate is not None and candidate.valid:
                    output_mode = self._publish_candidate(now, candidate)
                    reason = "" if output_mode else "candidate_kind_or_payload_invalid"
                else:
                    reason = "ceiling_candidate_invalid" if self.owner == self.OWNER_CEILING else "wall_candidate_invalid"
                    rospy.logerr_throttle(1.0, "%s; output stopped and owner retained", reason)
        if output_mode != self.OUTPUT_NONE:
            reason = ""
        self.mode_publisher.publish(UInt8(data=output_mode))
        self._publish_output_health(now, output_mode, reason, checks)


if __name__ == "__main__":
    rospy.init_node("attachment_output_mux_node")
    from px4_wall_ceiling_control.adapters.async_logging import install_ros_async_logging
    install_ros_async_logging(rospy)
    AttachmentOutputMuxNode()
    rospy.spin()
