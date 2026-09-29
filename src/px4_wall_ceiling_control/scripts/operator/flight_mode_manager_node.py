#!/usr/bin/env python3
"""ALTCTL -> OFFBOARD -> completed detach -> ALTCTL handover.

The output mux remains the sole MAVROS setpoint publisher. This node requests
its attitude prestream and uses MAVROS only to change flight modes.
"""

import threading
import time
import math

import rospy
from mavros_msgs.msg import EstimatorStatus, RCIn, State
from mavros_msgs.srv import SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt8

from px4_wall_ceiling_control.adapters.odometry import (
    odometry_is_finite,
    vertical_speed_enu_mps,
)
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion
from px4_wall_ceiling_control.freshness import is_fresh
from px4_wall_ceiling_control.msg import (
    CeilingAttachmentStatus,
    OperatorCommand,
    SensorHealth,
    WallPerchStatus,
)


class FlightModeManagerNode:
    IDLE = "IDLE"
    PRESTREAM = "PRESTREAM"
    ENTERING = "ENTERING"
    ACTIVE = "ACTIVE"
    RETURNING = "RETURNING"
    LOCKED = "LOCKED"

    def __init__(self):
        self.enabled = bool(rospy.get_param("~enabled", False))
        self.prestream_s = max(1.2, float(rospy.get_param("~prestream_s", 1.5)))
        self.retry_s = max(1.0, float(rospy.get_param("~retry_s", 2.0)))
        self.enter_timeout_s = max(2.0, float(rospy.get_param("~enter_timeout_s", 6.0)))
        self.acquire_timeout_s = max(2.0, float(rospy.get_param("~acquire_timeout_s", 6.0)))
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 0.5))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.5))
        self.estimator_timeout_s = float(rospy.get_param("~estimator_timeout_s", 0.5))
        self.operator_timeout_s = float(rospy.get_param("~operator_timeout_s", 0.5))
        self.sensor_timeout_s = float(rospy.get_param("~sensor_timeout_s", 0.5))
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
        self.lock_until_neutral = False
        self.owner_seen = False
        self.detach_seen = False
        self.recovery_seen = False
        self.completed = False
        self.fault_seen = False
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
        rospy.Subscriber("/mavros/state", State, self._on_flight, queue_size=10)
        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self._on_odom, queue_size=10
        )
        if self.require_rc_center:
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
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)
        if not self.enabled:
            rospy.logwarn("automatic flight mode handover is disabled")

    def _on_flight(self, message):
        self.flight, self.flight_at = message, rospy.Time.now()

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

    def _on_owner(self, message):
        self.owner, self.owner_at = int(message.data), rospy.Time.now()

    def _on_ceiling(self, message):
        self.ceiling, self.ceiling_at = message, rospy.Time.now()

    def _on_wall(self, message):
        self.wall, self.wall_at = message, rospy.Time.now()

    def _set_phase(self, phase):
        if self.phase != phase:
            rospy.loginfo("attachment flight phase: %s -> %s", self.phase, phase)
            self.phase = phase
            self.phase_since = time.monotonic()

    def _requested_mode(self, now):
        if not is_fresh(self.operator_at, self.operator_timeout_s, now):
            return None
        if self.operator is None or not self.operator.enable_control:
            return None
        if self.operator.mode in (
            OperatorCommand.MODE_CEILING,
            OperatorCommand.MODE_WALL,
        ):
            return self.operator.mode
        return None

    def _preflight_ready(self, now, selected):
        if not self.enabled or self.flight is None:
            return False
        if not is_fresh(self.flight_at, self.state_timeout_s, now):
            return False
        if not self.flight.connected or not self.flight.armed:
            return False
        if self.odom is None or not is_fresh(
            self.odom_at, self.odom_timeout_s, now
        ) or not odometry_is_finite(self.odom):
            return False
        if self.estimator is None or not is_fresh(
            self.estimator_at, self.estimator_timeout_s, now
        ):
            return False
        if not (
            self.estimator.attitude_status_flag
            and self.estimator.velocity_vert_status_flag
        ):
            return False
        if getattr(self, "require_rc_center", False) and not self._rc_fresh(now):
            return False
        if self.sensor is None or not is_fresh(
            self.sensor_at, self.sensor_timeout_s, now
        ):
            return False
        if selected == OperatorCommand.MODE_CEILING:
            return self.sensor.ceiling_valid
        if selected == OperatorCommand.MODE_WALL:
            return self.sensor.ceiling_valid and self.sensor.front_valid
        return False

    def _stream_ready(self, now):
        return bool(
            self.output_mode == 2
            and is_fresh(self.output_at, self.output_timeout_s, now)
            and self.owner == 0
            and is_fresh(self.owner_at, self.output_timeout_s, now)
        )

    def _rc_fresh(self, now):
        channels = getattr(self, "rc_channels", None)
        indices = (
            self.rc_roll_channel_index,
            self.rc_pitch_channel_index,
            self.rc_throttle_channel_index,
        )
        return bool(
            channels is not None
            and is_fresh(getattr(self, "rc_at", None), self.rc_timeout_s, now)
            and all(0 <= index < len(channels) for index in indices)
            and all(800 <= channels[index] <= 2200 for index in indices)
        )

    def _handover_ready(self, now):
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
        if self.odom is None or not is_fresh(
            self.odom_at, self.odom_timeout_s, now
        ) or not odometry_is_finite(self.odom):
            return False
        orientation = self.odom.pose.pose.orientation
        roll, pitch, _yaw = euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        limit = math.radians(self.handover_max_tilt_deg)
        vertical_speed = vertical_speed_enu_mps(
            self.odom, self.odom_twist_in_child_frame
        )
        if (
            abs(roll) > limit or abs(pitch) > limit
            or not math.isfinite(vertical_speed)
            or abs(vertical_speed) > self.handover_max_vertical_speed_mps
        ):
            return False
        if not getattr(self, "require_rc_center", False):
            return True
        if not self._rc_fresh(now):
            return False
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
            if status.state == CeilingAttachmentStatus.STATE_FAULT or status.detach_failed:
                self.fault_seen = True
            if status.state == CeilingAttachmentStatus.STATE_DETACH:
                self.detach_seen = True
            elif self.detach_seen and status.state == CeilingAttachmentStatus.STATE_RECOVERY_HOVER:
                self.recovery_seen = True
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
            if status is None or not is_fresh(received_at, self.status_timeout_s, now):
                return
            if status.state == WallPerchStatus.STATE_ABORT:
                self.fault_seen = True
            if status.state == WallPerchStatus.STATE_DETACH_ROTATE:
                self.detach_seen = True
            elif self.detach_seen and status.state == WallPerchStatus.STATE_RECOVER:
                self.recovery_seen = True
            elif (
                self.recovery_seen
                and status.state in (WallPerchStatus.STATE_EXIT, WallPerchStatus.STATE_IDLE)
                and not status.fault_detected
                and not self.fault_seen
            ):
                self.completed = True

    def _request_mode(self, mode):
        now = time.monotonic()
        if self.request_in_flight or (
            self.last_request_at is not None
            and now - self.last_request_at < self.retry_s
        ):
            return
        self.last_request_at = now
        self.request_in_flight = True
        threading.Thread(
            target=self._send_mode_request, args=(mode,), daemon=True
        ).start()

    def _send_mode_request(self, mode):
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=0.2)
            if mode == "OFFBOARD":
                if (
                    self.phase != self.ENTERING
                    or self._requested_mode(rospy.Time.now()) != self.selected_mode
                    or not self._preflight_ready(rospy.Time.now(), self.selected_mode)
                    or not self._stream_ready(rospy.Time.now())
                ):
                    return
            elif mode == "ALTCTL":
                if self.phase != self.RETURNING or not self._handover_ready(
                    rospy.Time.now()
                ):
                    return
            request = SetModeRequest()
            request.custom_mode = mode
            response = self.set_mode(request)
            if not response.mode_sent:
                rospy.logwarn("PX4 did not accept %s mode request", mode)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("MAVROS set_mode %s failed: %s", mode, exc)
        finally:
            self.request_in_flight = False

    def _reset(self, lock=False):
        self.selected_mode = None
        self.stream_since = None
        self.owner_seen = False
        self.detach_seen = False
        self.recovery_seen = False
        self.completed = False
        self.fault_seen = False
        self.lock_until_neutral = lock
        self._set_phase(self.LOCKED if lock else self.IDLE)

    def _on_timer(self, _event):
        now = rospy.Time.now()
        monotonic_now = time.monotonic()
        requested = self._requested_mode(now)
        flight_mode = self.flight.mode.upper() if self.flight else ""
        flight_fresh = is_fresh(self.flight_at, self.state_timeout_s, now)

        if self.phase == self.LOCKED and requested is None:
            self._reset(lock=False)
        if self.phase == self.IDLE:
            if (
                requested is not None
                and flight_mode == "ALTCTL"
                and self._preflight_ready(now, requested)
            ):
                self.selected_mode = requested
                self._set_phase(self.PRESTREAM)
        elif self.phase == self.PRESTREAM:
            if requested != self.selected_mode or flight_mode != "ALTCTL" or not self._preflight_ready(now, self.selected_mode):
                self._reset(lock=requested is not None)
            elif self._stream_ready(now):
                if self.stream_since is None:
                    self.stream_since = monotonic_now
                elif monotonic_now - self.stream_since >= self.prestream_s:
                    self._set_phase(self.ENTERING)
                    self._request_mode("OFFBOARD")
            else:
                self.stream_since = None
        elif self.phase == self.ENTERING:
            if flight_fresh and flight_mode == "OFFBOARD":
                self._set_phase(self.ACTIVE)
            elif (
                requested != self.selected_mode
                or not self._preflight_ready(now, self.selected_mode)
                or not self._stream_ready(now)
            ):
                self._reset(lock=requested is not None)
            elif monotonic_now - self.phase_since > self.enter_timeout_s:
                rospy.logerr("OFFBOARD entry timed out; release command before retry")
                self._reset(lock=True)
            else:
                self._request_mode("OFFBOARD")
        elif self.phase == self.ACTIVE:
            if flight_fresh and flight_mode != "OFFBOARD":
                rospy.logwarn("PX4 left OFFBOARD externally; automatic reentry inhibited")
                self._reset(lock=True)
            else:
                self._observe_completion(now)
                if self.completed and self.owner_seen and self.owner == 0 and is_fresh(self.owner_at, self.output_timeout_s, now):
                    self._set_phase(self.RETURNING)
                elif (
                    not self.owner_seen
                    and monotonic_now - self.phase_since > self.acquire_timeout_s
                ):
                    rospy.logerr("attachment owner was not acquired; returning to ALTCTL")
                    self._set_phase(self.RETURNING)
        elif self.phase == self.RETURNING:
            if flight_fresh and flight_mode == "ALTCTL":
                self._reset(lock=True)
            elif flight_fresh and flight_mode == "OFFBOARD" and self._handover_ready(now):
                self._request_mode("ALTCTL")
            elif flight_fresh:
                self._reset(lock=True)

        self.prestream_pub.publish(
            Bool(data=self.phase in (self.PRESTREAM, self.ENTERING, self.ACTIVE, self.RETURNING))
        )
        self.phase_pub.publish(String(data=self.phase))


if __name__ == "__main__":
    rospy.init_node("flight_mode_manager_node")
    FlightModeManagerNode()
    rospy.spin()
