#!/usr/bin/env python3
"""ROS/MAVROS decision port of the reference PX4 wall_perch state machine."""

import math

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from px4_wall_ceiling_control.adapters.odometry import vertical_speed_enu_mps
from px4_wall_ceiling_control.domain.geometry import (
    euler_from_quaternion,
    follow_rotation_about_body_z,
    hover_and_pitched_quaternions,
    quaternion_angle,
    quaternion_multiply,
    quaternion_normalize,
    quaternion_slerp,
    rotate_vector,
    smoothstep5,
)
from px4_wall_ceiling_control.domain.sample_hold import held_samples, record_hold_sample
from px4_wall_ceiling_control.freshness import clamp, is_fresh, is_valid_range
from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    OperatorCommand,
    SensorHealth,
    WallPerchStatus,
)


class WallPerchDecisionNode:
    STATE_NAMES = {
        WallPerchStatus.STATE_IDLE: "IDLE",
        WallPerchStatus.STATE_FRONT_WALL_DETECT: "FRONT_WALL_DETECT",
        WallPerchStatus.STATE_STABILIZE_HOVER: "STABILIZE_HOVER",
        WallPerchStatus.STATE_SLOW_APPROACH: "SLOW_APPROACH",
        WallPerchStatus.STATE_FLIP_TO_WALL: "FLIP_TO_WALL",
        WallPerchStatus.STATE_WALL_CAPTURE: "WALL_CAPTURE",
        WallPerchStatus.STATE_WALL_HOLD: "WALL_HOLD",
        WallPerchStatus.STATE_WALL_PIN: "WALL_PIN",
        WallPerchStatus.STATE_DETACH_ROTATE: "DETACH_ROTATE",
        WallPerchStatus.STATE_RECOVER: "RECOVER",
        WallPerchStatus.STATE_EXIT: "EXIT",
        WallPerchStatus.STATE_ABORT: "ABORT",
    }

    def __init__(self):
        self.decision_enabled = bool(rospy.get_param("~decision_enabled", False))
        self.front_ready_distance_m = float(
            rospy.get_param("~front_ready_distance_m", 0.50)
        )
        self.front_ready_hold_s = float(
            rospy.get_param("~front_ready_hold_s", 0.40)
        )
        self.flip_trigger_distance_m = float(
            rospy.get_param("~flip_trigger_distance_m", 0.20)
        )
        self.flip_trigger_hold_s = float(
            rospy.get_param("~flip_trigger_hold_s", 0.10)
        )
        self.top_contact_distance_m = float(
            rospy.get_param("~top_contact_distance_m", 0.05)
        )
        self.top_contact_hold_s = float(
            rospy.get_param("~top_contact_hold_s", 1.0)
        )
        self.top_clear_distance_m = float(
            rospy.get_param("~top_clear_distance_m", 0.10)
        )
        self.top_clear_hold_s = float(
            rospy.get_param("~top_clear_hold_s", 0.20)
        )
        self.wall_pressure_target_distance_m = float(
            rospy.get_param(
                "~wall_pressure_target_distance_m", self.top_contact_distance_m
            )
        )
        self.wall_pressure_tolerance_m = max(
            0.0, float(rospy.get_param("~wall_pressure_tolerance_m", 0.01))
        )
        # Retained for compatibility with the published state enum and old
        # parameter files.  The normal entry path now skips STABILIZE_HOVER.
        self.stabilize_time_s = float(rospy.get_param("~stabilize_time_s", 0.0))
        self.approach_timeout_s = float(
            rospy.get_param("~approach_timeout_s", 3.0)
        )
        self.flip_time_s = max(0.01, float(rospy.get_param("~flip_time_s", 1.0)))
        self.capture_time_s = max(
            0.01, float(rospy.get_param("~capture_time_s", 0.20))
        )
        self.capture_timeout_s = max(
            0.01,
            float(rospy.get_param("~capture_timeout_s", 5.0)),
        )
        self.hold_time_s = float(rospy.get_param("~hold_time_s", 0.0))
        self.hold_entry_blend_s = max(
            0.01, float(rospy.get_param("~hold_entry_blend_s", 0.50))
        )
        self.detach_reference_angle_rad = math.radians(
            float(rospy.get_param("~detach_reference_angle_deg", 10.0))
        )
        self.detach_axis_error_rad = math.radians(
            float(rospy.get_param("~detach_axis_error_deg", 5.0))
        )
        self.detach_max_rate_rps = float(
            rospy.get_param("~detach_max_rate_rps", 0.15)
        )
        self.detach_max_abs_vertical_speed_mps = float(
            rospy.get_param("~detach_max_abs_vertical_speed_mps", 0.15)
        )
        self.detach_stable_hold_s = float(
            rospy.get_param("~detach_stable_hold_s", 0.50)
        )
        self.detach_time_s = max(
            0.01, float(rospy.get_param("~detach_time_s", 1.0))
        )
        self.recover_time_s = float(rospy.get_param("~recover_time_s", 0.50))
        self.wall_angle_rad = math.radians(
            float(rospy.get_param("~wall_angle_deg", 90.0))
        )
        self.approach_pitch_rad = math.radians(
            float(rospy.get_param("~approach_pitch_deg", 3.0))
        )
        self.hover_thrust = clamp(
            float(rospy.get_param("~hover_thrust", 0.45)), 0.1, 0.9
        )
        self.approach_thrust = float(rospy.get_param("~approach_thrust", 0.47))
        self.flip_thrust_multiplier = float(
            rospy.get_param("~flip_thrust_multiplier", 1.35)
        )
        self.hold_thrust_multiplier = float(
            rospy.get_param("~hold_thrust_multiplier", 1.50)
        )
        self.recover_thrust_multiplier = float(
            rospy.get_param("~recover_thrust_multiplier", 1.10)
        )
        self.max_thrust = clamp(
            float(rospy.get_param("~max_thrust", 0.85)), 0.1, 1.0
        )
        if self.max_thrust <= self.hover_thrust:
            raise ValueError("wall detach max_thrust must exceed hover_thrust")
        pressure_floor = min(
            self.max_thrust,
            max(
                self.hover_thrust * self.flip_thrust_multiplier,
                self.hover_thrust * self.hold_thrust_multiplier,
            ),
        )
        self.wall_pressure_max_thrust = clamp(
            float(rospy.get_param("~wall_pressure_max_thrust", 0.75)),
            pressure_floor,
            self.max_thrust,
        )
        self.wall_pressure_ramp_up_per_s = max(
            0.0, float(rospy.get_param("~wall_pressure_ramp_up_per_s", 0.03))
        )
        self.wall_pressure_ramp_down_per_s = max(
            0.0, float(rospy.get_param("~wall_pressure_ramp_down_per_s", 0.05))
        )
        self.detach_altitude_kp = max(
            0.0, float(rospy.get_param("~detach_altitude_kp", 0.50))
        )
        self.detach_vertical_speed_kd = max(
            0.0, float(rospy.get_param("~detach_vertical_speed_kd", 0.30))
        )
        self.detach_compensation_start_rad = math.radians(clamp(
            float(rospy.get_param("~detach_compensation_start_deg", 75.0)),
            1.0, 89.0,
        ))
        self.detach_altitude_tolerance_m = max(
            0.0, float(rospy.get_param("~detach_altitude_tolerance_m", 0.10))
        )
        self.detach_vertical_speed_tolerance_mps = max(
            0.0, float(rospy.get_param("~detach_vertical_speed_tolerance_mps", 0.20))
        )
        self.max_rate_rps = float(rospy.get_param("~max_rate_rps", 3.0))
        self.flip_max_rate_rps = float(
            rospy.get_param("~flip_max_rate_rps", 6.0)
        )
        self.capture_max_rate_rps = float(
            rospy.get_param("~capture_max_rate_rps", 4.5)
        )
        self.recover_max_rate_rps = float(
            rospy.get_param("~recover_max_rate_rps", 3.0)
        )
        self.max_down_velocity_mps = float(
            rospy.get_param("~max_down_velocity_mps", 0.80)
        )
        self.min_altitude_m = float(rospy.get_param("~min_altitude_m", 0.30))
        self.recover_angle_rad = math.radians(
            float(rospy.get_param("~recover_angle_deg", 15.0))
        )
        self.sensor_timeout_s = float(rospy.get_param("~sensor_timeout_s", 0.60))
        self.sensor_health_timeout_s = float(
            rospy.get_param("~sensor_health_timeout_s", 0.30)
        )
        self.require_top_sensor_on_acquire = bool(
            rospy.get_param("~require_top_sensor_on_acquire", True)
        )
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.odom_twist_in_child_frame = bool(
            rospy.get_param("~odom_twist_in_child_frame", True)
        )
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 0.50))
        self.operator_timeout_s = float(
            rospy.get_param("~operator_timeout_s", 0.50)
        )
        self.filter_alpha = clamp(
            float(rospy.get_param("~range_filter_alpha", 0.30)), 0.0, 1.0
        )
        # PX4's raw actuator/mixer bypass has no equivalent in this ROS path.
        # If enabled, WALL_PIN remains an attitude target with bounded thrust.
        self.pin_enabled = bool(rospy.get_param("~pin_enabled", False))
        self.pin_pitch_rad = math.radians(
            float(rospy.get_param("~pin_pitch_deg", 85.0))
        )
        self.pin_thrust = float(rospy.get_param("~pin_thrust", 0.85))
        self.pin_hold_s = float(rospy.get_param("~pin_hold_s", 0.0))
        self.require_armed = bool(rospy.get_param("~require_armed", True))
        self.require_offboard = bool(rospy.get_param("~require_offboard", True))

        self.state = WallPerchStatus.STATE_IDLE
        self.state_entered_at = rospy.Time.now()
        self.front_range = None
        self.front_received_at = None
        self.front_filtered_m = None
        self.top_range = None
        self.top_received_at = None
        self.top_filtered_m = None
        self.sensor_health = None
        self.sensor_health_received_at = None
        self.odom = None
        self.odom_received_at = None
        self.flight_state = None
        self.flight_state_received_at = None
        self.operator = None
        self.operator_received_at = None
        self.detach_requested = False
        self.cancel_requested = False
        self.rearm_required = False
        self.front_ready_since = None
        self.front_ready_last_sample_at = None
        self.flip_ready_since = None
        self.flip_ready_last_sample_at = None
        self.top_contact_since = None
        self.top_contact_last_sample_at = None
        self.top_clear_since = None
        self.top_clear_last_sample_at = None
        self.q_hover = (0.0, 0.0, 0.0, 1.0)
        self.q_wall = (0.0, 0.0, 0.0, 1.0)
        self.q_contact = (0.0, 0.0, 0.0, 1.0)
        self.q_hold_reference = self.q_contact
        self.q_detach_start = self.q_contact
        self.detach_altitude_m = None
        self.detach_stable_since = None
        self.detach_stable_samples = 0
        self.wall_pressure_thrust_command = self._wall_pressure_start_thrust()
        self.previous_update_at = None
        self.control_dt = 0.02
        self.fault_count = 0
        self.last_reason = "initialized"

        self.status_publisher = rospy.Publisher(
            rospy.get_param("~status_topic", "/wall_perch/status"),
            WallPerchStatus,
            queue_size=10,
        )
        self.control_candidate_publisher = rospy.Publisher(
            rospy.get_param(
                "~control_candidate_topic", "/wall_perch/control_candidate"
            ),
            AttachmentControlCandidate,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~front_range_topic", "/sensor/front/range"),
            Range,
            self._on_front_range,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~top_range_topic", "/sensor/ceiling/range"),
            Range,
            self._on_top_range,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~sensor_health_topic", "/sensor/health"),
            SensorHealth,
            self._on_sensor_health,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/mavros/local_position/odom"),
            Odometry,
            self._on_odom,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~state_topic", "/mavros/state"),
            State,
            self._on_state,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~operator_topic", "/operator/command"),
            OperatorCommand,
            self._on_operator,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~detach_topic", "/wall_perch/detach"),
            Bool,
            self._on_detach,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~cancel_topic", "/wall_perch/cancel"),
            Bool,
            self._on_cancel,
            queue_size=10,
        )
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 2.0)), self._on_timer
        )

        if not self.decision_enabled:
            rospy.logwarn("wall perch decision is disabled")

    def _filter_range(self, message, previous):
        if not is_valid_range(message):
            return previous
        if previous is None:
            return message.range
        return previous + self.filter_alpha * (message.range - previous)

    def _wall_pressure_start_thrust(self):
        return min(
            self.hover_thrust * getattr(self, "flip_thrust_multiplier", 1.05),
            getattr(self, "wall_pressure_max_thrust", self.max_thrust),
            self.max_thrust,
        )

    def _wall_pressure_nominal_thrust(self):
        return min(
            self.hover_thrust * self.hold_thrust_multiplier,
            getattr(self, "wall_pressure_max_thrust", self.max_thrust),
            self.max_thrust,
        )

    @staticmethod
    def _slew_value(value, target, step):
        if value < target:
            return min(target, value + step)
        return max(target, value - step)

    def _wall_pressure_thrust(self):
        """Adjust wall pressure from the top lidar after the vehicle rotates."""
        start = self._wall_pressure_start_thrust()
        nominal = self._wall_pressure_nominal_thrust()
        upper = min(
            getattr(self, "wall_pressure_max_thrust", self.max_thrust),
            self.max_thrust,
        )
        thrust = clamp(
            getattr(self, "wall_pressure_thrust_command", start),
            min(start, nominal),
            upper,
        )
        top_distance = getattr(self, "top_filtered_m", None)
        if top_distance is None:
            return thrust

        error = top_distance - getattr(
            self,
            "wall_pressure_target_distance_m",
            getattr(self, "top_contact_distance_m", 0.05),
        )
        dt = max(0.0, getattr(self, "control_dt", 0.02))
        if error > getattr(self, "wall_pressure_tolerance_m", 0.01):
            # Wall contact/pressure is insufficient: increase only up to the
            # independent wall-pressure cap.
            thrust = min(
                upper,
                thrust + getattr(self, "wall_pressure_ramp_up_per_s", 0.03) * dt,
            )
        else:
            # Contact is in range (or over-compressed). Return gently to the
            # nominal hold pressure instead of remaining at the peak value.
            thrust = self._slew_value(
                thrust,
                nominal,
                getattr(self, "wall_pressure_ramp_down_per_s", 0.05) * dt,
            )

        self.wall_pressure_thrust_command = thrust
        return thrust

    def _on_front_range(self, message):
        if not is_valid_range(message):
            return
        now = rospy.Time.now()
        previous = (
            None
            if not is_fresh(self.front_received_at, self.sensor_timeout_s, now)
            else self.front_filtered_m
        )
        self.front_range = message
        self.front_received_at = now
        self.front_filtered_m = self._filter_range(message, previous)
        (
            self.front_ready_since,
            self.front_ready_last_sample_at,
        ) = self._record_hold_sample(
            self.front_filtered_m < self.front_ready_distance_m,
            now,
            self.front_ready_since,
        )
        (
            self.flip_ready_since,
            self.flip_ready_last_sample_at,
        ) = self._record_hold_sample(
            self.front_filtered_m <= self.flip_trigger_distance_m,
            now,
            self.flip_ready_since,
        )

    def _on_top_range(self, message):
        if not is_valid_range(message):
            return
        now = rospy.Time.now()
        previous = (
            None
            if not is_fresh(self.top_received_at, self.sensor_timeout_s, now)
            else self.top_filtered_m
        )
        self.top_range = message
        self.top_received_at = now
        self.top_filtered_m = self._filter_range(message, previous)
        (
            self.top_contact_since,
            self.top_contact_last_sample_at,
        ) = self._record_hold_sample(
            self.top_filtered_m <= self.top_contact_distance_m,
            now,
            self.top_contact_since,
        )
        (
            self.top_clear_since,
            self.top_clear_last_sample_at,
        ) = self._record_hold_sample(
            self.top_filtered_m >= self.top_clear_distance_m,
            now,
            self.top_clear_since,
        )

    def _on_sensor_health(self, message):
        self.sensor_health = message
        self.sensor_health_received_at = rospy.Time.now()

    def _on_odom(self, message):
        self.odom = message
        now = rospy.Time.now()
        self.odom_received_at = now
        if self.state == WallPerchStatus.STATE_WALL_HOLD:
            if self._detach_pose_valid():
                if self.detach_stable_since is None:
                    self.detach_stable_since = now
                self.detach_stable_samples += 1
            else:
                self.detach_stable_since = None
                self.detach_stable_samples = 0

    def _on_state(self, message):
        self.flight_state = message
        self.flight_state_received_at = rospy.Time.now()

    def _on_operator(self, message):
        self.operator = message
        self.operator_received_at = rospy.Time.now()

    def _on_detach(self, message):
        if message.data and self.state == WallPerchStatus.STATE_WALL_HOLD:
            self.detach_requested = True

    def _on_cancel(self, message):
        if message.data:
            self.cancel_requested = True

    @staticmethod
    def _quat_multiply(a, b):
        return quaternion_multiply(a, b)

    @staticmethod
    def _quat_normalize(q):
        return quaternion_normalize(q)

    @classmethod
    def _quat_slerp(cls, q0, q1, amount):
        return quaternion_slerp(q0, q1, amount)

    @staticmethod
    def _smoothstep5(value):
        return smoothstep5(value)

    @staticmethod
    def _euler_from_quaternion(q):
        return euler_from_quaternion(q)

    @classmethod
    def _hover_and_wall_quaternions(cls, yaw, wall_pitch):
        return hover_and_pitched_quaternions(yaw, wall_pitch)

    def _current_quaternion(self):
        if getattr(self, "odom", None) is None:
            return None
        q = self.odom.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return None
        if math.sqrt(sum(value * value for value in values)) < 1.0e-6:
            return None
        return self._quat_normalize(values)

    def _hold_follow_quaternion(self):
        measured = self._current_quaternion()
        reference = getattr(self, "q_hold_reference", self.q_contact)
        if measured is None:
            return reference
        followed = follow_rotation_about_body_z(reference, measured)
        return followed if followed is not None else reference

    def _detach_pose_valid(self):
        measured = self._current_quaternion()
        if measured is None or self._height_feedback() is None:
            return False
        reference = getattr(self, "q_hold_reference", self.q_contact)
        followed = self._hold_follow_quaternion()
        if quaternion_angle(measured, reference) > self.detach_reference_angle_rad:
            return False
        if quaternion_angle(measured, followed) > self.detach_axis_error_rad:
            return False
        if not self._rates_safe(self.detach_max_rate_rps):
            return False
        velocity_z = vertical_speed_enu_mps(
            self.odom, self.odom_twist_in_child_frame
        )
        return bool(
            math.isfinite(velocity_z)
            and abs(velocity_z) <= self.detach_max_abs_vertical_speed_mps
        )

    def _detach_pose_ready(self, now):
        return bool(
            self._odom_fresh(now)
            and self._state_age(now) >= self.hold_entry_blend_s
            and self.detach_stable_since is not None
            and self.detach_stable_samples >= 2
            and (now - self.detach_stable_since).to_sec() >= self.detach_stable_hold_s
            and self._detach_pose_valid()
        )

    def _state_age(self, now):
        return max(0.0, (now - self.state_entered_at).to_sec())

    def _mode_requested(self, now):
        return bool(
            self.decision_enabled
            and is_fresh(self.operator_received_at, self.operator_timeout_s, now)
            and self.operator is not None
            and self.operator.enable_control
            and self.operator.mode == OperatorCommand.MODE_WALL
        )

    def _front_fresh(self, now):
        return bool(
            is_fresh(self.front_received_at, self.sensor_timeout_s, now)
            and self.front_filtered_m is not None
        )

    def _top_fresh(self, now):
        return bool(
            is_fresh(self.top_received_at, self.sensor_timeout_s, now)
            and self.top_filtered_m is not None
        )

    def _sensor_health_fresh(self, now):
        return bool(
            self.sensor_health is not None
            and is_fresh(
                self.sensor_health_received_at,
                self.sensor_health_timeout_s,
                now,
            )
        )

    def _top_preflight_ready(self, now):
        if not self.require_top_sensor_on_acquire:
            return True
        return bool(
            self._sensor_health_fresh(now)
            and self.sensor_health.ceiling_valid
        )

    def _odom_fresh(self, now):
        return bool(
            is_fresh(self.odom_received_at, self.odom_timeout_s, now)
            and self._current_quaternion() is not None
        )

    def _flight_ready(self, now):
        if not is_fresh(self.flight_state_received_at, self.state_timeout_s, now):
            return False
        if self.flight_state is None or not self.flight_state.connected:
            return False
        if self.require_armed and not self.flight_state.armed:
            return False
        if self.require_offboard and self.flight_state.mode.upper() != "OFFBOARD":
            return False
        return True

    def _top_required(self):
        return self.state in (
            WallPerchStatus.STATE_FLIP_TO_WALL,
            WallPerchStatus.STATE_WALL_CAPTURE,
            WallPerchStatus.STATE_WALL_HOLD,
            WallPerchStatus.STATE_WALL_PIN,
            WallPerchStatus.STATE_DETACH_ROTATE,
            WallPerchStatus.STATE_RECOVER,
        )

    def _front_required(self):
        # The forward-facing lidar is only a decision/safety input until the
        # flip starts.  After rotation it may no longer face the wall.
        return self.state in (
            WallPerchStatus.STATE_IDLE,
            WallPerchStatus.STATE_FRONT_WALL_DETECT,
            WallPerchStatus.STATE_STABILIZE_HOVER,
            WallPerchStatus.STATE_SLOW_APPROACH,
        )

    def _rates_safe(self, limit=None):
        if self.odom is None:
            return False
        limit = self.max_rate_rps if limit is None else limit
        rate = self.odom.twist.twist.angular
        return all(
            math.isfinite(value) and abs(value) <= limit
            for value in (rate.x, rate.y, rate.z)
        )

    def _vertical_speed_safe(self):
        if self.odom is None:
            return False
        velocity_z = vertical_speed_enu_mps(
            self.odom,
            getattr(self, "odom_twist_in_child_frame", True),
        )
        return math.isfinite(velocity_z) and velocity_z >= -self.max_down_velocity_mps

    def _height_feedback(self):
        if getattr(self, "odom", None) is None:
            return None
        altitude = self.odom.pose.pose.position.z
        speed = vertical_speed_enu_mps(
            self.odom, getattr(self, "odom_twist_in_child_frame", True)
        )
        if not math.isfinite(altitude) or not math.isfinite(speed):
            return None
        return altitude, speed

    def _height_hold_thrust(self):
        """Collective needed for the departure altitude at the measured tilt."""
        feedback = self._height_feedback()
        attitude = self._current_quaternion()
        if getattr(self, "detach_altitude_m", None) is None or feedback is None or attitude is None:
            return math.nan
        altitude, speed = feedback
        vertical_fraction = max(0.0, rotate_vector(attitude, (0.0, 0.0, 1.0))[2])
        vertical_demand = (
            self.hover_thrust
            + self.detach_altitude_kp * (self.detach_altitude_m - altitude)
            - self.detach_vertical_speed_kd * speed
        )
        if vertical_demand <= 0.0:
            return 0.1
        return clamp(vertical_demand / max(vertical_fraction, 1.0e-3), 0.1, self.max_thrust)

    def _detach_height_settled(self):
        feedback = self._height_feedback()
        return bool(
            self.detach_altitude_m is not None
            and feedback is not None
            and abs(self.detach_altitude_m - feedback[0]) <= self.detach_altitude_tolerance_m
            and abs(feedback[1]) <= self.detach_vertical_speed_tolerance_mps
        )

    def _attitude_recovered(self):
        quaternion = self._current_quaternion()
        if quaternion is None:
            return False
        roll, pitch, _yaw = self._euler_from_quaternion(quaternion)
        return abs(roll) < self.recover_angle_rad and abs(pitch) < self.recover_angle_rad

    def _safety_ok(self, now):
        if not self._flight_ready(now) or not self._odom_fresh(now):
            return False
        if not self._top_preflight_ready(now):
            return False
        if self._front_required() and not self._front_fresh(now):
            return False
        if self._top_required() and not self._top_fresh(now):
            return False
        if self.state == WallPerchStatus.STATE_FLIP_TO_WALL:
            rate_limit = self.flip_max_rate_rps
        elif self.state in (
            WallPerchStatus.STATE_WALL_CAPTURE,
            WallPerchStatus.STATE_WALL_HOLD,
            WallPerchStatus.STATE_WALL_PIN,
        ):
            rate_limit = self.capture_max_rate_rps
        elif self.state in (
            WallPerchStatus.STATE_DETACH_ROTATE,
            WallPerchStatus.STATE_RECOVER,
            WallPerchStatus.STATE_ABORT,
        ):
            rate_limit = self.recover_max_rate_rps
        else:
            rate_limit = self.max_rate_rps
        return self._rates_safe(rate_limit) and self._vertical_speed_safe()

    @staticmethod
    def _record_hold_sample(condition, now, since):
        return record_hold_sample(condition, now, since)

    @staticmethod
    def _held_samples(fresh, since, last_sample_at, duration):
        return held_samples(fresh, since, last_sample_at, duration)

    def _front_ready(self, now):
        fresh = self._front_fresh(now)
        if not fresh:
            self.front_ready_since = None
            self.front_ready_last_sample_at = None
        return self._held_samples(
            fresh,
            self.front_ready_since,
            self.front_ready_last_sample_at,
            self.front_ready_hold_s,
        )

    def _flip_ready(self, now):
        fresh = self._front_fresh(now)
        if not fresh:
            self.flip_ready_since = None
            self.flip_ready_last_sample_at = None
        return self._held_samples(
            fresh,
            self.flip_ready_since,
            self.flip_ready_last_sample_at,
            self.flip_trigger_hold_s,
        )

    def _top_contact_ready(self, now):
        fresh = self._top_fresh(now)
        if not fresh:
            self.top_contact_since = None
            self.top_contact_last_sample_at = None
        return self._held_samples(
            fresh,
            self.top_contact_since,
            self.top_contact_last_sample_at,
            self.top_contact_hold_s,
        )

    def _top_contact_detected(self, now):
        return bool(
            self._top_fresh(now)
            and self.top_filtered_m <= self.top_contact_distance_m
        )

    def _top_clear_ready(self, now):
        fresh = self._top_fresh(now)
        if not fresh:
            self.top_clear_since = None
            self.top_clear_last_sample_at = None
        return self._held_samples(
            fresh,
            self.top_clear_since,
            self.top_clear_last_sample_at,
            self.top_clear_hold_s,
        )

    @staticmethod
    def _held_status(condition, since, last_sample_at, duration):
        return WallPerchDecisionNode._held_samples(
            condition, since, last_sample_at, duration
        )

    def _capture_attitude_references(self):
        quaternion = self._current_quaternion()
        yaw = self._euler_from_quaternion(quaternion)[2] if quaternion else 0.0
        self.q_hover, self.q_wall = self._hover_and_wall_quaternions(
            yaw, self.wall_angle_rad
        )
        self.q_contact = self.q_wall
        self.q_hold_reference = self.q_contact
        self.q_detach_start = self.q_contact

    def _enter_state(self, new_state, now, reason):
        if new_state == self.state:
            self.last_reason = reason
            return
        if (
            new_state == WallPerchStatus.STATE_WALL_CAPTURE
            and self.state == WallPerchStatus.STATE_FLIP_TO_WALL
        ):
            # Contact can arrive before the flip timer ends. Preserve the
            # current commanded attitude instead of jumping to q_wall.
            progress = clamp(self._state_age(now) / self.flip_time_s, 0.0, 1.0)
            self.q_contact = self._quat_slerp(
                self.q_hover, self.q_wall, self._smoothstep5(progress)
            )
        rospy.loginfo(
            "wall perch: %s -> %s (%s)",
            self.STATE_NAMES[self.state],
            self.STATE_NAMES[new_state],
            reason,
        )
        self.state = new_state
        self.state_entered_at = now
        self.last_reason = reason
        if new_state == WallPerchStatus.STATE_FRONT_WALL_DETECT:
            self.detach_altitude_m = None
            self.front_ready_since = None
            self.front_ready_last_sample_at = None
            self._capture_attitude_references()
        elif new_state == WallPerchStatus.STATE_STABILIZE_HOVER:
            self._capture_attitude_references()
        elif new_state == WallPerchStatus.STATE_SLOW_APPROACH:
            self.flip_ready_since = None
            self.flip_ready_last_sample_at = None
        elif new_state == WallPerchStatus.STATE_WALL_CAPTURE:
            self.top_contact_since = None
            self.top_contact_last_sample_at = None
            self.wall_pressure_thrust_command = self._wall_pressure_start_thrust()
        elif new_state == WallPerchStatus.STATE_WALL_HOLD:
            self.q_hold_reference = self._current_quaternion() or self.q_contact
            self.wall_pressure_thrust_command = clamp(
                getattr(
                    self,
                    "wall_pressure_thrust_command",
                    self._wall_pressure_nominal_thrust(),
                ),
                min(
                    self._wall_pressure_start_thrust(),
                    self._wall_pressure_nominal_thrust(),
                ),
                min(
                    getattr(self, "wall_pressure_max_thrust", self.max_thrust),
                    self.max_thrust,
                ),
            )
            self.detach_requested = False
            self.detach_stable_since = None
            self.detach_stable_samples = 0
        elif new_state == WallPerchStatus.STATE_RECOVER:
            self.top_clear_since = None
            self.top_clear_last_sample_at = None
        elif new_state == WallPerchStatus.STATE_DETACH_ROTATE:
            self.q_detach_start = self._current_quaternion() or self.q_contact
            feedback = self._height_feedback()
            self.detach_altitude_m = feedback[0] if feedback is not None else None
            self.detach_requested = False
        elif new_state == WallPerchStatus.STATE_ABORT:
            self.cancel_requested = False
        elif new_state == WallPerchStatus.STATE_EXIT:
            # Require a switch release before another maneuver. The reference
            # level-trigger would otherwise restart immediately after auto-detach.
            self.rearm_required = True
            self.front_ready_since = None
            self.front_ready_last_sample_at = None
            self.flip_ready_since = None
            self.flip_ready_last_sample_at = None
            self.top_contact_since = None
            self.top_contact_last_sample_at = None
            self.top_clear_since = None
            self.top_clear_last_sample_at = None
        elif new_state == WallPerchStatus.STATE_IDLE:
            self.detach_altitude_m = None

    def _pin_triggered(self):
        if not self.pin_enabled:
            return False
        quaternion = self._current_quaternion()
        if quaternion is None:
            return False
        return abs(self._euler_from_quaternion(quaternion)[1]) >= self.pin_pitch_rad

    def _update_state_machine(self, now):
        mode_requested = self._mode_requested(now)
        cancel = self.cancel_requested or (
            self.state not in (WallPerchStatus.STATE_IDLE, WallPerchStatus.STATE_EXIT)
            and not mode_requested
        )

        if cancel and self.state not in (
            WallPerchStatus.STATE_IDLE,
            WallPerchStatus.STATE_EXIT,
            WallPerchStatus.STATE_ABORT,
        ):
            self.fault_count += 1
            self._enter_state(WallPerchStatus.STATE_ABORT, now, "cancel or wall mode released")

        if self.state == WallPerchStatus.STATE_IDLE:
            if not mode_requested:
                self.rearm_required = False
                self.detach_requested = False
                self.cancel_requested = False
            altitude_ok = bool(
                self.odom is not None
                and math.isfinite(self.odom.pose.pose.position.z)
                and self.odom.pose.pose.position.z > self.min_altitude_m
            )
            if (
                mode_requested
                and not self.rearm_required
                and altitude_ok
                and self._safety_ok(now)
            ):
                self._enter_state(
                    WallPerchStatus.STATE_FRONT_WALL_DETECT,
                    now,
                    "wall mode armed",
                )

        elif self.state == WallPerchStatus.STATE_FRONT_WALL_DETECT:
            if not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "front detect safety failure")
            elif self._front_ready(now):
                self._enter_state(
                    WallPerchStatus.STATE_SLOW_APPROACH,
                    now,
                    "front wall ready; begin 3 deg approach",
                )

        elif self.state == WallPerchStatus.STATE_STABILIZE_HOVER:
            if not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "stabilize safety failure")
            elif (
                self._state_age(now) >= self.stabilize_time_s
                and self._attitude_recovered()
                and self._rates_safe()
            ):
                self._enter_state(WallPerchStatus.STATE_SLOW_APPROACH, now, "hover stabilized")

        elif self.state == WallPerchStatus.STATE_SLOW_APPROACH:
            if not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "approach safety failure")
            elif self._state_age(now) > self.approach_timeout_s:
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "approach timeout")
            elif not self._top_fresh(now):
                self.fault_count += 1
                self._enter_state(
                    WallPerchStatus.STATE_ABORT,
                    now,
                    "top range unavailable before flip",
                )
            elif self._flip_ready(now):
                self._enter_state(WallPerchStatus.STATE_FLIP_TO_WALL, now, "flip distance reached")

        elif self.state == WallPerchStatus.STATE_FLIP_TO_WALL:
            if not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "flip sensor or flight failure")
            elif self._pin_triggered():
                self._enter_state(WallPerchStatus.STATE_WALL_PIN, now, "pin pitch reached")
            elif self._state_age(now) >= self.flip_time_s or self._top_contact_detected(now):
                self._enter_state(WallPerchStatus.STATE_WALL_CAPTURE, now, "flip complete or contact seen")

        elif self.state == WallPerchStatus.STATE_WALL_CAPTURE:
            if not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "capture safety failure")
            elif self._pin_triggered():
                self._enter_state(WallPerchStatus.STATE_WALL_PIN, now, "pin pitch reached")
            elif self._top_contact_ready(now):
                self._enter_state(WallPerchStatus.STATE_WALL_HOLD, now, "wall contact confirmed")
            elif self._state_age(now) > self.capture_timeout_s:
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "wall capture timeout")

        elif self.state == WallPerchStatus.STATE_WALL_HOLD:
            contact_lost = bool(
                self.top_filtered_m is not None
                and self.top_filtered_m
                > getattr(
                    self,
                    "top_clear_distance_m",
                    getattr(self, "top_contact_distance_m", float("inf")),
                )
            )
            if not self._safety_ok(now) or contact_lost:
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "wall contact or safety lost")
            elif self._pin_triggered():
                self._enter_state(WallPerchStatus.STATE_WALL_PIN, now, "pin pitch reached")
            elif self.detach_requested or (
                self.hold_time_s > 0.0 and self._state_age(now) >= self.hold_time_s
            ):
                if self._detach_pose_ready(now):
                    self._enter_state(
                        WallPerchStatus.STATE_DETACH_ROTATE, now, "detach requested"
                    )
                elif self.detach_requested:
                    self.detach_requested = False
                    self.last_reason = (
                        "detach rejected: return to reference wall attitude and settle"
                    )

        elif self.state == WallPerchStatus.STATE_WALL_PIN:
            if not self._flight_ready(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "flight state lost while pinned")
            elif self.pin_hold_s > 0.0 and self._state_age(now) >= self.pin_hold_s:
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "pin hold timeout")

        elif self.state == WallPerchStatus.STATE_DETACH_ROTATE:
            if not self._safety_ok(now) or self._height_feedback() is None:
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "detach safety input lost")
            elif self._state_age(now) >= self.detach_time_s:
                self._enter_state(WallPerchStatus.STATE_RECOVER, now, "detach rotation complete")

        elif self.state == WallPerchStatus.STATE_RECOVER:
            if not self._safety_ok(now) or self._height_feedback() is None:
                self.fault_count += 1
                self._enter_state(
                    WallPerchStatus.STATE_ABORT,
                    now,
                    "recovery sensor or flight failure",
                )
                return
            stable = (
                self._attitude_recovered()
                and self._rates_safe(self.recover_max_rate_rps)
                and self._vertical_speed_safe()
            )
            if (
                self._state_age(now) >= self.recover_time_s
                and stable
                and self._top_clear_ready(now)
                and self._detach_height_settled()
            ):
                self._enter_state(WallPerchStatus.STATE_EXIT, now, "recovery complete")

        elif self.state == WallPerchStatus.STATE_EXIT:
            self._enter_state(WallPerchStatus.STATE_IDLE, now, "control released")

        elif self.state == WallPerchStatus.STATE_ABORT:
            stable = (
                self._odom_fresh(now)
                and self._attitude_recovered()
                and self._rates_safe()
                and self._vertical_speed_safe()
            )
            if self._state_age(now) >= self.recover_time_s and stable:
                self._enter_state(WallPerchStatus.STATE_EXIT, now, "abort recovery complete")

    def _candidate(self, now):
        neutral = self.state in (
            WallPerchStatus.STATE_FRONT_WALL_DETECT,
            WallPerchStatus.STATE_STABILIZE_HOVER,
        )
        quaternion = None
        thrust = math.nan
        progress = 0.0

        if self.state == WallPerchStatus.STATE_SLOW_APPROACH:
            pitch = (0.0, math.sin(self.approach_pitch_rad * 0.5), 0.0, math.cos(self.approach_pitch_rad * 0.5))
            quaternion = self._quat_normalize(self._quat_multiply(self.q_hover, pitch))
            thrust = self.approach_thrust
        elif self.state == WallPerchStatus.STATE_FLIP_TO_WALL:
            progress = clamp(self._state_age(now) / self.flip_time_s, 0.0, 1.0)
            quaternion = self._quat_slerp(
                self.q_hover, self.q_wall, self._smoothstep5(progress)
            )
            thrust = self.hover_thrust * self.flip_thrust_multiplier
        elif self.state == WallPerchStatus.STATE_WALL_CAPTURE:
            progress = clamp(self._state_age(now) / self.capture_time_s, 0.0, 1.0)
            quaternion = self.q_contact
            thrust = self._wall_pressure_thrust()
        elif self.state == WallPerchStatus.STATE_WALL_HOLD:
            followed = self._hold_follow_quaternion()
            entry = self._smoothstep5(
                self._state_age(now) / self.hold_entry_blend_s
            )
            quaternion = self._quat_slerp(self.q_contact, followed, entry)
            thrust = self._wall_pressure_thrust()
        elif self.state == WallPerchStatus.STATE_WALL_PIN:
            quaternion = self.q_contact
            thrust = self.pin_thrust
        elif self.state == WallPerchStatus.STATE_DETACH_ROTATE:
            progress = clamp(self._state_age(now) / self.detach_time_s, 0.0, 1.0)
            smoothed = self._smoothstep5(progress)
            quaternion = self._quat_slerp(
                self.q_detach_start, self.q_hover, smoothed
            )
            hold_thrust = self.hover_thrust * self.hold_thrust_multiplier
            # While nearly vertical, wall contact must still carry the weight.
            # Keep the wall-pressure command while the wheels roll on the wall.
            # Blend to altitude control as the thrust axis gains vertical
            # authority; the pressure component fades as the body levels.
            attitude = self._current_quaternion()
            vertical_fraction = (
                max(0.0, rotate_vector(attitude, (0.0, 0.0, 1.0))[2])
                if attitude is not None else 0.0
            )
            start = math.cos(self.detach_compensation_start_rad)
            full = min(1.0, max(start + 0.01, self.hover_thrust / self.max_thrust))
            blend = self._smoothstep5((vertical_fraction - start) / (full - start))
            thrust = hold_thrust + blend * (self._height_hold_thrust() - hold_thrust)
        elif self.state == WallPerchStatus.STATE_RECOVER:
            quaternion = self.q_hover
            thrust = self._height_hold_thrust()
        elif self.state == WallPerchStatus.STATE_ABORT:
            quaternion = self.q_hover
            thrust = (
                self._height_hold_thrust()
                if self.detach_altitude_m is not None and self._height_feedback() is not None
                else self.hover_thrust * self.recover_thrust_multiplier
            )

        if quaternion is not None:
            thrust = clamp(thrust, 0.1, self.max_thrust)
        return neutral, quaternion, thrust, progress

    def _build_status(self, now):
        status = WallPerchStatus()
        status.header.stamp = now
        status.state = self.state
        status.state_name = self.STATE_NAMES[self.state]
        status.enabled = self.state not in (
            WallPerchStatus.STATE_IDLE,
            WallPerchStatus.STATE_EXIT,
        )
        status.acquire_request = bool(
            self.state == WallPerchStatus.STATE_FRONT_WALL_DETECT
            and self._mode_requested(now)
            and self._safety_ok(now)
        )
        status.keep_ownership = status.enabled
        status.front_range_fresh = self._front_fresh(now)
        status.top_range_fresh = self._top_fresh(now)
        status.sensor_health_fresh = self._sensor_health_fresh(now)
        status.top_preflight_ready = self._top_preflight_ready(now)
        status.odom_fresh = self._odom_fresh(now)
        status.flight_state_fresh = self._flight_ready(now)
        status.front_ready = self._held_status(
            status.front_range_fresh
            and self.front_filtered_m < self.front_ready_distance_m,
            self.front_ready_since,
            self.front_ready_last_sample_at,
            self.front_ready_hold_s,
        )
        status.flip_ready = self._held_status(
            status.front_range_fresh
            and self.front_filtered_m <= self.flip_trigger_distance_m,
            self.flip_ready_since,
            self.flip_ready_last_sample_at,
            self.flip_trigger_hold_s,
        )
        status.top_contact_ready = self._held_status(
            status.top_range_fresh
            and self.top_filtered_m <= self.top_contact_distance_m,
            self.top_contact_since,
            self.top_contact_last_sample_at,
            self.top_contact_hold_s,
        )
        status.top_clear_ready = self._held_status(
            status.top_range_fresh
            and self.top_filtered_m >= self.top_clear_distance_m,
            self.top_clear_since,
            self.top_clear_last_sample_at,
            self.top_clear_hold_s,
        )
        status.fault_detected = bool(
            self.state == WallPerchStatus.STATE_ABORT
            or not status.odom_fresh
            or not status.flight_state_fresh
            or (status.enabled and self._front_required() and not status.front_range_fresh)
            or (self._top_required() and not status.top_range_fresh)
        )
        status.front_distance_raw_m = (
            self.front_range.range if self.front_range is not None else math.nan
        )
        status.front_distance_filtered_m = (
            self.front_filtered_m if self.front_filtered_m is not None else math.nan
        )
        status.top_distance_raw_m = (
            self.top_range.range if self.top_range is not None else math.nan
        )
        status.top_distance_filtered_m = (
            self.top_filtered_m if self.top_filtered_m is not None else math.nan
        )
        neutral, quaternion, thrust, progress = self._candidate(now)
        status.neutral_setpoint_requested = neutral
        status.attitude_setpoint_valid = quaternion is not None
        if quaternion is not None:
            status.attitude_setpoint = Quaternion(*quaternion)
        status.thrust_normalized = thrust
        status.candidate_valid = bool(
            status.enabled
            and status.flight_state_fresh
            and status.odom_fresh
            and (neutral or (quaternion is not None and math.isfinite(thrust)))
        )
        status.progress = progress
        status.fault_count = self.fault_count
        status.reason = self.last_reason
        return status

    @staticmethod
    def _build_control_candidate(status):
        candidate = AttachmentControlCandidate()
        candidate.header = status.header
        candidate.source = AttachmentControlCandidate.SOURCE_WALL
        candidate.acquire_request = status.acquire_request
        candidate.keep_ownership = status.keep_ownership
        candidate.reason = status.reason
        candidate.velocity_z_enu_mps = math.nan
        candidate.thrust_normalized = math.nan

        if status.neutral_setpoint_requested:
            candidate.kind = AttachmentControlCandidate.KIND_VERTICAL_VELOCITY
            candidate.velocity_z_enu_mps = 0.0
        elif status.attitude_setpoint_valid and math.isfinite(
            status.thrust_normalized
        ):
            candidate.kind = AttachmentControlCandidate.KIND_ATTITUDE_THRUST
            candidate.attitude_setpoint = status.attitude_setpoint
            candidate.thrust_normalized = status.thrust_normalized
        else:
            candidate.kind = AttachmentControlCandidate.KIND_NONE

        candidate.valid = bool(
            status.candidate_valid
            and candidate.kind != AttachmentControlCandidate.KIND_NONE
        )
        return candidate

    def _on_timer(self, _event):
        now = rospy.Time.now()
        if self.previous_update_at is None:
            self.control_dt = 0.02
        else:
            self.control_dt = clamp(
                (now - self.previous_update_at).to_sec(), 0.001, 0.05
            )
        self.previous_update_at = now
        self._update_state_machine(now)
        status = self._build_status(now)
        self.status_publisher.publish(status)
        self.control_candidate_publisher.publish(
            self._build_control_candidate(status)
        )


if __name__ == "__main__":
    rospy.init_node("wall_perch_decision_node")
    WallPerchDecisionNode()
    rospy.spin()
