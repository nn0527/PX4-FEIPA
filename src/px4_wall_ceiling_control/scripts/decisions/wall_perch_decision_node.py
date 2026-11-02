#!/usr/bin/env python3
"""ROS/MAVROS decision port of the reference PX4 wall_perch state machine."""

import math
import threading
from contextlib import nullcontext
import uuid
from statistics import median

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State, AttitudeTarget
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String, UInt8

from px4_wall_ceiling_control.adapters.odometry import odometry_is_finite, vertical_speed_enu_mps
from px4_wall_ceiling_control.adapters.takeoff_height import TakeoffHeightReference
from px4_wall_ceiling_control.adapters.kill_switch import install_kill_guard, kill_clear
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
from px4_wall_ceiling_control.freshness import (
    clamp, is_valid_range, is_fresh_receipt as is_fresh, is_fresh_source,
)
from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    OperatorCommand,
    SensorHealth,
    WallPerchStatus,
)


from px4_wall_ceiling_control.adapters.wall_horizontal_velocity import WallHorizontalVelocityMixin
from px4_wall_ceiling_control.adapters.wall_detach_vertical import WallDetachVerticalMixin
from px4_wall_ceiling_control.adapters.wall_detach_braking import WallDetachBrakingMixin


from px4_wall_ceiling_control.adapters.wall_simple_recovery import WallSimpleRecoveryMixin
from px4_wall_ceiling_control.domain.simple_recovery import ENTRY_TILT_DEG
from px4_wall_ceiling_control.adapters.incline_attachment import InclineAttachmentMixin


class WallPerchDecisionNode(InclineAttachmentMixin, WallSimpleRecoveryMixin, WallDetachBrakingMixin, WallDetachVerticalMixin, WallHorizontalVelocityMixin):
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
        self.feedback_lock = threading.RLock()
        install_kill_guard(self, rospy, String)
        self.maneuver_id = uuid.uuid4().hex
        self.control_sequence = 0
        self.odom_source_stamp = None
        self.manual_approach = bool(rospy.get_param("~manual_approach", False))
        self.manager_phase = None
        self.manager_phase_received_at = None
        self.manager_phase_timeout_s = 0.5
        self.auto_approach = bool(rospy.get_param("~auto_approach", False))
        if self.manual_approach and self.auto_approach:
            raise ValueError("manual and automatic approach cannot both be enabled")
        self.flip_thrust_follow_tilt = bool(rospy.get_param("~flip_thrust_follow_tilt", False))
        self.flip_vertical_support_enabled = bool(rospy.get_param("~flip_vertical_support_enabled", False) and self.auto_approach)
        self.flip_support_fade_start_rad = math.radians(float(rospy.get_param("~flip_support_fade_start_deg", 20.0)))
        self.flip_support_fade_end_rad = math.radians(float(rospy.get_param("~flip_support_fade_end_deg", 60.0)))
        self.flip_thrust_slew_per_s = float(rospy.get_param("~flip_thrust_slew_per_s", .50))
        if not (0 <= self.flip_support_fade_start_rad < self.flip_support_fade_end_rad < math.pi/2
                and math.isfinite(self.flip_thrust_slew_per_s) and 0 < self.flip_thrust_slew_per_s <= 1.):
            raise ValueError("invalid early flip vertical support limits")
        self._reset_flip_vertical_support()
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
            rospy.get_param("~flip_trigger_hold_s", 0.0)
        )
        self.top_contact_distance_m = float(
            rospy.get_param("~top_contact_distance_m", 0.05)
        )
        # Retained for the legacy top-frame diagnostic only. HOLD admission
        # uses completed trajectory + measured pitch, without top feedback.
        self.hold_entry_top_distance_check_enabled = bool(
            rospy.get_param("~hold_entry_top_distance_check_enabled", False)
        )
        self.hold_entry_min_pitch_rad = math.radians(float(
            rospy.get_param("~hold_entry_min_pitch_deg", 80.0)
        ))
        if (not math.isfinite(self.hold_entry_min_pitch_rad)
                or not 0.0 < self.hold_entry_min_pitch_rad <= math.pi / 2.):
            raise ValueError("hold_entry_min_pitch_deg must be in (0, 90]")
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
        # A stalled callback must not catch up the attitude trajectory in one
        # command. The hard deadline remains wall-clock based and independent.
        self.flip_duration_timeout_s = float(rospy.get_param(
            "~flip_duration_timeout_s", max(6.0, 2.0 * self.flip_time_s)))
        if (not math.isfinite(self.flip_duration_timeout_s)
                or self.flip_duration_timeout_s <= self.flip_time_s):
            raise ValueError("flip_duration_timeout_s must exceed flip_time_s")
        self.flip_max_tracking_error_rad = math.radians(float(rospy.get_param(
            "~flip_max_tracking_error_deg", 15.0)))
        if (not math.isfinite(self.flip_max_tracking_error_rad)
                or not 0.0 < self.flip_max_tracking_error_rad <= math.pi / 2.):
            raise ValueError("flip_max_tracking_error_deg must be in (0, 90]")
        self.flip_tracking_limited = False
        self.flip_elapsed_s = 0.0
        self.flip_step_at = None
        self.flip_step_dt = 0.0
        self.flip_clock_reversed = False
        self.flip_thrust_evaluated_at = None
        self.q_flip_start = None
        # Retain the late-flip top window for diagnostics/pressure feedback;
        # it no longer authorizes HOLD.
        self.flip_top_start_fraction = clamp(
            float(rospy.get_param("~flip_top_start_fraction", 0.90)), 0.0, 1.0
        )
        self.top_hold_accept_after = None
        self.top_hold_frame_at = None
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
        self.detach_axis_error_rad = math.radians(
            float(rospy.get_param("~detach_axis_error_deg", 15.0))
        )
        self.detach_max_rate_rps = float(
            rospy.get_param("~detach_max_rate_rps", math.radians(25.0))
        )
        self.detach_vertical_speed_check_enabled = bool(
            rospy.get_param("~detach_vertical_speed_check_enabled", False)
        )
        self.detach_max_abs_vertical_speed_mps = float(
            rospy.get_param("~detach_max_abs_vertical_speed_mps", 0.20)
        )
        self.detach_stable_hold_s = float(
            rospy.get_param("~detach_stable_hold_s", 0.50)
        )
        self.detach_time_s = max(
            0.01, float(rospy.get_param("~detach_time_s", 1.0))
        )
        self.detach_request_timeout_s = float(rospy.get_param("~detach_request_timeout_s", 8.0))
        self.detach_duration_timeout_s = float(rospy.get_param("~detach_duration_timeout_s", 8.0))
        self.detach_max_tracking_error_rad = math.radians(float(rospy.get_param("~detach_max_tracking_error_deg", 30.0)))
        self.detach_recovered_tilt_rad = math.radians(float(rospy.get_param("~detach_recovered_tilt_deg", 5.0)))
        self.detach_recovered_rate_rps = math.radians(float(rospy.get_param("~detach_recovered_rate_deg_s", 10.0)))
        self.detach_recovered_hold_s = float(rospy.get_param("~detach_recovered_hold_s", .30))
        self.recover_timeout_s = float(rospy.get_param("~recover_timeout_s", 15.0))
        if (not all(math.isfinite(v) and v > 0. for v in (
                self.detach_request_timeout_s, self.detach_duration_timeout_s,
                self.detach_max_tracking_error_rad, self.detach_recovered_tilt_rad,
                self.detach_recovered_rate_rps, self.detach_recovered_hold_s,
                self.recover_timeout_s))
                or self.detach_duration_timeout_s <= self.detach_time_s
                or self.detach_max_tracking_error_rad > math.pi/2.
                or self.detach_recovered_tilt_rad > math.radians(5.)):
            raise ValueError("invalid detach duration or recovery confirmation limits")
        self.detach_elapsed_s = self.detach_step_dt = 0.
        self.detach_step_at = None
        self.detach_clock_reversed = self.detach_tracking_limited = False
        self.detach_recovered_since = self.detach_recovered_last_stamp = None
        self.detach_recovered_samples = 0
        self._init_detach_braking(rospy.get_param)
        self.recover_time_s = float(rospy.get_param("~recover_time_s", 0.50))
        if not math.isfinite(self.recover_time_s) or not 0. <= self.recover_time_s < self.recover_timeout_s:
            raise ValueError("recover_time_s must be below recover_timeout_s")
        # Approach/recovery vertical velocity loop; all thrust values are normalized.
        self.recovery_max_up_speed_mps = float(rospy.get_param("~recovery_max_up_speed_mps", 0.10))
        self.recovery_max_down_speed_mps = float(rospy.get_param("~recovery_max_down_speed_mps", 0.15))
        self.recovery_height_to_speed_kp = float(rospy.get_param("~recovery_height_to_speed_kp", 0.50))
        self.recovery_velocity_kp = float(rospy.get_param("~recovery_velocity_kp", 0.30))
        self.recovery_positive_compensation_max = float(rospy.get_param("~recovery_positive_compensation_max", 0.08))
        self.recovery_max_thrust = float(rospy.get_param("~recovery_max_thrust", 0.60))
        if (not all(math.isfinite(v) and v >= 0.0 for v in (
                self.recovery_max_up_speed_mps, self.recovery_max_down_speed_mps,
                self.recovery_height_to_speed_kp, self.recovery_velocity_kp,
                self.recovery_positive_compensation_max))
                or not math.isfinite(self.recovery_max_thrust)
                or not 0.1 <= self.recovery_max_thrust <= 1.0):
            raise ValueError("invalid recovery velocity/thrust limits")

        self.wall_angle_rad = math.radians(
            float(rospy.get_param("~wall_angle_deg", 90.0))
        )
        self.approach_pitch_rad = math.radians(
            float(rospy.get_param("~approach_pitch_deg", 3.0))
        )
        self.hover_thrust = clamp(
            float(rospy.get_param("~hover_thrust", 0.49)), 0.1, 0.9
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
        self.bench_detach_hover_thrust = float(rospy.get_param("~bench_detach_hover_thrust", self.hover_thrust))
        self.bench_detach_release_thrust = float(rospy.get_param("~bench_detach_release_thrust", self.hover_thrust))
        self.bench_detach_thrust_slew_per_s = float(rospy.get_param("~bench_detach_thrust_slew_per_s", 0.45))
        if not math.isfinite(self.bench_detach_release_thrust) or not 0.1 <= self.bench_detach_release_thrust <= self.bench_detach_hover_thrust:
            raise ValueError("bench detach release thrust must be finite and no greater than level support")
        if not math.isfinite(self.bench_detach_thrust_slew_per_s) or self.bench_detach_thrust_slew_per_s <= 0:
            raise ValueError("bench detach thrust slew must be finite and positive")
        if not math.isfinite(self.bench_detach_hover_thrust) or not 0.1 <= self.bench_detach_hover_thrust < self.max_thrust:
            raise ValueError("bench_detach_hover_thrust must be finite and below max_thrust")
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
        self.flip_max_pitch_rate_rps = float(rospy.get_param("~flip_max_pitch_rate_rps", self.flip_max_rate_rps))
        self.capture_max_pitch_rate_rps = float(rospy.get_param("~capture_max_pitch_rate_rps", self.capture_max_rate_rps))
        if not all(math.isfinite(value) and value > 0 for value in (self.flip_max_pitch_rate_rps, self.capture_max_pitch_rate_rps)):
            raise ValueError("pitch rate limits must be finite and positive")
        self.recover_max_rate_rps = float(
            rospy.get_param("~recover_max_rate_rps", 3.0)
        )
        self.max_down_velocity_mps = float(
            rospy.get_param("~max_down_velocity_mps", 0.80)
        )
        if (not math.isfinite(self.detach_max_abs_vertical_speed_mps)
                or not 0. < self.detach_max_abs_vertical_speed_mps <= self.max_down_velocity_mps):
            raise ValueError("detach admission speed must be positive and no greater than descent protection")
        self.min_altitude_m = float(rospy.get_param("~min_altitude_m", 0.30))
        self.altitude_reference = rospy.get_param("~altitude_reference", "arming_ground")
        self.bench_wait_for_ranges = bool(rospy.get_param("~bench_wait_for_ranges", False))
        self.ignore_range_loss_during_attachment = bool(
            rospy.get_param("~ignore_range_loss_during_attachment", False)
        )
        self.ignore_range_error_258 = bool(
            rospy.get_param("~ignore_range_error_258", False)
        )
        self.ignore_range_loss_during_hold = bool(
            rospy.get_param("~ignore_range_loss_during_hold", False)
        )
        self.ignore_down_velocity_during_hold = bool(
            rospy.get_param("~ignore_down_velocity_during_hold", False)
        )
        self.manual_wall_recovery_enabled = bool(
            rospy.get_param("~manual_wall_recovery_enabled", False)
        )
        if self.ignore_range_loss_during_hold:
            rospy.logwarn("WALL_HOLD/WALL_PIN range-loss abort checks disabled; actual range and health remain unchanged")
        if self.altitude_reference not in ("arming_ground", "local_origin"):
            raise ValueError("altitude_reference must be arming_ground or local_origin")
        self.takeoff_height = TakeoffHeightReference()
        self.require_min_altitude = bool(
            rospy.get_param("~require_min_altitude", True)
        )
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
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 1.50))
        self.operator_timeout_s = float(
            rospy.get_param("~operator_timeout_s", 0.50)
        )
        self.filter_alpha = clamp(
            float(rospy.get_param("~range_filter_alpha", 0.30)), 0.0, 1.0
        )
        # Top lidar changes viewing direction during the flip: do not carry
        # metre-scale history into centimetre-scale contact confirmation.
        self.top_flip_filter_alpha = clamp(
            float(rospy.get_param("~top_flip_filter_alpha", 0.70)), 0.0, 1.0
        )
        self.top_hold_filter_alpha = clamp(
            float(rospy.get_param("~top_hold_filter_alpha", self.filter_alpha)), 0.0, 1.0
        )
        self.top_capture_window_samples = max(
            3, int(rospy.get_param("~top_capture_window_samples", 3))
        )
        self.top_capture_samples = []
        self.top_capture_near_samples = 0
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
        self.front_source_stamp = None
        self.approach_abort_front_distance_m = float(rospy.get_param("~approach_abort_front_distance_m", 1.8))
        self.approach_far_sample = None
        self.front_filtered_m = None
        self.top_range = None
        self.top_received_at = None
        self.top_source_stamp = None
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
        self.detach_altitude_m = None  # Rotation-only legacy reference.
        self.approach_altitude_m = None
        self.approach_takeover_source = None
        self.approach_takeover_thrust = None
        self.approach_base_vertical_thrust = None
        self.recovery_altitude_m = None
        self.recovery_wait_for_level = False
        self.vertical_reference_source = "none"
        self.vertical_target_speed = math.nan
        self.vertical_correction = math.nan
        self.hold_achieved = False
        self.reached_state = WallPerchStatus.STATE_IDLE
        self.detach_completed = False
        self.fault_recovery_completed = False
        self.recovery_started_at = None
        self.detach_stable_since = None
        self.detach_stable_last_stamp = None
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
        rospy.Subscriber("/attachment/flight_phase", String, self._on_manager_phase, queue_size=10)
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
            queue_size=1,
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
        self._init_horizontal_velocity()
        self._init_detach_vertical()
        if not self.horizontal_velocity_enabled:
            # Flip and detach continuity need final output feedback even when
            # horizontal velocity control is disabled in a launch profile.
            rospy.Subscriber('/attachment/output_owner', UInt8,
                             self._on_horizontal_output_owner, queue_size=10)
            rospy.Subscriber('/mavros/setpoint_raw/attitude', AttitudeTarget,
                             self._on_horizontal_output, queue_size=10)
        self.approach_use_takeover_thrust = bool(
            rospy.get_param("~approach_use_takeover_thrust", True)
            and self.auto_approach and self.horizontal_velocity_enabled)
        self.approach_source_timeout_s = float(rospy.get_param(
            "/attachment_output_mux_node/fcu_target_timeout_s", .35))
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self._init_incline()
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

    def _wall_pressure_thrust(self, now=None):
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
        now = rospy.Time.now() if now is None else now
        if top_distance is None or not self._top_fresh(now):
            # A brief measurement outage holds the last bounded pressure;
            # cached distance never drives another increase or decrease.
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

    def _accept_range_source(self, direction, message, now):
        stamp = message.header.stamp
        previous = getattr(self, direction + "_source_stamp", None)
        if (not is_fresh_source(stamp, self.sensor_timeout_s, now, future_tolerance_s=0.)
                or (previous is not None and stamp <= previous)):
            return False
        setattr(self, direction + "_source_stamp", stamp)
        return True

    def _on_front_range(self, message):
        if not is_valid_range(message):
            if getattr(self, "auto_approach", False) and self._approach_stage():
                self.front_ready_since = self.front_ready_last_sample_at = None
                self.flip_ready_since = self.flip_ready_last_sample_at = None
            return
        now = rospy.Time.now()
        if not self._accept_range_source("front", message, now):
            return
        previous = (
            None
            if not self._front_fresh(now)
            else self.front_filtered_m
        )
        if previous is None:
            self.front_ready_since = self.front_ready_last_sample_at = None
            self.flip_ready_since = self.flip_ready_last_sample_at = None
        if self._approach_stage() and self._mode_requested(now) and message.range > getattr(self, "approach_abort_front_distance_m", 1.8):
            self.approach_far_sample = (now, float(message.range))
        self.front_range = message
        self.front_received_at = now
        self.front_filtered_m = self._filter_range(message, previous)
        (
            self.front_ready_since,
            self.front_ready_last_sample_at,
        ) = self._record_hold_sample(
            self.front_filtered_m <= self.front_ready_distance_m
            and (not getattr(self, "auto_approach", False) or not self._approach_stage() or self._auto_approach_ranges_ready(now)),
            message.header.stamp,
            self.front_ready_since,
        )
        (
            self.flip_ready_since,
            self.flip_ready_last_sample_at,
        ) = self._record_hold_sample(
            (message.range <= self.flip_trigger_distance_m + 1.e-8
             if self.flip_trigger_hold_s <= 0.0 else
             self.front_filtered_m <= self.flip_trigger_distance_m)
            and (not getattr(self, "auto_approach", False) or not self._approach_stage() or self._auto_approach_ranges_ready(now)),
            message.header.stamp,
            self.flip_ready_since,
        )

    def _on_top_range(self, message):
        now = rospy.Time.now()
        if self.state == WallPerchStatus.STATE_FLIP_TO_WALL and not self._late_top_window_open(now):
            return  # The early flip does not consume top measurements.
        if not is_valid_range(message):
            # Top failures must never reset the forward-only flip gate.
            # A bad sample must not count toward continuous contact confirmation.
            # Keep the last valid range and its original timestamp for the
            # bounded bench safety grace; never refresh them with invalid data.
            self.top_contact_since = None
            self.top_contact_last_sample_at = None
            self.top_capture_samples = []
            self.top_capture_near_samples = 0
            return
        now = rospy.Time.now()
        if not self._accept_range_source("top", message, now):
            return
        previous = (
            None
            if not self._top_fresh(now)
            else self.top_filtered_m
        )
        self.top_range = message
        self.top_received_at = now
        if self.state in (WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE):
            after = getattr(self, "top_hold_accept_after", None)
            stamp = message.header.stamp
            if after is not None and stamp >= after:
                self.top_hold_frame_at = stamp
        if previous is None:
            self.top_capture_samples = []
            self.top_capture_near_samples = 0
            self.top_contact_since = None
            self.top_contact_last_sample_at = None
        if self.state == WallPerchStatus.STATE_WALL_CAPTURE:
            self.top_capture_samples.append(message.range)
            self.top_capture_samples = self.top_capture_samples[-self.top_capture_window_samples:]
            self.top_filtered_m = median(self.top_capture_samples)
            self.top_capture_near_samples = (
                self.top_capture_near_samples + 1
                if message.range <= self.top_contact_distance_m else 0
            )
            contact = (message.range <= self.top_contact_distance_m
                       and self.top_filtered_m <= self.top_contact_distance_m)
        else:
            alpha = (self.top_flip_filter_alpha
                     if self.state == WallPerchStatus.STATE_FLIP_TO_WALL
                     else self.top_hold_filter_alpha
                     if self.state in (WallPerchStatus.STATE_WALL_HOLD,
                                       WallPerchStatus.STATE_WALL_PIN)
                     else self.filter_alpha)
            self.top_filtered_m = (message.range if previous is None
                                   else previous + alpha * (message.range - previous))
            contact = self.top_filtered_m <= self.top_contact_distance_m
        (
            self.top_contact_since,
            self.top_contact_last_sample_at,
        ) = self._record_hold_sample(
            contact,
            message.header.stamp,
            self.top_contact_since,
        )
        (
            self.top_clear_since,
            self.top_clear_last_sample_at,
        ) = self._record_hold_sample(
            self.top_filtered_m >= self.top_clear_distance_m,
            message.header.stamp,
            self.top_clear_since,
        )

    def _on_sensor_health(self, message):
        self.sensor_health = message
        self.sensor_health_received_at = rospy.Time.now()
        if getattr(self, "auto_approach", False) and self._approach_stage() and not message.front_valid:
            self.front_ready_since = self.front_ready_last_sample_at = None
            self.flip_ready_since = self.flip_ready_last_sample_at = None

    def _on_odom(self, message):
        with getattr(self, 'feedback_lock', nullcontext()):
            self._accept_odom(message)

    def _accept_odom(self, message):
        now = rospy.Time.now()
        stamp = message.header.stamp
        previous = getattr(self, "odom_source_stamp", None)
        if (not is_fresh_source(stamp, self.odom_timeout_s, now)
                or (previous is not None and stamp <= previous)):
            # Receiving a replay does not refresh feedback or stability evidence.
            return
        gap = (not is_fresh(getattr(self, "odom_received_at", None), self.odom_timeout_s, now)
               or (previous is not None and (stamp - previous).to_sec() > self.odom_timeout_s))
        self.odom = message
        self.odom_source_stamp = stamp
        self.odom_received_at = now
        self._observe_range_pose(message, now)
        if hasattr(self, "takeoff_height"):
            flight = self.flight_state
            confirmed = bool(flight is not None and flight.connected and not flight.armed
                and is_fresh(self.flight_state_received_at, self.state_timeout_s, now))
            self.takeoff_height.observe_odom(message.pose.pose.position.z, now.to_sec(), confirmed)
        if self.state == WallPerchStatus.STATE_WALL_HOLD:
            if self._detach_pose_valid():
                if self.detach_stable_since is None or gap:
                    self.detach_stable_since = stamp
                    self.detach_stable_samples = 0
                self.detach_stable_last_stamp = stamp
                self.detach_stable_samples += 1
            else:
                self.detach_stable_since = None
                self.detach_stable_last_stamp = None
                self.detach_stable_samples = 0
        if self.state == WallPerchStatus.STATE_DETACH_ROTATE and not getattr(self, "bench_wait_for_ranges", False):
            if self._detach_rotation_pose_valid():
                if getattr(self, "detach_recovered_since", None) is None or gap:
                    self.detach_recovered_since = stamp
                    self.detach_recovered_samples = 0
                self.detach_recovered_last_stamp = stamp
                self.detach_recovered_samples += 1
            else:
                self.detach_recovered_since = self.detach_recovered_last_stamp = None
                self.detach_recovered_samples = 0

    def _capture_approach_takeover_thrust(self, now):
        # Snapshot only the last POSCTL feedback. OFFBOARD echoes are never learned.
        self.approach_takeover_thrust = self.approach_base_vertical_thrust = None
        sample = getattr(self, "approach_takeover_source", None)
        if sample is None or not self._odom_fresh(now):
            return
        target, received_at = sample
        if (not is_fresh(received_at, self.approach_source_timeout_s, now)
                or not is_fresh_source(target.header.stamp, self.approach_source_timeout_s, now)
                or target.type_mask & AttitudeTarget.IGNORE_THRUST
                or not math.isfinite(target.thrust) or not .1 <= target.thrust <= 1.):
            return
        attitude = self._current_quaternion()
        if attitude is None:
            return
        vertical_fraction = rotate_vector(attitude, (0., 0., 1.))[2]
        if not math.isfinite(vertical_fraction) or vertical_fraction <= 0:
            return
        self.approach_takeover_thrust = float(target.thrust)
        self.approach_base_vertical_thrust = target.thrust * vertical_fraction
        rospy.loginfo("wall approach: captured takeover thrust %.3f, vertical support %.3f",
                      self.approach_takeover_thrust, self.approach_base_vertical_thrust)

    def _on_state(self, message):
        with getattr(self, 'feedback_lock', nullcontext()):
            self._accept_flight_state(message)

    def _accept_flight_state(self, message):
        if self._range_task_active() and message.connected and message.armed:
            now = rospy.Time.now()
            previous = getattr(self, 'detach_range_last_flight_source', None)
            if (not is_fresh_source(message.header.stamp, self.state_timeout_s, now)
                    or (previous is not None and message.header.stamp <= previous)):
                return
            self.detach_range_last_flight_source = message.header.stamp
        previous_mode = getattr(getattr(self, "flight_state", None), "mode", "")
        self.flight_state = message
        self.flight_state_received_at = rospy.Time.now()
        if message.connected and message.armed and message.mode == "OFFBOARD" and previous_mode != "OFFBOARD":
            if getattr(self, "approach_use_takeover_thrust", False) and previous_mode.upper() == "POSCTL":
                self._capture_approach_takeover_thrust(self.flight_state_received_at)
            if self._odom_fresh(self.flight_state_received_at):
                feedback = self._height_feedback()
                if feedback is not None:
                    self.approach_altitude_m = feedback[0]
                    rospy.loginfo("wall vertical: takeover height %.3f m", feedback[0])
        if (not message.connected or not message.armed
                or (previous_mode.upper() == "OFFBOARD" and message.mode.upper() != "OFFBOARD")):
            if hasattr(self, "_reset_recovery_horizontal"):
                self._reset_recovery_horizontal("退出本次OFFBOARD：取消退墙动作", clear_departure=True)
        if not message.armed:
            if (getattr(self, 'detach_range_enabled', False)
                    and getattr(self, 'detach_task_phase', 'NONE') not in ('NONE', 'COMPLETED')):
                self.detach_range_faulted = True
                self.detach_task_phase = 'FAULT'
                self.detach_range_block_reason = '飞控上锁，本轮未正常完成'
            self._reset_flip_vertical_support()
            self._reset_detach_vertical()
            self.approach_takeover_thrust = self.approach_base_vertical_thrust = None
        if hasattr(self, "takeoff_height"):
            self.takeoff_height.observe_state(message.connected, message.armed,
                                              self.flight_state_received_at.to_sec())

    def _entry_height(self):
        if self.odom is None:
            return None
        z = self.odom.pose.pose.position.z
        if getattr(self, "altitude_reference", "local_origin") == "arming_ground":
            return self.takeoff_height.height(z)
        return z if math.isfinite(z) else None

    def _entry_height_ready(self):
        height = self._entry_height()
        return bool(not getattr(self, "require_min_altitude", True) or
                    (height is not None and height > self.min_altitude_m))

    def _on_operator(self, message):
        with getattr(self, 'feedback_lock', nullcontext()):
            if hasattr(self, 'incline'):
                self._incline_on_operator(message)
            self.operator = message
            self.operator_received_at = rospy.Time.now()

    def _on_detach(self, message):
        with getattr(self, 'feedback_lock', nullcontext()):
            if getattr(self, 'incline_selected', False):
                if message.data and self.incline.phase == 'HOLD':
                    self.detach_requested = True
                    self.detach_requested_at = rospy.Time.now()
                return  # Do not initialize the wall range-height task.
            if (message.data and self.state == WallPerchStatus.STATE_WALL_HOLD
                    and not self.detach_requested):
                self.detach_requested = True
                self.detach_requested_at = rospy.Time.now()
                if getattr(self, 'detach_range_enabled', False):
                    self.detach_task_phase = 'ROTATING_TO_RANGE_ENTRY'

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

    def _simple_departure(self):
        return bool(getattr(self, 'detach_range_enabled', False)
                    and not getattr(self, 'bench_wait_for_ranges', False))

    def _detach_pose_valid(self):
        measured = self._current_quaternion()
        range_departure = getattr(self, 'detach_range_enabled', False)
        if measured is None or (not range_departure and self._height_feedback() is None):
            return False
        if self._simple_departure():
            return True  # Fresh attitude/authority/output are checked by admission.
        followed = self._hold_follow_quaternion()
        if quaternion_angle(measured, followed) > self.detach_axis_error_rad:
            return False
        if not self._rates_safe(self.detach_max_rate_rps):
            return False
        if range_departure:
            return True  # Local Z and fused vertical speed do not gate range departure.
        # Wall attachment can bias estimated velocity; admission may ignore its magnitude.
        if not getattr(self, "detach_vertical_speed_check_enabled", False):
            return True
        velocity_z = vertical_speed_enu_mps(
            self.odom, self.odom_twist_in_child_frame
        )
        return bool(
            math.isfinite(velocity_z)
            and (getattr(self, "bench_wait_for_ranges", False) or
                 abs(velocity_z) <= min(self.detach_max_abs_vertical_speed_mps,
                                       self.max_down_velocity_mps))
        )

    def _detach_pose_ready(self, now):
        if self._simple_departure():
            return self._odom_fresh(now) and self._detach_pose_valid()
        return bool(
            self._odom_fresh(now)
            and self._state_age(now) >= self.hold_entry_blend_s
            and self.detach_stable_since is not None
            and self.detach_stable_samples >= 2
            and getattr(self, "detach_stable_last_stamp", None) is not None
            and (self.detach_stable_last_stamp - self.detach_stable_since).to_sec() >= self.detach_stable_hold_s
            and self._detach_pose_valid()
        )

    def _detach_start_ready(self, now):
        if getattr(self, 'detach_range_enabled', False) and not self._range_config().installation_ready:
            return False
        if not self._detach_pose_ready(now):
            return False
        if getattr(self, "bench_wait_for_ranges", False):
            return True
        return ((self._simple_departure() or self._detach_horizontal_reference_ready())
                and self._valid_wall_output(now) is not None
                and self._detach_vertical_feedback_ok(now))

    def _detach_horizontal_reference_ready(self):
        # Vertical wall flight can temporarily lose optical flow. Admission
        # needs the frozen takeover frame, not a currently level flow solution.
        equilibrium = getattr(self, "horizontal_equilibrium", None)
        yaw = getattr(self, "horizontal_takeover_yaw", None)
        return bool(getattr(self, "horizontal_velocity_enabled", False)
                    and equilibrium is not None and equilibrium.ready and equilibrium.frozen
                    and yaw is not None and math.isfinite(yaw))

    def _state_age(self, now):
        return max(0.0, (now - self.state_entered_at).to_sec())

    def _detach_block_reason(self, now):
        if getattr(self, 'detach_range_enabled', False) and not self._range_config().installation_ready:
            return '待核对下视测距设备量程、安装轴和上游几何修正'
        if not self._odom_fresh(now) or (not getattr(self, 'detach_range_enabled', False)
                                         and self._height_feedback() is None):
            return "离墙高度／速度反馈无效或过期"
        q = self._current_quaternion()
        if q is None:
            return "离墙姿态反馈无效"
        if not getattr(self, "bench_wait_for_ranges", False):
            if not self._simple_departure() and not self._detach_horizontal_reference_ready():
                return "无法正常退墙：水平闭环未启用或本次接管平衡姿态／航向未锁定"
            if self._valid_wall_output(now) is None:
                return "等待新鲜有效的侧吸最终姿态／推力输出"
            if not self._detach_vertical_feedback_ok(now):
                return self.detach_vertical_reason
        if self._simple_departure():
            return '等待新鲜姿态、控制权限和实际输出；不等待局部高度、光流或稳定窗口'
        axis = math.degrees(quaternion_angle(q, self._hold_follow_quaternion()))
        if axis > math.degrees(self.detach_axis_error_rad):
            return "离墙旋转轴偏差 {:.1f}°，需 ≤ {:.1f}°".format(axis, math.degrees(self.detach_axis_error_rad))
        omega = self.odom.twist.twist.angular
        rate = max(abs(omega.x), abs(omega.y), abs(omega.z))
        if rate > self.detach_max_rate_rps:
            return "角速度 {:.2f} rad/s，需 ≤ {:.2f} rad/s".format(rate, self.detach_max_rate_rps)
        if getattr(self, 'detach_range_enabled', False):
            return "等待姿态和角速度连续稳定 {:.2f} 秒；仅支撑等待测距".format(self.detach_stable_hold_s)
        speed = vertical_speed_enu_mps(self.odom, self.odom_twist_in_child_frame)
        if (getattr(self, "detach_vertical_speed_check_enabled", False)
                and not getattr(self, "bench_wait_for_ranges", False)
                and abs(speed) > self.detach_max_abs_vertical_speed_mps):
            return "竖直速度估算 {:+.2f} m/s，需 |v| ≤ {:.2f} m/s".format(speed, self.detach_max_abs_vertical_speed_mps)
        subject = "姿态和速度" if self.detach_vertical_speed_check_enabled else "姿态和角速度（竖直速度准入已屏蔽）"
        return "等待{}连续稳定 {:.2f} 秒".format(subject, self.detach_stable_hold_s)

    def _mode_requested(self, now):
        return bool(
            self.decision_enabled
            and is_fresh(self.operator_received_at, self.operator_timeout_s, now)
            and self.operator is not None
            and self.operator.enable_control
            and self.operator.mode in (OperatorCommand.MODE_WALL, OperatorCommand.MODE_INCLINE)
        )

    def _front_fresh(self, now):
        return bool(
            is_fresh(self.front_received_at, self.sensor_timeout_s, now)
            and getattr(self, "front_range", None) is not None
            and is_fresh_source(self.front_range.header.stamp, self.sensor_timeout_s, now, future_tolerance_s=0.)
            and self.front_filtered_m is not None
        )

    def _top_fresh(self, now):
        return bool(
            is_fresh(self.top_received_at, self.sensor_timeout_s, now)
            and getattr(self, "top_range", None) is not None
            and is_fresh_source(self.top_range.header.stamp, self.sensor_timeout_s, now, future_tolerance_s=0.)
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

    def _ignore_hold_range_loss(self):
        return bool(
            getattr(self, "ignore_range_loss_during_hold", False)
            and self.state in (WallPerchStatus.STATE_WALL_HOLD,
                               WallPerchStatus.STATE_WALL_PIN)
        )

    def _ignore_attachment_range_loss(self):
        return bool(
            getattr(self, "ignore_range_loss_during_attachment", False)
            and self.state in (
                WallPerchStatus.STATE_IDLE,
                WallPerchStatus.STATE_FRONT_WALL_DETECT,
                WallPerchStatus.STATE_STABILIZE_HOVER,
                WallPerchStatus.STATE_SLOW_APPROACH,
                WallPerchStatus.STATE_FLIP_TO_WALL,
                WallPerchStatus.STATE_WALL_CAPTURE,
                WallPerchStatus.STATE_WALL_HOLD,
                WallPerchStatus.STATE_WALL_PIN,
                WallPerchStatus.STATE_DETACH_ROTATE,
                WallPerchStatus.STATE_RECOVER,
                WallPerchStatus.STATE_ABORT,
                WallPerchStatus.STATE_EXIT,
            )
        )

    def _range_valid_for_safety(self, side, now):
        if not self._sensor_health_fresh(now):
            return False
        health = self.sensor_health
        if getattr(health, side + "_valid", False):
            return True
        fresh = self._top_fresh(now) if side == "ceiling" else self._front_fresh(now)
        return bool(
            getattr(self, "ignore_range_error_258", False)
            and getattr(health, side + "_error_code", 0) == 258
            and fresh
        )

    def _top_preflight_ready(self, now):
        if self._approach_stage() or self.state in (
            WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE,
        ):
            return True  # Top input is not an entry/flip safety prerequisite.
        if self._ignore_hold_range_loss() or self._ignore_attachment_range_loss():
            return True
        if getattr(self, "bench_wait_for_ranges", False) and self.state in (
            WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE,
            WallPerchStatus.STATE_WALL_HOLD, WallPerchStatus.STATE_WALL_PIN,
        ):
            # Ignore the instantaneous invalid-frame bit only while a recent
            # valid sample and the sensor manager heartbeat remain available.
            return self._sensor_health_fresh(now) and self._top_fresh(now)
        if getattr(self, "bench_wait_for_ranges", False) and self.state in (
            WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER,
            WallPerchStatus.STATE_ABORT, WallPerchStatus.STATE_EXIT,
        ):
            return True
        if self._bench_waiting():
            return True
        if not self.require_top_sensor_on_acquire:
            return True
        return bool(
            self._sensor_health_fresh(now)
            and self._range_valid_for_safety("ceiling", now)
        )

    def _odom_fresh(self, now):
        return bool(
            is_fresh(self.odom_received_at, self.odom_timeout_s, now)
            and getattr(self, "odom", None) is not None
            and is_fresh_source(self.odom.header.stamp, self.odom_timeout_s, now)
            and self._current_quaternion() is not None
        )

    def _flight_ready(self, now):
        if not kill_clear(self, now):
            return False
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
        if self._simple_departure() and (self.state == WallPerchStatus.STATE_DETACH_ROTATE
                                         or self._locked_recovery()):
            return False
        if self._approach_stage() or self.state in (
            WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE,
        ):
            return False  # Wait for a late valid frame; absence is not an immediate abort.
        if self._ignore_hold_range_loss() or self._ignore_attachment_range_loss():
            return False
        if getattr(self, "bench_wait_for_ranges", False) and self.state in (
            WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER,
        ):
            return False
        return self.state in (
            WallPerchStatus.STATE_FLIP_TO_WALL,
            WallPerchStatus.STATE_WALL_CAPTURE,
            WallPerchStatus.STATE_WALL_HOLD,
            WallPerchStatus.STATE_WALL_PIN,
            WallPerchStatus.STATE_DETACH_ROTATE,
            WallPerchStatus.STATE_RECOVER,
        )

    def _front_required(self):
        if self._bench_waiting() or self._ignore_attachment_range_loss():
            return False
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
        pitch_limit = limit
        if getattr(self, "state", None) == WallPerchStatus.STATE_FLIP_TO_WALL:
            pitch_limit = getattr(self, "flip_max_pitch_rate_rps", limit)
        elif getattr(self, "state", None) == WallPerchStatus.STATE_WALL_CAPTURE:
            pitch_limit = getattr(self, "capture_max_pitch_rate_rps", limit)
        return all(
            math.isfinite(value) and abs(value) <= axis_limit
            for value, axis_limit in ((rate.x, limit), (rate.y, pitch_limit), (rate.z, limit))
        )

    def _bench_waiting(self):
        return bool(getattr(self, "bench_wait_for_ranges", False) and self.state in (
            WallPerchStatus.STATE_IDLE, WallPerchStatus.STATE_FRONT_WALL_DETECT,
            WallPerchStatus.STATE_SLOW_APPROACH))

    def _approach_stage(self):
        return getattr(self, "state", None) in (
            WallPerchStatus.STATE_IDLE, WallPerchStatus.STATE_FRONT_WALL_DETECT,
            WallPerchStatus.STATE_STABILIZE_HOVER, WallPerchStatus.STATE_SLOW_APPROACH,
        )

    def _approach_far_distance(self, now):
        sample = getattr(self, "approach_far_sample", None)
        if sample is not None and is_fresh(sample[0], self.sensor_timeout_s, now):
            return sample[1]
        # Also check the latest valid input when a request is first enabled.
        message = getattr(self, "front_range", None)
        if (message is not None and self._front_fresh(now) and is_valid_range(message)
                and self._sensor_health_fresh(now) and self.sensor_health.front_valid
                and message.range > getattr(self, "approach_abort_front_distance_m", 1.8)):
            return float(message.range)
        return None

    def _on_manager_phase(self, message):
        self.manager_phase = message.data
        self.manager_phase_received_at = rospy.Time.now()

    def _auto_takeover_confirmed(self, now):
        received = self.manager_phase_received_at
        return bool(self.manager_phase == "ACTIVE" and received is not None
                    and 0.0 <= (now - received).to_sec() <= self.manager_phase_timeout_s)

    def _approach_motion_allowed(self, now):
        # Range validity gates contact transitions, not motion in bypass mode.
        return bool(getattr(self, "ignore_range_loss_during_attachment", False)
                    or self._auto_approach_ranges_ready(now))

    def _front_flip_input_ready(self, now):
        # Only real, fresh forward input authorizes the flip. Top health is unrelated.
        return bool(self._sensor_health_fresh(now) and self.sensor_health.front_valid
                    and self._front_fresh(now))

    def _auto_approach_ranges_ready(self, now):
        return self._front_flip_input_ready(now)

    def _bench_ranges_ready(self, now):
        if self._approach_stage():
            return self._front_flip_input_ready(now)
        return bool(self._sensor_health_fresh(now) and self.sensor_health.front_valid
                    and self.sensor_health.ceiling_valid and self._front_fresh(now)
                    and self._top_fresh(now))

    def _vertical_speed_safe(self):
        if self._range_locked():
            now = self._range_feedback_now().to_sec()
            s = self.detach_range.read_snapshot(now)
            velocity, velocity_valid = self._range_control_velocity(s, now)
            # Single-frame/short-loss support has no invented velocity. Stability
            # still requires valid range velocity through _range_height_settled;
            # the separate fast-loss deadline bounds support without a speed.
            return bool((s.valid and (not velocity_valid or velocity >= -self.max_down_velocity_mps))
                        or (self.detach_range.invalid_since is not None
                            and now-self.detach_range.invalid_since < self.detach_range.config.loss_support_s))
        if self._range_task_active():
            # No measured vertical speed before range: keep attitude/authority
            # checks and rotation timeout, never use fused vz as a substitute.
            return True
        if self.odom is None:
            return False
        velocity_z = vertical_speed_enu_mps(
            self.odom,
            getattr(self, "odom_twist_in_child_frame", True),
        )
        if getattr(self, "bench_wait_for_ranges", False) and self.state in (
            WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER,
            WallPerchStatus.STATE_ABORT, WallPerchStatus.STATE_EXIT,
        ):
            return math.isfinite(velocity_z)
        if getattr(self, "ignore_down_velocity_during_hold", False) and self.state in (
            WallPerchStatus.STATE_WALL_HOLD, WallPerchStatus.STATE_WALL_PIN,
        ):
            return math.isfinite(velocity_z)
        return math.isfinite(velocity_z) and velocity_z >= -self.max_down_velocity_mps

    def _height_feedback(self):
        if self._range_locked():
            return self._range_height_feedback()
        if self._range_task_active():
            return None  # Support-only, not a local-height fallback.
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
        if getattr(self, "bench_wait_for_ranges", False):
            base = getattr(self, "bench_detach_hover_thrust", self.hover_thrust)
            release = getattr(self, "bench_detach_release_thrust", self.hover_thrust)
            return release + self._smoothstep5(clamp(vertical_fraction, 0.0, 1.0)) * (base - release)
        vertical_demand = (
            self.hover_thrust
            + self.detach_altitude_kp * (self.detach_altitude_m - altitude)
            - self.detach_vertical_speed_kd * speed
        )
        if vertical_demand <= 0.0:
            return 0.1
        return clamp(vertical_demand / max(vertical_fraction, 1.0e-3), 0.1, self.max_thrust)

    def _prepare_vertical_reference(self, now):
        """Legacy recovery reference; normal detach retains its departure height."""
        if getattr(self, "detach_vertical_active", False):
            return
        if not getattr(self, "recovery_wait_for_level", False) or not self._odom_fresh(now):
            return
        feedback, attitude = self._height_feedback(), self._current_quaternion()
        if feedback is None or attitude is None:
            return
        tilt = math.acos(clamp(rotate_vector(attitude, (0., 0., 1.))[2], -1., 1.))
        if tilt <= math.radians(5.):
            self.recovery_altitude_m = feedback[0]
            self.recovery_wait_for_level = False
            self.vertical_reference_source = "first_level_5deg"
            rospy.loginfo("wall vertical: first actual level height %.3f m", feedback[0])

    def _vertical_height_thrust(self, reference, base_override=None):
        """Height -> bounded ENU vz -> collective, including descent braking."""
        feedback = self._height_feedback()
        attitude = self._current_quaternion()
        self.vertical_target_speed = self.vertical_correction = math.nan
        if feedback is None or attitude is None:
            return math.nan  # No fabricated feedback or fixed boosted fallback.
        altitude, speed = feedback
        # While returning level, brake vz without pulling back to the wall height.
        target_speed = (0.0 if reference is None else clamp(
            self.recovery_height_to_speed_kp * (reference - altitude),
            -self.recovery_max_down_speed_mps, self.recovery_max_up_speed_mps))
        correction = min(self.recovery_positive_compensation_max,
                         self.recovery_velocity_kp * (target_speed - speed))
        self.vertical_target_speed, self.vertical_correction = target_speed, correction
        base_thrust = self.hover_thrust
        if (getattr(self, "approach_use_takeover_thrust", False)
                and self.state == WallPerchStatus.STATE_SLOW_APPROACH):
            base_thrust = getattr(self, "approach_base_vertical_thrust", None)
            if base_thrust is None:
                return math.nan  # Do not substitute the old fixed hover value.
        if base_override is not None:
            base_thrust = base_override
        if not math.isfinite(base_thrust):
            return math.nan
        vertical_demand = max(0.0, base_thrust + correction)
        vertical_fraction = max(0.0, rotate_vector(attitude, (0., 0., 1.))[2])
        return clamp(vertical_demand / max(vertical_fraction, 1.e-3),
                     0.1, min(self.max_thrust, self.recovery_max_thrust))

    def _recovery_height_thrust(self):
        return self._vertical_height_thrust(getattr(self, "recovery_altitude_m", None))

    def _detach_height_settled(self):
        if self._range_locked():
            return self._range_height_settled(self._range_feedback_now())
        if self._range_task_active():
            return False  # Cannot complete/handover without measured range height.
        if getattr(self, "bench_wait_for_ranges", False):
            return self._height_feedback() is not None
        if getattr(self, 'state', None) != WallPerchStatus.STATE_ABORT and not self._detach_height_loop_ready():
            return False
        feedback = self._height_feedback()
        return bool(
            getattr(self, "recovery_altitude_m", None) is not None
            and feedback is not None
            and abs(self.recovery_altitude_m - feedback[0]) <= self.detach_altitude_tolerance_m
            and abs(feedback[1]) <= self.detach_vertical_speed_tolerance_mps
        )

    def _attitude_recovered(self):
        quaternion = self._current_quaternion()
        if quaternion is None:
            return False
        roll, pitch, _yaw = self._euler_from_quaternion(quaternion)
        return abs(roll) < self.recover_angle_rad and abs(pitch) < self.recover_angle_rad

    def _manual_wall_recovery_active(self, now):
        """Allow wheel correction after trajectory completion and pitch entry.

        Top measurements do not authorize or block this control-phase latch.
        This is a control-phase latch, not proof of physical wall contact.
        """
        if not getattr(self, "manual_wall_recovery_enabled", False):
            return False
        return bool(self.state == WallPerchStatus.STATE_WALL_HOLD or (
            self.state in (WallPerchStatus.STATE_FLIP_TO_WALL,
                           WallPerchStatus.STATE_WALL_CAPTURE)
            and self._hold_entry_ready(now)))

    def _manual_wall_recovery_reason(self, now):
        if not self._manual_wall_recovery_active(now) or self.state != WallPerchStatus.STATE_WALL_HOLD:
            return ""
        warnings = []
        if not self._rates_safe(self.capture_max_rate_rps):
            warnings.append("机体转速较大")
        vz = vertical_speed_enu_mps(self.odom, self.odom_twist_in_child_frame)
        if math.isfinite(vz) and vz < -self.max_down_velocity_mps:
            warnings.append("估计下降速度越限")
        measured = self._current_quaternion()
        if measured is not None and quaternion_angle(measured, self._hold_follow_quaternion()) > self.detach_axis_error_rad:
            warnings.append("姿态偏离贴墙方向")
        if self._top_clear_ready(now):
            warnings.append("顶部距离偏大")
        if not warnings:
            return ""
        return "贴墙调整：保持压墙推力，等待轮子回正后手动脱离（{}）".format("；".join(warnings))

    def _safety_ok(self, now):
        if not self._flight_ready(now) or not self._odom_fresh(now):
            return False
        if self._simple_departure() and (
                self.state == WallPerchStatus.STATE_DETACH_ROTATE
                or self._locked_recovery()
                or (self.state == WallPerchStatus.STATE_WALL_HOLD and self.detach_requested)):
            # The demo's command-to-level segment needs attitude/authority only.
            # Range velocity damps thrust; a single descent estimate must not
            # replace the fixed attitude trajectory with emergency leveling.
            return self._rates_safe(self.recover_max_rate_rps)
        if getattr(self, "auto_approach", False) and self._approach_stage():
            # Range loss does not cancel flight/odom/kill protection.
            return self._rates_safe() and self._vertical_speed_safe()
        if not self._top_preflight_ready(now):
            return False
        if self._front_required() and not self._front_fresh(now):
            return False
        if self._front_required() and self._sensor_health_fresh(now) and not self._range_valid_for_safety("front", now):
            return False
        if self._top_required() and not self._top_fresh(now):
            return False
        if self._manual_wall_recovery_active(now):
            # Wheel correction may involve descent, fast body rotation or a
            # large axis error. Keep bounded wall pressure instead of leveling.
            # Finite feedback, flight/odom freshness, kill and mode still gate it.
            return odometry_is_finite(self.odom)
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
        if self.flip_trigger_hold_s <= 0.0:
            # A single accepted raw sample is sufficient; filtering and the
            # shared two-sample hold helper must not delay this gate.
            sample = getattr(self, "front_range", None)
            return bool(fresh and self._front_flip_input_ready(now)
                        and self.flip_ready_since is not None
                        and self.flip_ready_last_sample_at is not None
                        and sample is not None and is_valid_range(sample)
                        # Range.range is float32: admit 0.20 m after wire rounding.
                        and sample.range <= self.flip_trigger_distance_m + 1.e-8)
        return self._held_samples(
            fresh,
            self.flip_ready_since,
            self.flip_ready_last_sample_at,
            self.flip_trigger_hold_s,
        )

    def _advance_flip_progress(self, now):
        """One capped elapsed-time budget for attitude and thrust per control tick.

        Discard excess time after a callback stall. Sensor callbacks only read
        this clock; they cannot accelerate the commanded trajectory.
        """
        previous = getattr(self, "flip_step_at", None)
        if previous is None:
            self.flip_step_at = now
            self.flip_step_dt = 0.0
            self.flip_elapsed_s = 0.0
            return
        if now == previous:
            return
        delta = (now - previous).to_sec()
        if delta < 0.0:
            self.flip_clock_reversed = True
            self.flip_step_dt = 0.0
            return
        self.flip_step_dt = min(delta, .05)
        self.flip_step_at = now
        elapsed = getattr(self, "flip_elapsed_s", 0.0)
        proposed = min(self.flip_time_s, elapsed + self.flip_step_dt)
        measured = self._current_quaternion() if self._odom_fresh(now) else None
        limit = getattr(self, "flip_max_tracking_error_rad", math.radians(15.))
        def error(seconds):
            target = self._quat_slerp(self.q_flip_start, self.q_wall,
                self._smoothstep5(seconds / self.flip_time_s))
            return quaternion_angle(target, measured)
        current_error = error(elapsed) if measured is not None else math.inf
        next_error = error(proposed) if measured is not None else math.inf
        if measured is None or (current_error > limit and next_error >= current_error):
            proposed = elapsed
        elif current_error <= limit and next_error > limit:
            # Bound only target lead. Vertical thrust still spends the whole
            # capped elapsed-time budget so tracking lag cannot freeze braking.
            low, high = elapsed, proposed
            for _ in range(20):
                middle = .5 * (low + high)
                if error(middle) <= limit:
                    low = middle
                else:
                    high = middle
            proposed = low
        self.flip_tracking_limited = proposed + 1.e-9 < min(self.flip_time_s, elapsed + self.flip_step_dt)
        self.flip_elapsed_s = proposed
        if (self._flip_progress() >= getattr(self, "flip_top_start_fraction", .90)
                and getattr(self, "top_hold_accept_after", None) is None):
            # The qualifying source frame must be generated after the actual
            # window opening, not after the old start+nominal-time estimate.
            self.top_hold_accept_after = now
            self.top_hold_frame_at = None

    def _flip_progress(self):
        return clamp(getattr(self, "flip_elapsed_s", 0.0) / self.flip_time_s, 0.0, 1.0)

    def _late_top_window_open(self, now):
        after = getattr(self, "top_hold_accept_after", None)
        return bool(after is not None and now >= after
                    and (self.state != WallPerchStatus.STATE_FLIP_TO_WALL
                         or self._flip_progress() >= getattr(self, "flip_top_start_fraction", .90)))

    def _late_top_frame_ready(self, now):
        """Legacy top-frame diagnostic, never a HOLD admission condition."""
        frame_at = getattr(self, "top_hold_frame_at", None)
        return bool(self._late_top_window_open(now) and frame_at is not None
                    and is_fresh(frame_at, self.sensor_timeout_s, now)
                    and self._top_fresh(now)
                    and (not getattr(self, "hold_entry_top_distance_check_enabled", False)
                         or self._top_contact_detected(now)))

    def _hold_entry_ready(self, now):
        if self.state not in (WallPerchStatus.STATE_FLIP_TO_WALL,
                              WallPerchStatus.STATE_WALL_CAPTURE):
            return False
        if (self.state == WallPerchStatus.STATE_FLIP_TO_WALL
                and self._flip_progress() < 1.0):
            return False
        if not self._odom_fresh(now):
            return False
        quaternion = self._current_quaternion()
        if quaternion is None:
            return False
        pitch = self._euler_from_quaternion(quaternion)[1]
        return bool(math.isfinite(pitch) and pitch + 1.e-9 >= getattr(
            self, "hold_entry_min_pitch_rad", math.radians(80.0)))

    def _top_contact_ready(self, now):
        fresh = self._top_fresh(now)
        if not fresh:
            self.top_contact_since = None
            self.top_contact_last_sample_at = None
        if (self.state == WallPerchStatus.STATE_WALL_CAPTURE
                and self.top_capture_near_samples < self.top_capture_window_samples):
            return False
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
        yaw = getattr(self, 'horizontal_takeover_yaw', None)
        if yaw is None or not math.isfinite(yaw):
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
            new_state in (WallPerchStatus.STATE_WALL_CAPTURE, WallPerchStatus.STATE_WALL_HOLD)
            and self.state == WallPerchStatus.STATE_FLIP_TO_WALL
        ):
            # Preserve the current commanded attitude at the phase boundary.
            progress = self._flip_progress()
            self.q_contact = self._quat_slerp(
                self.q_flip_start, self.q_wall, self._smoothstep5(progress)
            )
        if new_state == WallPerchStatus.STATE_FLIP_TO_WALL:
            self.q_flip_start = self._flip_entry_quaternion(now)
            if self.q_flip_start is None:
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "flip attitude source missing or stale")
                return
        if new_state == WallPerchStatus.STATE_FLIP_TO_WALL and getattr(self, "flip_vertical_support_enabled", False):
            if not self._prepare_flip_vertical_support(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "flip vertical support source missing or stale")
                return
        if new_state == WallPerchStatus.STATE_DETACH_ROTATE and not getattr(self, "bench_wait_for_ranges", False):
            if not self._simple_departure() and not self._detach_horizontal_reference_ready():
                self.last_reason = "离墙等待：水平闭环未启用或本次接管平衡姿态／航向未锁定"
                return
            output = self._valid_wall_output(now)
            if output is None or not self._begin_detach_vertical(now, output):
                self.last_reason = "离墙等待：" + (getattr(self, "detach_vertical_reason", "")
                    if output is not None else "最终姿态／推力输出无效或过期")
                return
            q = output.orientation
            self.q_detach_start = self._quat_normalize((q.x, q.y, q.z, q.w))
        if new_state in (WallPerchStatus.STATE_IDLE, WallPerchStatus.STATE_FRONT_WALL_DETECT):
            self.recovery_started_at = None
            self._reset_flip_vertical_support()
            self._reset_detach_vertical()
            if new_state == WallPerchStatus.STATE_FRONT_WALL_DETECT:
                self._reset_range_task()
            if hasattr(self, "_reset_recovery_horizontal"):
                self._reset_recovery_horizontal("新动作／释放：清零恢复控制", clear_departure=True)
        rospy.loginfo(
            "wall perch: %s -> %s (%s)",
            self.STATE_NAMES[self.state],
            self.STATE_NAMES[new_state],
            reason,
        )
        if new_state == WallPerchStatus.STATE_FRONT_WALL_DETECT:
            self.maneuver_id = uuid.uuid4().hex
            self.control_sequence = 0
            self.hold_achieved = False
            self.detach_completed = False
            self.fault_recovery_completed = False
            self.reached_state = new_state
        elif new_state not in (WallPerchStatus.STATE_IDLE, WallPerchStatus.STATE_ABORT, WallPerchStatus.STATE_EXIT):
            self.reached_state = max(getattr(self, "reached_state", 0), new_state)
        if new_state == WallPerchStatus.STATE_WALL_HOLD:
            self.hold_achieved = True
        if new_state == WallPerchStatus.STATE_EXIT and self.state == WallPerchStatus.STATE_RECOVER:
            self.detach_completed = not self._range_task_active()
            self.reached_state = new_state
        if new_state == WallPerchStatus.STATE_EXIT and self.state == WallPerchStatus.STATE_RECOVER:
            self.recovery_completed = not self._range_task_active()
            if self._range_task_active():
                self.fault_recovery_completed = False
                self.detach_task_phase = 'HANDOVER_READY'
                if self.detach_range_handover_started is None:
                    self.detach_range_handover_started = now.to_sec()
        elif new_state == WallPerchStatus.STATE_EXIT and self.state == WallPerchStatus.STATE_ABORT:
            self.recovery_completed = False
            self.fault_recovery_completed = True
        old_state = self.state
        self.state = new_state
        if getattr(self,"horizontal_velocity_enabled",False):
            self._clear_horizontal_integral("阶段切换：清零接近积分")
        if getattr(self,"horizontal_velocity_enabled",False) and new_state in (WallPerchStatus.STATE_IDLE,WallPerchStatus.STATE_FLIP_TO_WALL):
            self._reset_horizontal_gate("阶段切换：重置水平控制历史")
        self.state_entered_at = now
        self.last_reason = reason
        if new_state == WallPerchStatus.STATE_FLIP_TO_WALL:
            self.flip_elapsed_s = self.flip_step_dt = 0.0
            self.flip_step_at = now
            self.flip_clock_reversed = False
            self.flip_tracking_limited = False
            self.flip_thrust_evaluated_at = None
            self.top_hold_accept_after = now if getattr(self, "flip_top_start_fraction", .90) == 0. else None
            self.top_hold_frame_at = None
        elif new_state not in (WallPerchStatus.STATE_WALL_CAPTURE, WallPerchStatus.STATE_WALL_HOLD):
            self.top_hold_accept_after = None
            self.top_hold_frame_at = None
        if new_state in (WallPerchStatus.STATE_FLIP_TO_WALL,
                         WallPerchStatus.STATE_WALL_CAPTURE):
            self.top_capture_samples = []
            self.top_capture_near_samples = 0
            self.top_contact_since = None
            self.top_contact_last_sample_at = None
            self.top_clear_since = None
            self.top_clear_last_sample_at = None
            # Seed from existing valid data without refreshing its timestamp.
            # Capture confirmation below still requires new samples.
            if self._top_fresh(now) and self.top_range is not None:
                self.top_filtered_m = self.top_range.range
        if new_state == WallPerchStatus.STATE_FRONT_WALL_DETECT:
            self.recovery_completed = False
            self.detach_altitude_m = None
            self.recovery_altitude_m = None
            self.recovery_wait_for_level = False
            if getattr(self, "approach_altitude_m", None) is None and self._odom_fresh(now):
                feedback = self._height_feedback()
                self.approach_altitude_m = feedback[0] if feedback is not None else None
            self.vertical_reference_source = "offboard_takeover"
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
            self.detach_stable_last_stamp = None
            self.detach_stable_samples = 0
        elif new_state == WallPerchStatus.STATE_RECOVER:
            if self._simple_departure() and old_state == WallPerchStatus.STATE_DETACH_ROTATE:
                self.recover_entry_attitude = self._current_quaternion()
                self.recover_attitude_locked = True
                self.recover_stable = False
                self._begin_departure_clearance(now)
                # q_hover was captured near level before attachment. Never
                # recapture yaw from the disturbed pose at recovery entry.
                if self.departure_yaw is None:
                    self.departure_yaw = euler_from_quaternion(self.q_hover)[2]
                self._begin_simple_recovery(now)
                self.detach_level_stable_at = now.to_sec()
                rospy.loginfo('wall departure demo: LEVEL_STABLE request=%.9f rotation=%.9f stable=%.9f',
                              getattr(self, 'detach_requested_at', now).to_sec(), self.detach_range.started,
                              self.detach_level_stable_at)
            if self._range_locked() and not self.detach_range_faulted:
                self.detach_task_phase = 'RANGE_HEIGHT_CONTROL'
            if getattr(self, "recovery_started_at", None) is None:
                self.recovery_started_at = now
            self.detach_completed = self.recovery_completed = False
            self.top_clear_since = None
            self.top_clear_last_sample_at = None
        elif new_state == WallPerchStatus.STATE_DETACH_ROTATE:
            self._reset_detach_braking()
            self.recovery_started_at = None
            self.detach_requested = False
            if getattr(self, "bench_wait_for_ranges", False):
                self.recovery_altitude_m = None
                self.recovery_wait_for_level = True
                self.vertical_reference_source = "waiting_first_level_5deg"
                self.q_detach_start = self._current_quaternion() or self.q_contact
                feedback = self._height_feedback()
                self.detach_altitude_m = feedback[0] if feedback is not None else None
                self.bench_detach_thrust_value = self._wall_pressure_thrust(now)
                self.bench_detach_thrust_at = now
            else:
                self.detach_elapsed_s = self.detach_step_dt = 0.
                self.detach_step_at = now
                self.detach_clock_reversed = self.detach_tracking_limited = False
                self.detach_recovered_since = self.detach_recovered_last_stamp = None
                self.detach_recovered_samples = 0
                self.detach_level_stable_at = None
                if not self._simple_departure():
                    self._begin_departure_clearance(now)
        elif new_state == WallPerchStatus.STATE_ABORT:
            if self._range_task_active():
                self.detach_range_failure_cause = self.detach_range_failure_cause or reason
                self.detach_range_faulted = True
                self.detach_task_phase = 'FAULT'
                self.detach_range_block_reason = reason
            self.recovery_completed = False
            if hasattr(self, "_reset_recovery_horizontal"):
                self._reset_recovery_horizontal("异常恢复：取消主动退墙", clear_departure=True)
            if old_state in (WallPerchStatus.STATE_FRONT_WALL_DETECT,
                             WallPerchStatus.STATE_STABILIZE_HOVER,
                             WallPerchStatus.STATE_SLOW_APPROACH):
                self.recovery_altitude_m = getattr(self, "approach_altitude_m", None)
                self.recovery_wait_for_level = self.recovery_altitude_m is None
                self.vertical_reference_source = ("offboard_takeover" if self.recovery_altitude_m is not None
                                                  else "waiting_first_level_5deg")
            elif (not getattr(self, "detach_vertical_active", False)
                  and old_state not in (WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER)):
                self.recovery_altitude_m = None
                self.recovery_wait_for_level = True
                self.vertical_reference_source = "waiting_first_level_5deg"
            self._prepare_vertical_reference(now)
            self.cancel_requested = False
            self.detach_requested = False
        elif new_state == WallPerchStatus.STATE_EXIT:
            if self._range_task_active():
                self.detach_range_exit_entered_at = now.to_sec()
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
            self.approach_takeover_thrust = self.approach_base_vertical_thrust = None
            self.detach_altitude_m = self.approach_altitude_m = self.recovery_altitude_m = None
            self.recovery_wait_for_level = False
            self.vertical_reference_source = "none"
            if getattr(self,"horizontal_velocity_enabled",False):
                self.horizontal_equilibrium.clear()
                self.horizontal_takeover_yaw=None

    def _pin_triggered(self):
        if not self.pin_enabled:
            return False
        quaternion = self._current_quaternion()
        if quaternion is None:
            return False
        return abs(self._euler_from_quaternion(quaternion)[1]) >= self.pin_pitch_rad

    def _update_state_machine(self, now):
        # Landing is a terminal branch. Never wait for range/flow/settling and
        # never run the normal ABORT -> EXIT -> POSCTL path for this request.
        if getattr(self, 'abort_land_requested', False):
            flight = self.flight_state
            if (flight is not None and is_fresh(self.flight_state_received_at, self.state_timeout_s, now)
                    and is_fresh_source(flight.header.stamp, self.state_timeout_s, now)):
                if not flight.armed:
                    self.abort_land_requested = False
                    self.rearm_required = True
                    self._enter_state(WallPerchStatus.STATE_IDLE, now, '降落分支解除武装，释放ROS控制')
                elif flight.mode.upper() == 'AUTO.LAND':
                    self.last_reason = 'PX4 AUTO.LAND反馈已确认，停止ROS姿态推力输出'
                elif flight.mode.upper() != 'OFFBOARD':
                    self.last_reason = '人工/外部模式接管，停止ROS姿态推力输出'
            return
        if self._range_task_active() and self.state != WallPerchStatus.STATE_EXIT:
            flight = self.flight_state
            if (flight is not None and flight.connected and flight.mode.upper() != 'OFFBOARD'
                    and is_fresh(self.flight_state_received_at, self.state_timeout_s, now)):
                self.detach_range_faulted = True
                self.detach_task_phase = 'FAULT'
                self.detach_range_block_reason = '人工/外部模式接管，本轮未正常完成'
                self.detach_completed = self.recovery_completed = False
                self._enter_state(WallPerchStatus.STATE_IDLE, now, '人工/外部模式接管，本轮未正常完成')
                return
        self._prepare_detach_vertical_feedback(now)
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

        if not mode_requested:
            self.approach_far_sample = None
        if mode_requested and self._approach_stage() and not getattr(self, "rearm_required", False):
            far_distance = self._approach_far_distance(now)
            if far_distance is not None:
                if self.state == WallPerchStatus.STATE_IDLE:
                    self._capture_attitude_references()
                self.fault_count += 1
                self.approach_far_sample = None
                self._enter_state(WallPerchStatus.STATE_ABORT, now,
                    "approach front distance {:.3f} m exceeds {:.3f} m".format(
                        far_distance, getattr(self, "approach_abort_front_distance_m", 1.8)))
                return

        if self.state == WallPerchStatus.STATE_IDLE:
            if not mode_requested:
                self.rearm_required = False
                self.detach_requested = False
                self.cancel_requested = False
            altitude_ok = self._entry_height_ready()
            safety_ok = self._safety_ok(now)
            if mode_requested and not self._wall_fusion_ready(now):
                self.last_reason = "等待光流/对地测距参数就绪：" + self.wall_fusion_reason
                return
            if (getattr(self, "auto_approach", False) and mode_requested
                    and not self._auto_takeover_confirmed(now)):
                self.last_reason = "等待本次 OFFBOARD 接管确认"
                return
            if (getattr(self, "auto_approach", False) and mode_requested and not self.rearm_required
                    and altitude_ok and safety_ok):
                self._enter_state(WallPerchStatus.STATE_FRONT_WALL_DETECT, now, "automatic wall takeover confirmed")
                self._enter_state(WallPerchStatus.STATE_SLOW_APPROACH, now, "自动接近开始；总时限20秒")
            elif (mode_requested and not self.rearm_required and altitude_ok and safety_ok
                    and self._ignore_attachment_range_loss()
                    and (not self._bench_ranges_ready(now)
                         or (getattr(self, "manual_approach", False) and not self._flip_ready(now)))):
                self.last_reason = "接管等待：需要前向雷达新鲜有效距离"
                self.flip_ready_since = self.flip_ready_last_sample_at = None
            elif mode_requested and not self.rearm_required and altitude_ok and safety_ok:
                self._enter_state(WallPerchStatus.STATE_FRONT_WALL_DETECT, now, "wall mode armed")
                if getattr(self, "manual_approach", False):
                    ready = self._flip_ready(now) and self._front_flip_input_ready(now)
                    self._enter_state(WallPerchStatus.STATE_FLIP_TO_WALL if ready else WallPerchStatus.STATE_ABORT,
                                      now, "manual approach complete; begin flip" if ready else "manual flip gate lost during takeover")
            elif mode_requested:
                if self.rearm_required:
                    self.last_reason = "waiting for operator release before rearm"
                elif not altitude_ok:
                    altitude = self._entry_height()
                    self.last_reason = ("takeoff ground reference unavailable; restart while disarmed"
                        if altitude is None else
                        "minimum altitude gate blocked: height={:.3f} m <= {:.3f} m".format(
                            altitude, self.min_altitude_m))
                elif not safety_ok:
                    self.last_reason = "waiting for fresh wall safety inputs"

        elif self.state == WallPerchStatus.STATE_FRONT_WALL_DETECT:
            if not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "front detect safety failure")
            elif self._front_ready(now) and (not (self._bench_waiting() or self._ignore_attachment_range_loss()) or self._bench_ranges_ready(now)):
                self._enter_state(
                    WallPerchStatus.STATE_SLOW_APPROACH,
                    now,
                    "front wall ready; begin 3 deg approach",
                )
            elif self._bench_waiting() or self._ignore_attachment_range_loss():
                self.last_reason = "接近等待：需要两路新鲜有效距离及墙面距离条件"

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
            if getattr(self, "auto_approach", False):
                if not self._safety_ok(now):
                    self.fault_count += 1
                    self._enter_state(WallPerchStatus.STATE_ABORT, now, "automatic approach flight or odometry failure")
                elif self._state_age(now) >= self.approach_timeout_s:
                    self.fault_count += 1
                    self._enter_state(WallPerchStatus.STATE_ABORT, now, "自动接近超时：总时限20秒")
                elif not self._auto_approach_ranges_ready(now):
                    self.flip_ready_since = self.flip_ready_last_sample_at = None
                    self.last_reason = ("前向雷达告警：无效或过期，继续自动接近；等待有效前距确认翻转"
                                        if self._approach_motion_allowed(now) else
                                        "自动接近暂停：雷达无效或过期，自主目标返回水平；总时限20秒")
                elif getattr(self,"horizontal_velocity_enabled",False) and not self.horizontal_velocity_ready:
                    self.flip_ready_since = self.flip_ready_last_sample_at = None
                    self.last_reason = "自动接近暂停：" + self.horizontal_velocity_reason
                elif self._flip_ready(now):
                    self._enter_state(WallPerchStatus.STATE_FLIP_TO_WALL, now, "flip distance reached")
                else:
                    flip_gate = ("前距≤%.2f m单帧原始有效数据" % self.flip_trigger_distance_m
                                 if self.flip_trigger_hold_s <= 0.0 else
                                 "前距≤%.2f m连续%.2f s" % (self.flip_trigger_distance_m, self.flip_trigger_hold_s))
                    self.last_reason = (("自动接近：目标前向%.2f m/s、横向0；等待" % self.horizontal_approach_speed
                        if getattr(self,"horizontal_velocity_enabled",False) else "自动接近：前倾3°；等待") + flip_gate)
            elif not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "approach safety failure")
            elif (self._bench_waiting() or self._ignore_attachment_range_loss()) and not self._bench_ranges_ready(now):
                self.last_reason = "接近暂停：等待前向雷达新鲜有效距离，不凭无效距离触发翻转"
                self.state_entered_at = now
                self.flip_ready_since = self.flip_ready_last_sample_at = None
            elif self._state_age(now) > self.approach_timeout_s:
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "approach timeout")
            elif self._flip_ready(now):
                self._enter_state(WallPerchStatus.STATE_FLIP_TO_WALL, now, "flip distance reached")

        elif self.state == WallPerchStatus.STATE_FLIP_TO_WALL:
            self._advance_flip_progress(now)
            if getattr(self, "flip_clock_reversed", False):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "flip clock moved backwards")
            elif self._state_age(now) >= getattr(self, "flip_duration_timeout_s", max(6., 2.*self.flip_time_s)):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "flip wall-clock duration timeout")
            elif not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "flip sensor or flight failure")
            elif self._hold_entry_ready(now):
                self._enter_state(WallPerchStatus.STATE_WALL_HOLD, now, "flip complete; measured pitch ready")
            elif self._pin_triggered():
                self._enter_state(WallPerchStatus.STATE_WALL_PIN, now, "pin pitch reached")
            elif self._flip_progress() >= 1.0:
                self._enter_state(WallPerchStatus.STATE_WALL_CAPTURE, now, "flip complete; waiting for measured pitch")

        elif self.state == WallPerchStatus.STATE_WALL_CAPTURE:
            if not self._safety_ok(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "capture safety failure")
            elif self._state_age(now) >= self.capture_timeout_s:
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "wall capture timeout")
            elif self._pin_triggered():
                self._enter_state(WallPerchStatus.STATE_WALL_PIN, now, "pin pitch reached")
            elif self._hold_entry_ready(now):
                self._enter_state(WallPerchStatus.STATE_WALL_HOLD, now, "measured pitch ready after flip")

        elif self.state == WallPerchStatus.STATE_WALL_HOLD:
            contact_lost = self._top_clear_ready(now)
            manual_recovery = self._manual_wall_recovery_active(now)
            if not self._safety_ok(now) or (contact_lost and not manual_recovery
                    and not (self._simple_departure() and self.detach_requested)):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "wall contact or safety lost")
            elif self._pin_triggered():
                self._enter_state(WallPerchStatus.STATE_WALL_PIN, now, "pin pitch reached")
            elif self.detach_requested or (
                not manual_recovery and self.hold_time_s > 0.0
                and self._state_age(now) >= self.hold_time_s
            ):
                requested_at = getattr(self, "detach_requested_at", now)
                if (self.detach_requested and (now - requested_at).to_sec() >=
                        getattr(self, "detach_request_timeout_s", 8.)):
                    self.detach_requested = False
                    if getattr(self, 'detach_range_enabled', False):
                        self.detach_task_phase = 'FAULT'
                        self.detach_range_block_reason = '离墙请求准入超时'
                    self.last_reason = "离墙等待超时，请求已取消；" + self._detach_block_reason(now)
                elif self._detach_start_ready(now):
                    self._enter_state(WallPerchStatus.STATE_DETACH_ROTATE, now, "detach requested")
                elif self.detach_requested:
                    self.last_reason = "离墙请求已接收；" + self._detach_block_reason(now)

        elif self.state == WallPerchStatus.STATE_WALL_PIN:
            if not self._flight_ready(now):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "flight state lost while pinned")
            elif self.pin_hold_s > 0.0 and self._state_age(now) >= self.pin_hold_s:
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "pin hold timeout")

        elif self.state == WallPerchStatus.STATE_DETACH_ROTATE:
            bench = getattr(self, "bench_wait_for_ranges", False)
            if not bench:
                self._advance_detach_progress(now)
            if (self._simple_departure() and self._detach_progress() >= 1.
                    and self._odom_fresh(now)):
                pitch = math.degrees(euler_from_quaternion(self._current_quaternion())[1])
                self.detach_final_pitch_deg = pitch
                if pitch < 0.:
                    self.abort_land_requested = True
                    self._enter_state(WallPerchStatus.STATE_ABORT, now,
                                      'detach最终俯仰角%.3f°为负，请求PX4 AUTO.LAND' % pitch)
                    return
            if not bench and getattr(self, "detach_brake_fault", ""):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, self.detach_brake_fault)
            elif getattr(self, "detach_vertical_fault", ""):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, self.detach_vertical_fault)
            elif not self._safety_ok(now) or (not self._range_task_active() and self._height_feedback() is None):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "detach safety input lost")
            elif not bench and (self.detach_clock_reversed or self._state_age(now) >= self.detach_duration_timeout_s):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "detach clock reversed" if self.detach_clock_reversed else "detach rotation timeout")
            elif self._simple_departure() and self._detach_progress() >= 1.:
                pitch = math.degrees(euler_from_quaternion(self._current_quaternion())[1])
                self.detach_final_pitch_deg = pitch
                tilt = math.acos(clamp(rotate_vector(self._current_quaternion(), (0., 0., 1.))[2], -1., 1.))
                # The 0..5 degree braking objective and the strict <10 degree
                # RECOVER entry cone are separate contracts. Exclude equality
                # despite quaternion roundoff; negative pitch keeps the land branch.
                if 0. <= pitch and tilt < math.radians(ENTRY_TILT_DEG) - 1.e-12:
                    self._enter_state(WallPerchStatus.STATE_RECOVER, now,
                                      'detach最终俯仰角%.3f°，实测倾角%.3f°，平滑恢复悬停基准并等待姿态稳定接管（无自动后退）' %
                                      (pitch, math.degrees(tilt)))
                else:
                    self.last_reason = 'detach轨迹结束，等待实测倾角<%.0f°；当前%.3f°' % (ENTRY_TILT_DEG, math.degrees(tilt))
            elif ((self._attitude_recovered() and self._rates_safe(self.recover_max_rate_rps))
                  if bench else (not self._simple_departure() and self._detach_rotation_ready(now))):
                self._enter_state(WallPerchStatus.STATE_RECOVER, now, "detach measured and published attitude recovered")

        elif self.state == WallPerchStatus.STATE_RECOVER:
            if getattr(self, "detach_vertical_fault", ""):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, self.detach_vertical_fault)
                return
            if (not self._range_locked() and not getattr(self, "bench_wait_for_ranges", False)
                    and self._recovery_total_age(now) >= getattr(self, "recover_timeout_s", 15.)):
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, "recovery timeout; normal detach incomplete")
                return
            if not self._safety_ok(now) or (not self._range_task_active() and self._height_feedback() is None):
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
            if self._range_locked():
                if self._range_stability_ready(now):
                    reason = ('姿态稳定，准备交回POSCTL（无自动后退）' if self._locked_recovery()
                              else '测距闭环稳定，准备交回POSCTL')
                    self._enter_state(WallPerchStatus.STATE_EXIT, now, reason)
                return
            if (
                self._state_age(now) >= self.recover_time_s
                and stable
                and self._detach_height_settled()
                and self._horizontal_recovery_ready(now)
                and (getattr(self, "bench_wait_for_ranges", False) or self._departure_clearance_ready(now))
            ):
                self._enter_state(WallPerchStatus.STATE_EXIT, now, "recovery complete")

        elif self.state == WallPerchStatus.STATE_EXIT:
            if self._range_locked():
                flight = self.flight_state
                if (flight is not None and flight.connected
                        and flight.mode.upper() == 'POSCTL'
                        and is_fresh(self.flight_state_received_at, self.state_timeout_s, now)
                        and is_fresh_source(flight.header.stamp, self.state_timeout_s, now)
                        and flight.header.stamp.to_sec() > (self.detach_range_exit_entered_at or now.to_sec())
                        and self.flight_state_received_at.to_sec() > (self.detach_range_exit_entered_at or now.to_sec())):
                    self.detach_range_posctl_confirmed_at = self.flight_state_received_at.to_sec()
                    if not self.detach_range_faulted:
                        self.detach_task_phase = 'COMPLETED'
                        self.detach_completed = self.recovery_completed = True
                    self._enter_state(WallPerchStatus.STATE_IDLE, now, 'POSCTL新鲜反馈已确认，释放ROS控制')
                    return
                if self.detach_vertical_fault and not self.detach_range_faulted:
                    self.detach_range_faulted = True
                if self.detach_range_faulted:
                    self.detach_task_phase = 'FAULT'
                    if not self.fault_recovery_completed:
                        self._enter_state(WallPerchStatus.STATE_ABORT, now, self.detach_vertical_fault or '本轮测距控制异常')
                    return
                if flight is not None and flight.mode.upper() == 'OFFBOARD':
                    if self._locked_recovery():
                        self._range_stability_ready(now)
                    elif not self._range_stability_ready(now):
                        self._enter_state(WallPerchStatus.STATE_RECOVER, now, self.detach_range_block_reason)
                elif is_fresh(self.flight_state_received_at, self.state_timeout_s, now):
                    self.detach_range_faulted = True
                    self.detach_task_phase = 'FAULT'
                    self._enter_state(WallPerchStatus.STATE_IDLE, now, '人工/外部模式接管，本轮未正常完成')
                return
            # Keep a bounded level target until fresh FCU feedback confirms
            # OFFBOARD has ended. Owner release alone is not RC handover.
            normal_departure = not getattr(self, "fault_recovery_completed", False)
            if normal_departure and getattr(self, "detach_vertical_fault", ""):
                self.detach_completed = self.recovery_completed = False
                self.fault_count += 1
                self._enter_state(WallPerchStatus.STATE_ABORT, now, self.detach_vertical_fault)
                return
            if self._flight_ready(now):
                height_pending = not self._detach_height_loop_ready()
                if (normal_departure
                        and not getattr(self, "bench_wait_for_ranges", False)
                        and self.flight_state is not None and self.flight_state.mode.upper() == "OFFBOARD"
                        and (height_pending or not self._departure_clearance_ready(now))):
                    # EXIT is provisional until the FCU leaves OFFBOARD. A
                    # revoked height/distance gate resumes existing controllers
                    # without resetting their height reference or time budget.
                    self.detach_completed = self.recovery_completed = False
                    lost_gate = "height feedback continuity lost" if height_pending else "departure clearance lost"
                    if self._recovery_total_age(now) >= getattr(self, "recover_timeout_s", 15.):
                        self.fault_count += 1
                        self._enter_state(WallPerchStatus.STATE_ABORT, now,
                            "recovery timeout after " + lost_gate + "; normal detach incomplete")
                    else:
                        self._enter_state(WallPerchStatus.STATE_RECOVER, now,
                            lost_gate + " before pilot mode handover; resume recovery")
                    return
                self.last_reason = ("fault recovery complete; waiting for pilot mode handover"
                    if getattr(self, "fault_recovery_completed", False)
                    else "recovery complete; waiting for pilot mode handover")
            elif is_fresh(self.flight_state_received_at, self.state_timeout_s, now):
                self._enter_state(WallPerchStatus.STATE_IDLE, now, "control released")

        elif self.state == WallPerchStatus.STATE_ABORT:
            stable = (
                self._odom_fresh(now)
                and self._attitude_recovered()
                and self._rates_safe()
                and self._vertical_speed_safe()
                and self._detach_height_settled()
            )
            if (self._state_age(now) >= self.recover_time_s and stable
                    and self._horizontal_recovery_ready(now)
                    and (getattr(self, "bench_wait_for_ranges", False)
                         or self._published_recovery_target_ready(now))):
                self._enter_state(WallPerchStatus.STATE_EXIT, now, "fault recovery complete; normal detach incomplete")

    def _recovery_total_age(self, now):
        started = getattr(self, "recovery_started_at", None)
        return max(0., (now-started).to_sec()) if started is not None else self._state_age(now)

    def _valid_wall_output(self, now):
        """Return only the last fresh, actually published WALL attitude/thrust."""
        output = getattr(self, "horizontal_output", None)
        if (output is None or output.type_mask & (AttitudeTarget.IGNORE_ATTITUDE | AttitudeTarget.IGNORE_THRUST)
                or not is_fresh(getattr(self, "horizontal_output_at", None), .20, now)
                or not is_fresh_source(output.header.stamp, .20, now, future_tolerance_s=0.)
                or getattr(self, "horizontal_output_owner", None) != AttachmentControlCandidate.SOURCE_WALL
                or not is_fresh(getattr(self, "horizontal_output_owner_at", None), .30, now)
                or not math.isfinite(output.thrust) or not .1 <= output.thrust <= self.max_thrust + 1.e-6):
            return None
        q = output.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values) <= 1.e-12:
            return None
        return output

    def _detach_progress(self):
        if getattr(self, "detach_brake_planner", None) is not None:
            return self._brake_progress()
        return clamp(getattr(self, "detach_elapsed_s", 0.) / self.detach_time_s, 0., 1.)

    def _advance_detach_progress(self, now):
        previous = getattr(self, "detach_step_at", None)
        if previous is None:
            self.detach_step_at = now
            self.detach_elapsed_s = self.detach_step_dt = 0.
            return
        if now == previous:
            return
        delta = (now - previous).to_sec()
        if delta < 0.:
            self.detach_clock_reversed = True
            self.detach_step_dt = 0.
            return
        self.detach_step_at = now
        self.detach_step_dt = min(delta, .05)
        # Both normal departure paths must use measured-rate braking before
        # advancing the nominal timed reference. Range-height control changes
        # thrust feedback, not whether attitude braking is executed.
        if self._advance_detach_braking(now, self.detach_step_dt):
            return
        elapsed = self.detach_elapsed_s
        proposed = min(self.detach_time_s, elapsed + self.detach_step_dt)
        measured = self._current_quaternion() if self._odom_fresh(now) else None
        limit = getattr(self, "detach_max_tracking_error_rad", math.radians(30.))
        def error(seconds):
            target = self._quat_slerp(self.q_detach_start, self.q_hover,
                                     self._smoothstep5(seconds / self.detach_time_s))
            return quaternion_angle(target, measured)
        current_error = error(elapsed) if measured is not None else math.inf
        next_error = error(proposed) if measured is not None else math.inf
        if measured is None or (current_error > limit and next_error >= current_error):
            proposed = elapsed
        elif current_error <= limit and next_error > limit:
            low, high = elapsed, proposed
            for _ in range(20):
                middle = .5 * (low + high)
                if error(middle) <= limit:
                    low = middle
                else:
                    high = middle
            proposed = low
        self.detach_tracking_limited = proposed + 1.e-9 < min(self.detach_time_s, elapsed + self.detach_step_dt)
        self.detach_elapsed_s = proposed

    def _detach_rotation_pose_valid(self):
        attitude = self._current_quaternion()
        if attitude is None or getattr(self, "odom", None) is None:
            return False
        tilt = math.acos(clamp(rotate_vector(attitude, (0., 0., 1.))[2], -1., 1.))
        rates = self.odom.twist.twist.angular
        omega = (rates.x, rates.y, rates.z)
        return bool(tilt <= getattr(self, "detach_recovered_tilt_rad", math.radians(5.))
                    and all(math.isfinite(v) for v in omega)
                    and math.sqrt(sum(v*v for v in omega)) <=
                    getattr(self, "detach_recovered_rate_rps", math.radians(10.)))

    def _detach_rotation_ready(self, now):
        if ((not self._simple_departure() and not self._detach_height_loop_ready())
                or self._detach_progress() < 1. or not self._odom_fresh(now)
                or not self._detach_rotation_pose_valid()
                or getattr(self, "detach_recovered_samples", 0) < 2
                or getattr(self, "detach_recovered_since", None) is None
                or getattr(self, "detach_recovered_last_stamp", None) is None
                or (self.detach_recovered_last_stamp - self.detach_recovered_since).to_sec()
                   < getattr(self, "detach_recovered_hold_s", .30)):
            return False
        return self._published_recovery_target_ready(now)

    def _published_recovery_target_ready(self, now):
        output = self._valid_wall_output(now)
        if output is None:
            return False
        q = output.orientation
        tilt = math.acos(clamp(rotate_vector((q.x, q.y, q.z, q.w), (0., 0., 1.))[2], -1., 1.))
        # RECOVER's five-degree cone must not force a discontinuous projection
        # of a previous command which is still outside that cone.
        return tilt <= getattr(self, "detach_recovered_tilt_rad", math.radians(5.)) + 1.e-9

    def _flip_entry_quaternion(self, now):
        output = getattr(self, "horizontal_output", None)
        if (output is not None
                and not output.type_mask & AttitudeTarget.IGNORE_ATTITUDE
                and is_fresh(getattr(self, "horizontal_output_at", None), .20, now)
                and is_fresh_source(output.header.stamp, .20, now, future_tolerance_s=0.)
                and getattr(self, "horizontal_output_owner", None) == AttachmentControlCandidate.SOURCE_WALL
                and is_fresh(getattr(self, "horizontal_output_owner_at", None), .30, now)):
            q = output.orientation
            values = (q.x, q.y, q.z, q.w)
            if (all(math.isfinite(v) for v in values)
                    and sum(v*v for v in values) > 1.e-12):
                return self._quat_normalize(values)
        # Manual/bench acquisition begins at FLIP and has no preceding WALL
        # publication. It may start from fresh measured attitude, never zero.
        if not getattr(self, "auto_approach", False) and self._odom_fresh(now):
            return self._current_quaternion()
        return None

    def _reset_flip_vertical_support(self):
        self.flip_entry_thrust = self.flip_base_vertical_thrust = None
        self.flip_altitude_m = None
        self.flip_support_weight = 0.0
        self.flip_support_thrust = self.flip_pressure_thrust = math.nan
        self.flip_thrust_value = self.flip_thrust_at = None

    def _prepare_flip_vertical_support(self, now):
        self._reset_flip_vertical_support()
        output = getattr(self, "horizontal_output", None)
        base = getattr(self, "approach_base_vertical_thrust", None)
        reference = getattr(self, "approach_altitude_m", None)
        if (self.state != WallPerchStatus.STATE_SLOW_APPROACH or not self._odom_fresh(now)
                or base is None or not math.isfinite(base) or base <= 0
                or reference is None or not math.isfinite(reference)
                or output is None or output.type_mask & AttitudeTarget.IGNORE_THRUST
                or not math.isfinite(output.thrust) or not .1 <= output.thrust <= self.max_thrust
                or not is_fresh(getattr(self, "horizontal_output_at", None), .20, now)
                or not is_fresh_source(output.header.stamp, .20, now, future_tolerance_s=0.)
                or getattr(self, "horizontal_output_owner", None) != AttachmentControlCandidate.SOURCE_WALL
                or not is_fresh(getattr(self, "horizontal_output_owner_at", None), .30, now)):
            return False
        # Keep feedback correction separate from the captured vertical feedforward.
        # Capturing the complete last command as base would add its correction twice.
        self.flip_entry_thrust = self.flip_thrust_value = float(output.thrust)
        self.flip_base_vertical_thrust = base
        self.flip_altitude_m = reference
        self.flip_thrust_at = now
        self.flip_support_weight = 1.0
        rospy.loginfo("wall flip support: entry %.3f, vertical base %.3f, height %.3f m",
                      self.flip_entry_thrust, base, reference)
        return True

    def _legacy_flip_thrust(self):
        if not getattr(self, "flip_thrust_follow_tilt", False):
            return self.hover_thrust * self.flip_thrust_multiplier
        attitude = self._current_quaternion()
        if attitude is None:
            return self.hover_thrust  # Invalid pose is rejected by the safety gate.
        # Use the measured thrust-axis tilt, not elapsed trajectory time or yaw.
        # A vehicle still level must not receive the full wall-facing thrust.
        up = rotate_vector(attitude, (0.0, 0.0, 1.0))
        angle = math.acos(clamp(up[2], -1.0, 1.0))
        blend = self._smoothstep5(clamp(angle / max(abs(self.wall_angle_rad), 1e-6), 0.0, 1.0))
        return self.hover_thrust * (1.0 + (self.flip_thrust_multiplier - 1.0) * blend)

    def _flip_thrust(self, now=None):
        legacy = self._legacy_flip_thrust()
        if not getattr(self, "flip_vertical_support_enabled", False):
            return legacy
        now = rospy.Time.now() if now is None else now
        self._advance_flip_progress(now)
        if getattr(self, "flip_clock_reversed", False):
            return math.nan
        if getattr(self, "flip_thrust_evaluated_at", None) == now:
            return self.flip_thrust_value
        attitude = self._current_quaternion()
        if (attitude is None or not self._odom_fresh(now)
                or getattr(self, "flip_base_vertical_thrust", None) is None
                or getattr(self, "flip_thrust_at", None) is None):
            return math.nan
        tilt = math.acos(clamp(rotate_vector(attitude, (0., 0., 1.))[2], -1., 1.))
        blend = self._smoothstep5(clamp((tilt-self.flip_support_fade_start_rad) /
            (self.flip_support_fade_end_rad-self.flip_support_fade_start_rad), 0., 1.))
        # Fade only out; noisy angle feedback must not reenable height pulling near contact.
        self.flip_support_weight = min(self.flip_support_weight, 1.-blend)
        self.flip_pressure_thrust = legacy
        self.flip_support_thrust = math.nan
        desired = legacy
        self.vertical_target_speed = self.vertical_correction = math.nan
        if self.flip_support_weight > 0.:
            support = self._vertical_height_thrust(self.flip_altitude_m, self.flip_base_vertical_thrust)
            if not math.isfinite(support):
                return math.nan
            self.flip_support_thrust = support
            desired = self.flip_support_weight*support + (1.-self.flip_support_weight)*legacy
        # The first flip target equals the last actually sent approach thrust.
        # Subsequent correction and pressure transitions share one collective slew bound.
        step = self.flip_thrust_slew_per_s*self.flip_step_dt
        result = clamp(desired, self.flip_thrust_value-step, self.flip_thrust_value+step)
        self.flip_thrust_value = clamp(result, .1, self.max_thrust)
        self.flip_thrust_at = now
        self.flip_thrust_evaluated_at = now
        return self.flip_thrust_value

    def _candidate(self, now):
        if getattr(self, 'abort_land_requested', False) and not self._flight_ready(now):
            return False, None, math.nan, 0.
        if (self.state == WallPerchStatus.STATE_SLOW_APPROACH
                and not getattr(self, "auto_approach", False)
                and not getattr(self, "ignore_range_loss_during_attachment", False)
                and self._bench_waiting()
                and not self._bench_ranges_ready(now)):
            return True, None, math.nan, 0.0
        neutral = self.state in (
            WallPerchStatus.STATE_FRONT_WALL_DETECT,
            WallPerchStatus.STATE_STABILIZE_HOVER,
        )
        quaternion = None
        thrust = math.nan
        progress = 0.0

        if self.state == WallPerchStatus.STATE_SLOW_APPROACH:
            angle = 0.0 if getattr(self,"horizontal_velocity_enabled",False) else self.approach_pitch_rad
            if getattr(self, "auto_approach", False) and not self._approach_motion_allowed(now):
                angle = 0.0
            pitch = (0.0, math.sin(angle * 0.5), 0.0, math.cos(angle * 0.5))
            quaternion = self._quat_normalize(self._quat_multiply(self.q_hover, pitch))
            thrust = self._vertical_height_thrust(getattr(self, "approach_altitude_m", None))
        elif self.state == WallPerchStatus.STATE_FLIP_TO_WALL:
            self._advance_flip_progress(now)
            progress = self._flip_progress()
            quaternion = self._quat_slerp(
                self.q_flip_start, self.q_wall, self._smoothstep5(progress)
            )
            thrust = self._flip_thrust(now)
        elif self.state == WallPerchStatus.STATE_WALL_CAPTURE:
            progress = clamp(self._state_age(now) / self.capture_time_s, 0.0, 1.0)
            quaternion = self.q_contact
            thrust = self._wall_pressure_thrust(now)
        elif self.state == WallPerchStatus.STATE_WALL_HOLD:
            progress = 1.0
            followed = self._hold_follow_quaternion()
            entry = self._smoothstep5(
                self._state_age(now) / self.hold_entry_blend_s
            )
            quaternion = self._quat_slerp(self.q_contact, followed, entry)
            thrust = self._wall_pressure_thrust(now)
        elif self.state == WallPerchStatus.STATE_WALL_PIN:
            quaternion = self.q_contact
            thrust = self.pin_thrust
        elif self.state == WallPerchStatus.STATE_DETACH_ROTATE and getattr(self, "bench_wait_for_ranges", False):
            trajectory_progress = clamp(self._state_age(now) / self.detach_time_s, 0.0, 1.0)
            quaternion = self._quat_slerp(
                self.q_detach_start, self.q_hover, self._smoothstep5(trajectory_progress)
            )
            thrust = self._height_hold_thrust()
            attitude = self._current_quaternion()
            if attitude is not None:
                start_tilt = math.acos(clamp(rotate_vector(self.q_detach_start, (0, 0, 1))[2], -1, 1))
                tilt = math.acos(clamp(rotate_vector(attitude, (0, 0, 1))[2], -1, 1))
                progress = clamp(1.0 - tilt / max(start_tilt, 1.0e-3), 0, 1)
        elif self.state == WallPerchStatus.STATE_DETACH_ROTATE:
            self._advance_detach_progress(now)
            progress = self._detach_progress()
            quaternion = self._detach_target_quaternion()
            thrust = self._detach_vertical_thrust(now, self.detach_step_dt)
        elif self.state in (WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_EXIT, WallPerchStatus.STATE_ABORT):
            quaternion = self.recover_entry_attitude if self._locked_recovery() else self.q_hover
            thrust = (self._detach_vertical_thrust(now, min(.05, max(0., self.control_dt)))
                      if getattr(self, "detach_vertical_active", False) else self._recovery_height_thrust())

        quaternion = self._horizontal_attitude(now, quaternion)
        if quaternion is not None and math.isfinite(thrust):
            if (getattr(self, "bench_wait_for_ranges", False) and getattr(self, "detach_altitude_m", None) is not None
                    and hasattr(self, "bench_detach_thrust_at") and math.isfinite(thrust)):
                delta = max(0.0, (now - self.bench_detach_thrust_at).to_sec()) * self.bench_detach_thrust_slew_per_s
                thrust = clamp(thrust, self.bench_detach_thrust_value - delta, self.bench_detach_thrust_value + delta)
                self.bench_detach_thrust_value, self.bench_detach_thrust_at = thrust, now
            thrust = clamp(thrust, 0.1, self.max_thrust)
            if (not getattr(self, "detach_vertical_active", False)
                    and (self.state in (WallPerchStatus.STATE_SLOW_APPROACH, WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_ABORT, WallPerchStatus.STATE_EXIT)
                         or (self.state == WallPerchStatus.STATE_DETACH_ROTATE and getattr(self, "recovery_altitude_m", None) is not None))):
                thrust = min(thrust, self.recovery_max_thrust)
        return neutral, quaternion, thrust, progress

    def _acquire_requested(self, now):
        return bool((self.state == WallPerchStatus.STATE_FRONT_WALL_DETECT
                     or (getattr(self, "auto_approach", False) and self.state == WallPerchStatus.STATE_SLOW_APPROACH)
                     or (getattr(self, "manual_approach", False) and self.state == WallPerchStatus.STATE_FLIP_TO_WALL))
                    and self._mode_requested(now) and self._safety_ok(now))

    def _build_status(self, now):
        status = WallPerchStatus()
        settings = getattr(self, 'incline_settings', None)
        if settings is not None:
            status.incline_ready = (not settings.error(self.max_thrust)
                and self.horizontal_velocity_enabled and self.auto_approach)
            status.incline_thrust_limit = settings.flip_thrust_max
        status.recover_attitude_locked = getattr(self, 'recover_attitude_locked', False)
        status.recover_stable = getattr(self, 'recover_stable', False)
        status.abort_land_requested = getattr(self, 'abort_land_requested', False)
        status.detach_final_pitch_deg = getattr(self, 'detach_final_pitch_deg', math.nan)
        for key, value in zip('xyzw', getattr(self, 'recover_entry_attitude', (0., 0., 0., 1.))):
            setattr(status.recover_entry_attitude, key, value)
        status.header.stamp = now
        status.state = self.state
        status.state_name = self.STATE_NAMES[self.state]
        status.enabled = self.state != WallPerchStatus.STATE_IDLE
        status.acquire_request = self._acquire_requested(now)
        status.keep_ownership = status.enabled
        status.front_range_fresh = self._front_fresh(now)
        status.top_range_fresh = self._top_fresh(now)
        status.sensor_health_fresh = self._sensor_health_fresh(now)
        status.top_preflight_ready = self._top_preflight_ready(now)
        status.odom_fresh = self._odom_fresh(now)
        height = self._entry_height()
        status.altitude_reference_valid = height is not None
        status.height_above_takeoff_m = height if height is not None else math.nan
        status.min_entry_height_m = self.min_altitude_m
        status.entry_height_ready = self._entry_height_ready()
        status.hold_achieved = getattr(self, "hold_achieved", False)
        status.reached_state_name = self.STATE_NAMES[getattr(self, "reached_state", self.state)]
        status.detach_completed = getattr(self, "detach_completed", False)
        status.recovery_completed = getattr(self, "recovery_completed", False)
        status.fault_recovery_completed = getattr(self, "fault_recovery_completed", False)
        status.detach_pending = bool(getattr(self, "detach_requested", False) and self.state == WallPerchStatus.STATE_WALL_HOLD)
        detach_start_ready = bool(self.state == WallPerchStatus.STATE_WALL_HOLD
                                  and self._detach_start_ready(now))
        status.detach_block_reason = self._detach_block_reason(now) if self.state == WallPerchStatus.STATE_WALL_HOLD and not detach_start_ready else ""
        status.detach_ready = bool(self.state == WallPerchStatus.STATE_WALL_HOLD
            and self._safety_ok(now) and detach_start_ready
            and (self._manual_wall_recovery_active(now) or not self._top_clear_ready(now)))
        status.flight_state_fresh = self._flight_ready(now)
        status.auto_approach_enabled = getattr(self, "auto_approach", False)
        status.approach_paused = bool(status.auto_approach_enabled and self.state == WallPerchStatus.STATE_SLOW_APPROACH
                                      and not self._approach_motion_allowed(now))
        status.front_ready = self._held_status(
            status.front_range_fresh
            and self.front_filtered_m <= self.front_ready_distance_m,
            self.front_ready_since,
            self.front_ready_last_sample_at,
            self.front_ready_hold_s,
        )
        status.flip_ready = self._flip_ready(now)
        status.top_contact_ready = (self._late_top_frame_ready(now)
            if self.state in (WallPerchStatus.STATE_FLIP_TO_WALL, WallPerchStatus.STATE_WALL_CAPTURE)
            else self._held_status(
                status.top_range_fresh and self.top_filtered_m <= self.top_contact_distance_m,
                self.top_contact_since, self.top_contact_last_sample_at, self.top_contact_hold_s,
            ))
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
        self._fill_horizontal_status(status)
        flip_support = bool(self.state == WallPerchStatus.STATE_FLIP_TO_WALL
                            and getattr(self, "flip_vertical_support_enabled", False))
        reference = (getattr(self, "flip_altitude_m", None) if flip_support else
                     getattr(self, "approach_altitude_m", None) if self.state == WallPerchStatus.STATE_SLOW_APPROACH
                     else getattr(self, "recovery_altitude_m", None))
        status.vertical_control_active = self.state in (WallPerchStatus.STATE_SLOW_APPROACH,
            WallPerchStatus.STATE_RECOVER, WallPerchStatus.STATE_ABORT, WallPerchStatus.STATE_EXIT) or (
            self.state == WallPerchStatus.STATE_DETACH_ROTATE and reference is not None)
        status.vertical_control_active = status.vertical_control_active or (flip_support and self.flip_support_weight > 0.)
        status.vertical_reference_valid = reference is not None
        status.vertical_reference_m = reference if reference is not None else math.nan
        status.vertical_reference_source = getattr(self, "vertical_reference_source", "none")
        status.vertical_target_enu_mps = getattr(self, "vertical_target_speed", math.nan) if status.vertical_control_active else math.nan
        status.vertical_thrust_correction = getattr(self, "vertical_correction", math.nan) if status.vertical_control_active else math.nan
        status.approach_takeover_thrust = (getattr(self, "approach_takeover_thrust", None)
                                           if getattr(self, "approach_takeover_thrust", None) is not None else math.nan)
        status.approach_base_vertical_thrust = (getattr(self, "approach_base_vertical_thrust", None)
                                               if getattr(self, "approach_base_vertical_thrust", None) is not None else math.nan)
        status.flip_vertical_support_enabled = flip_support
        status.flip_vertical_support_weight = self.flip_support_weight if flip_support else 0.
        status.flip_entry_thrust = self.flip_entry_thrust if flip_support and self.flip_entry_thrust is not None else math.nan
        status.flip_base_vertical_thrust = self.flip_base_vertical_thrust if flip_support and self.flip_base_vertical_thrust is not None else math.nan
        status.flip_support_thrust = self.flip_support_thrust if flip_support else math.nan
        status.flip_pressure_thrust = self.flip_pressure_thrust if flip_support else math.nan
        status.reason = self.last_reason
        if self.state == WallPerchStatus.STATE_DETACH_ROTATE and getattr(self, 'detach_brake_reason', ''):
            status.reason += '; ' + self.detach_brake_reason
        adjustment_reason = self._manual_wall_recovery_reason(now)
        if adjustment_reason:
            status.reason += "; " + adjustment_reason
        if self.state == WallPerchStatus.STATE_FLIP_TO_WALL and getattr(self, "flip_tracking_limited", False):
            status.reason += "; 翻转目标跟踪误差受限，减缓姿态轨迹"
        if flip_support:
            status.reason += ("; 翻转初段竖直闭环（继承接近基准）" if self.flip_support_weight >= 1. else
                              "; 竖直支撑平滑退出" if self.flip_support_weight > 0. else "; 已切换原贴墙推力")
        if status.horizontal_equilibrium_enabled and not status.horizontal_equilibrium_valid:
            status.reason += "; " + status.horizontal_equilibrium_reason
        if status.vertical_control_active:
            status.reason += ("; 高度闭环基准 %.3f m（%s）" % (reference, status.vertical_reference_source)
                              if reference is not None else "; 竖直速度制动：等待实际回平至5°锁定高度")
        if self.state == WallPerchStatus.STATE_SLOW_APPROACH and getattr(self, "approach_use_takeover_thrust", False):
            status.reason += ("; 接管推力 %.3f，竖直支撑基准 %.3f" % (
                self.approach_takeover_thrust, self.approach_base_vertical_thrust)
                if self.approach_base_vertical_thrust is not None else "; 接管推力缺失，禁止使用固定值替代")
        if status.horizontal_velocity_enabled and self.state in (WallPerchStatus.STATE_RECOVER,WallPerchStatus.STATE_ABORT,WallPerchStatus.STATE_EXIT):
            status.reason += "; " + status.horizontal_velocity_reason
        if self.state == WallPerchStatus.STATE_DETACH_ROTATE and getattr(self, "detach_tracking_limited", False):
            status.reason += "; 脱离轨迹跟踪误差受限"
        self._fill_detach_vertical_status(status)
        return status

    @staticmethod
    def _build_control_candidate(status, maneuver_id="", control_sequence=0):
        candidate = AttachmentControlCandidate()
        candidate.header = status.header
        candidate.phase_valid = bool(maneuver_id and control_sequence > 0)
        candidate.control_phase = status.state
        candidate.maneuver_id = maneuver_id
        candidate.control_sequence = control_sequence
        candidate.recover_attitude_locked = getattr(status, "recover_attitude_locked", False)
        candidate.incline_active = getattr(status, "incline_active", False)
        candidate.horizontal_velocity_enabled = status.horizontal_velocity_enabled
        candidate.horizontal_velocity_active = status.horizontal_velocity_active
        candidate.approach_paused = status.approach_paused
        candidate.recovery_support_valid = status.recovery_support_valid
        candidate.recovery_thrust_limit = status.recovery_thrust_limit
        candidate.detach_task_phase = status.detach_task_phase
        candidate.detach_range_episode = status.detach_range_episode
        candidate.range_height_control = bool(status.detach_range_height_enabled and status.detach_range_target_locked
                                              and status.detach_vertical_active)
        candidate.departure_support_only = bool(status.detach_range_height_enabled
                                                and status.detach_vertical_active
                                                and not status.detach_range_target_locked)
        candidate.range_feedback_valid = status.detach_range_valid
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
        # A timer tick sees one odometry message and its matching receipt time.
        # Queue size one drops superseded frames while this short section runs.
        with getattr(self, 'feedback_lock', nullcontext()):
            self._control_cycle(_event)

    def _control_cycle(self, _event):
        now = rospy.Time.now()
        self.detach_range_cycle_at = now
        try:
            self._control_cycle_at(now)
        finally:
            self.detach_range_cycle_at = None

    def _control_cycle_at(self, now):
        if self.previous_update_at is None:
            self.control_dt = 0.02
        else:
            self.control_dt = clamp(
                (now - self.previous_update_at).to_sec(), 0.001, 0.05
            )
        self.previous_update_at = now
        if getattr(self, 'incline_selected', False):
            self._incline_cycle(now)
            return
        self._prepare_vertical_reference(now)
        self._prepare_horizontal_velocity(now)
        self._prepare_horizontal_equilibrium(now)
        # Range fault budgets must be visible to the state machine in the same
        # cycle, rather than first changing while the candidate is built.
        if self._range_task_active():
            self._prepare_range_feedback(now)
        self._update_state_machine(now)
        status = self._build_status(now)
        self.status_publisher.publish(status)
        self._publish_departure_trace(now, status)
        self.control_sequence += 1
        self.control_candidate_publisher.publish(
            self._build_control_candidate(status, self.maneuver_id, self.control_sequence)
        )


if __name__ == "__main__":
    rospy.init_node("wall_perch_decision_node")
    from px4_wall_ceiling_control.adapters.async_logging import install_ros_async_logging
    install_ros_async_logging(rospy)
    WallPerchDecisionNode()
    rospy.spin()
