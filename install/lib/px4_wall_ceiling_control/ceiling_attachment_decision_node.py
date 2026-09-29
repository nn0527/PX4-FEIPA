#!/usr/bin/env python3
"""Ceiling attachment state machine adapted from the reference PX4 module.

The shared two-sensor range arbiter feeds this decision. MAVROS translation belongs downstream in
``attachment_output_mux_node.py`` in the unified runtime chain.
"""

import math

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from px4_wall_ceiling_control.adapters.odometry import vertical_speed_enu_mps
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion
from px4_wall_ceiling_control.domain.sample_hold import held_samples, record_hold_sample
from px4_wall_ceiling_control.freshness import clamp, is_fresh, is_valid_range
from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    AttachmentMechanismCommand,
    AttachmentMechanismStatus,
    CeilingAttachmentStatus,
    OperatorCommand,
)


class CeilingAttachmentDecisionNode:
    STATE_NAMES = {
        CeilingAttachmentStatus.STATE_NORMAL_FLIGHT: "NORMAL_FLIGHT",
        CeilingAttachmentStatus.STATE_CEILING_ARMED: "CEILING_ARMED",
        CeilingAttachmentStatus.STATE_APPROACH: "APPROACH",
        CeilingAttachmentStatus.STATE_ATTACH_CONTROL: "ATTACH_CONTROL",
        CeilingAttachmentStatus.STATE_SURFACE_HOLD: "SURFACE_HOLD",
        CeilingAttachmentStatus.STATE_DETACH: "DETACH",
        CeilingAttachmentStatus.STATE_RECOVERY_HOVER: "RECOVERY_HOVER",
        CeilingAttachmentStatus.STATE_FAULT: "FAULT",
    }

    def __init__(self):
        self.contact_distance_m = float(rospy.get_param("~contact_distance_m", 0.10))
        self.target_compression_m = float(
            rospy.get_param("~target_compression_m", 0.02)
        )
        self.activation_distance_m = float(
            rospy.get_param("~activation_distance_m", 0.50)
        )
        self.contact_margin_m = float(rospy.get_param("~contact_margin_m", 0.02))
        self.lost_contact_margin_m = float(
            rospy.get_param("~lost_contact_margin_m", 0.08)
        )
        self.filter_alpha = clamp(
            float(rospy.get_param("~range_filter_alpha", 0.20)), 0.0, 1.0
        )
        self.range_timeout_s = float(rospy.get_param("~range_timeout_s", 0.60))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.odom_twist_in_child_frame = bool(
            rospy.get_param("~odom_twist_in_child_frame", True)
        )
        self.operator_timeout_s = float(
            rospy.get_param("~operator_timeout_s", 0.50)
        )
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 0.50))
        self.require_armed = bool(rospy.get_param("~require_armed", True))
        self.require_offboard = bool(rospy.get_param("~require_offboard", True))
        self.vertical_speed_gate_mps = float(
            rospy.get_param("~vertical_speed_gate_mps", 0.20)
        )
        self.max_roll_rad = math.radians(
            float(rospy.get_param("~max_roll_deg", 15.0))
        )
        self.max_pitch_rad = math.radians(
            float(rospy.get_param("~max_pitch_deg", 15.0))
        )
        self.approach_settle_time_s = float(
            rospy.get_param("~approach_settle_time_s", 0.50)
        )
        self.approach_speed_mps = float(
            rospy.get_param("~approach_speed_mps", 0.15)
        )
        self.approach_timeout_s = float(
            rospy.get_param("~approach_timeout_s", 8.0)
        )
        self.attach_control_time_s = float(
            rospy.get_param("~attach_control_time_s", 0.50)
        )
        self.attach_control_timeout_s = max(
            self.attach_control_time_s,
            float(rospy.get_param("~attach_control_timeout_s", 5.0)),
        )
        self.attach_confirm_compression_m = float(
            rospy.get_param("~attach_confirm_compression_m", 0.015)
        )
        self.attach_confirm_time_s = float(
            rospy.get_param("~attach_confirm_time_s", 0.30)
        )
        self.require_attach_confirmation = bool(
            rospy.get_param("~require_attach_confirmation", True)
        )
        self.stable_tolerance_m = float(
            rospy.get_param("~stable_tolerance_m", 0.01)
        )
        self.stable_time_s = float(rospy.get_param("~stable_time_s", 0.50))
        self.recovery_hover_time_s = float(
            rospy.get_param("~recovery_hover_time_s", 2.0)
        )
        self.detach_timeout_s = float(rospy.get_param("~detach_timeout_s", 4.0))
        self.detach_safe_distance_m = max(
            self.contact_distance_m,
            float(rospy.get_param("~detach_safe_distance_m", 0.30)),
        )
        self.detach_clear_hold_s = float(
            rospy.get_param("~detach_clear_hold_s", 0.20)
        )
        self.hover_thrust = clamp(
            float(rospy.get_param("~hover_thrust", 0.50)), 0.1, 0.9
        )
        self.attach_thrust_multiplier = float(
            rospy.get_param("~attach_thrust_multiplier", 1.20)
        )
        self.surface_thrust_multiplier = float(
            rospy.get_param("~surface_thrust_multiplier", 1.15)
        )
        self.max_thrust = clamp(
            float(rospy.get_param("~max_thrust", 0.90)), 0.05, 0.95
        )
        self.attach_max_thrust = clamp(
            float(rospy.get_param("~attach_max_thrust", 0.55)),
            self.hover_thrust,
            self.max_thrust,
        )
        self.attach_thrust_ramp_per_s = max(
            0.0, float(rospy.get_param("~attach_thrust_ramp_per_s", 0.02))
        )
        self.detach_min_thrust = clamp(
            float(rospy.get_param("~detach_min_thrust", 0.30)),
            0.05,
            self.hover_thrust,
        )
        self.detach_distance_ramp_mps = float(
            rospy.get_param("~detach_distance_ramp_mps", 0.05)
        )
        self.distance_kp = float(rospy.get_param("~distance_kp", 0.8))
        self.distance_ki = float(rospy.get_param("~distance_ki", 0.0))
        self.distance_kd = float(rospy.get_param("~distance_kd", 0.02))
        self.integral_limit = abs(float(rospy.get_param("~integral_limit", 1.0)))
        self.require_mechanism_ack = bool(
            rospy.get_param("~require_mechanism_ack", True)
        )
        self.mechanism_timeout_s = float(
            rospy.get_param("~mechanism_timeout_s", 1.0)
        )

        self.state = CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
        self.state_entered_at = rospy.Time.now()
        self.range_message = None
        self.range_received_at = None
        self.distance_filtered_m = None
        self.odom = None
        self.odom_received_at = None
        self.flight_state = None
        self.flight_state_received_at = None
        self.operator = None
        self.operator_received_at = None
        self.detach_requested = False
        self.detach_failed = False
        self.detach_clear_since = None
        self.detach_clear_last_sample_at = None
        self.rearm_required = False
        self.q_hover = Quaternion(w=1.0)
        self.mechanism_sequence = 0
        self.mechanism_status = None
        self.mechanism_status_received_at = None
        self.attach_detected_at = None
        self.attach_last_sample_at = None
        self.stable_detected_at = None
        self.stable_last_sample_at = None
        self.target_distance_m = self._compressed_target_distance()
        self.distance_error_integral = 0.0
        self.previous_distance_error = 0.0
        self.previous_update_at = None
        self.attach_thrust_command = self._attach_base_thrust()
        self.fault_count = 0
        self.last_reason = "initialized"

        range_topic = rospy.get_param("~range_topic", "/sensor/ceiling/range")
        odom_topic = rospy.get_param(
            "~odom_topic", "/mavros/local_position/odom"
        )
        operator_topic = rospy.get_param("~operator_topic", "/operator/command")
        detach_topic = rospy.get_param("~detach_topic", "/ceiling_attachment/detach")
        status_topic = rospy.get_param("~status_topic", "/ceiling_attachment/status")
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))

        self.status_publisher = rospy.Publisher(
            status_topic, CeilingAttachmentStatus, queue_size=10
        )
        self.control_candidate_publisher = rospy.Publisher(
            rospy.get_param(
                "~control_candidate_topic",
                "/ceiling_attachment/control_candidate",
            ),
            AttachmentControlCandidate,
            queue_size=10,
        )
        self.mechanism_candidate_publisher = rospy.Publisher(
            rospy.get_param(
                "~mechanism_candidate_topic", "/attachment/mechanism_candidate"
            ),
            AttachmentMechanismCommand,
            queue_size=10,
        )
        rospy.Subscriber(range_topic, Range, self._on_range, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber(
            rospy.get_param("~state_topic", "/mavros/state"),
            State,
            self._on_state,
            queue_size=10,
        )
        rospy.Subscriber(
            operator_topic, OperatorCommand, self._on_operator, queue_size=10
        )
        rospy.Subscriber(detach_topic, Bool, self._on_detach, queue_size=10)
        rospy.Subscriber(
            rospy.get_param(
                "~mechanism_status_topic", "/attachment/mechanism_status"
            ),
            AttachmentMechanismStatus,
            self._on_mechanism_status,
            queue_size=10,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 2.0)), self._on_timer
        )

    def _compressed_target_distance(self):
        return max(0.0, self.contact_distance_m - self.target_compression_m)

    def _attach_base_thrust(self):
        return min(
            self.hover_thrust * self.attach_thrust_multiplier,
            self.attach_max_thrust,
            self.max_thrust,
        )

    def _surface_hold_thrust(self):
        return min(
            self.hover_thrust * self.surface_thrust_multiplier,
            self.attach_max_thrust,
            self.max_thrust,
        )

    def _attach_thrust(self, dt):
        """Increase ceiling pressure until the range reaches its stable band."""
        lower = min(self._attach_base_thrust(), self._surface_hold_thrust())
        upper = min(self.attach_max_thrust, self.max_thrust)
        thrust = clamp(self.attach_thrust_command, lower, upper)

        if self.distance_filtered_m is None:
            return thrust

        error = self.distance_filtered_m - self._compressed_target_distance()
        step = self.attach_thrust_ramp_per_s * max(0.0, dt)
        if error > self.stable_tolerance_m:
            # The measured compression is still insufficient. Increase only
            # while a fresh range sample remains outside the stable band.
            thrust = min(upper, thrust + step)
        elif error < -self.stable_tolerance_m:
            # Avoid continuing to load an already over-compressed mechanism.
            thrust = max(self._surface_hold_thrust(), thrust - step)

        self.attach_thrust_command = thrust
        return thrust

    def _on_range(self, message):
        if not is_valid_range(message):
            return
        now = rospy.Time.now()
        reset_filter = not is_fresh(
            self.range_received_at, self.range_timeout_s, now
        )
        self.range_message = message
        self.range_received_at = now
        if self.distance_filtered_m is None or reset_filter:
            self.distance_filtered_m = message.range
        else:
            self.distance_filtered_m += self.filter_alpha * (
                message.range - self.distance_filtered_m
            )
        compression = self.contact_distance_m - self.distance_filtered_m
        (
            self.attach_detected_at,
            self.attach_last_sample_at,
        ) = self._record_hold_sample(
            compression >= self.attach_confirm_compression_m,
            now,
            self.attach_detected_at,
        )
        (
            self.stable_detected_at,
            self.stable_last_sample_at,
        ) = self._record_hold_sample(
            abs(self.distance_filtered_m - self._compressed_target_distance())
            <= self.stable_tolerance_m,
            now,
            self.stable_detected_at,
        )
        (
            self.detach_clear_since,
            self.detach_clear_last_sample_at,
        ) = self._record_hold_sample(
            self.distance_filtered_m >= self.detach_safe_distance_m,
            now,
            self.detach_clear_since,
        )

    def _on_odom(self, message):
        self.odom = message
        self.odom_received_at = rospy.Time.now()

    def _on_state(self, message):
        self.flight_state = message
        self.flight_state_received_at = rospy.Time.now()

    def _on_operator(self, message):
        self.operator = message
        self.operator_received_at = rospy.Time.now()

    def _on_detach(self, message):
        if message.data:
            self.detach_requested = True

    def _on_mechanism_status(self, message):
        self.mechanism_status = message
        self.mechanism_status_received_at = rospy.Time.now()

    @staticmethod
    def _roll_pitch_from_quaternion(quaternion):
        roll, pitch, _yaw = euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )
        return roll, pitch

    def _state_age_s(self, now):
        return max(0.0, (now - self.state_entered_at).to_sec())

    def _mode_requested(self, now):
        return bool(
            is_fresh(self.operator_received_at, self.operator_timeout_s, now)
            and self.operator is not None
            and self.operator.mode == OperatorCommand.MODE_CEILING
            and self.operator.enable_control
        )

    def _arm_requested(self, now):
        return self._mode_requested(now)

    def _range_fresh(self, now):
        return bool(
            is_fresh(self.range_received_at, self.range_timeout_s, now)
            and self.distance_filtered_m is not None
        )

    def _odom_fresh(self, now):
        return is_fresh(self.odom_received_at, self.odom_timeout_s, now)

    def _flight_ready(self, now):
        if not is_fresh(
            self.flight_state_received_at, self.state_timeout_s, now
        ):
            return False
        if self.flight_state is None or not self.flight_state.connected:
            return False
        if self.require_armed and not self.flight_state.armed:
            return False
        if self.require_offboard and self.flight_state.mode.upper() != "OFFBOARD":
            return False
        return True

    def _capture_hover_attitude(self):
        if self.odom is None:
            return
        q = self.odom.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1.0e-6:
            return
        self.q_hover = Quaternion(
            x=q.x / norm,
            y=q.y / norm,
            z=q.z / norm,
            w=q.w / norm,
        )

    def _mechanism_release_confirmed(self, now):
        if not self.require_mechanism_ack:
            return True
        status = self.mechanism_status
        return bool(
            is_fresh(
                self.mechanism_status_received_at, self.mechanism_timeout_s, now
            )
            and status is not None
            and status.source == AttachmentMechanismCommand.SOURCE_CEILING
            and status.sequence == self.mechanism_sequence
            and status.acknowledged
            and status.released
            and not status.fault
        )

    def _enter_state(self, new_state, now, reason):
        if self.state == new_state:
            self.last_reason = reason
            return
        rospy.loginfo(
            "ceiling attachment: %s -> %s (%s)",
            self.STATE_NAMES[self.state],
            self.STATE_NAMES[new_state],
            reason,
        )
        self.state = new_state
        self.state_entered_at = now
        self.last_reason = reason
        self.attach_detected_at = None
        self.attach_last_sample_at = None
        self.stable_detected_at = None
        self.stable_last_sample_at = None
        self.detach_clear_since = None
        self.detach_clear_last_sample_at = None

        if new_state in (
            CeilingAttachmentStatus.STATE_CEILING_ARMED,
            CeilingAttachmentStatus.STATE_APPROACH,
        ):
            if new_state == CeilingAttachmentStatus.STATE_CEILING_ARMED:
                self.detach_failed = False
            self.target_distance_m = self._compressed_target_distance()
            self._reset_distance_pid()
        elif new_state == CeilingAttachmentStatus.STATE_DETACH:
            self.detach_requested = False
            self.target_distance_m = self._compressed_target_distance()
            self._reset_distance_pid()
            self.mechanism_sequence += 1
            self.detach_failed = False
        elif new_state == CeilingAttachmentStatus.STATE_ATTACH_CONTROL:
            self.attach_thrust_command = self._attach_base_thrust()
        elif new_state in (
            CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
            CeilingAttachmentStatus.STATE_FAULT,
        ):
            self.rearm_required = True

    def _reset_distance_pid(self):
        self.distance_error_integral = 0.0
        self.previous_distance_error = 0.0

    def _tilt_fault(self, now):
        if not self._odom_fresh(now) or self.odom is None:
            return False
        roll, pitch = self._roll_pitch_from_quaternion(self.odom.pose.pose.orientation)
        return abs(roll) > self.max_roll_rad or abs(pitch) > self.max_pitch_rad

    def _contact_lost(self):
        return bool(
            self.distance_filtered_m is not None
            and self.distance_filtered_m
            > self.contact_distance_m + self.lost_contact_margin_m
        )

    @staticmethod
    def _record_hold_sample(condition, now, since):
        return record_hold_sample(condition, now, since)

    @staticmethod
    def _held_samples(fresh, since, last_sample_at, duration):
        return held_samples(fresh, since, last_sample_at, duration)

    def _attach_confirmed(self, now):
        fresh = self._range_fresh(now)
        if not fresh:
            self.attach_detected_at = None
            self.attach_last_sample_at = None
        return self._held_samples(
            fresh,
            self.attach_detected_at,
            self.attach_last_sample_at,
            self.attach_confirm_time_s,
        )

    def _distance_stable(self, now):
        fresh = self._range_fresh(now)
        if not fresh:
            self.stable_detected_at = None
            self.stable_last_sample_at = None
        return self._held_samples(
            fresh,
            self.stable_detected_at,
            self.stable_last_sample_at,
            self.stable_time_s,
        )

    def _detach_clear(self, now):
        fresh = self._range_fresh(now)
        if not fresh:
            self.detach_clear_since = None
            self.detach_clear_last_sample_at = None
        return self._held_samples(
            fresh,
            self.detach_clear_since,
            self.detach_clear_last_sample_at,
            self.detach_clear_hold_s,
        )

    def _update_state_machine(self, now):
        mode_requested = self._mode_requested(now)
        arm_requested = self._arm_requested(now)
        range_fresh = self._range_fresh(now)
        odom_fresh = self._odom_fresh(now)
        flight_ready = self._flight_ready(now)
        attached_state = self.state in (
            CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
            CeilingAttachmentStatus.STATE_SURFACE_HOLD,
        )

        if attached_state and (self.detach_requested or not mode_requested):
            self._enter_state(
                CeilingAttachmentStatus.STATE_DETACH,
                now,
                "detach requested or ceiling mode released",
            )
        elif attached_state and (
            not range_fresh
            or not odom_fresh
            or self._tilt_fault(now)
            or self._contact_lost()
        ):
            self.fault_count += 1
            target = (
                CeilingAttachmentStatus.STATE_DETACH
                if flight_ready
                else CeilingAttachmentStatus.STATE_FAULT
            )
            self._enter_state(target, now, "attached input or contact safety lost")

        if self.state == CeilingAttachmentStatus.STATE_NORMAL_FLIGHT:
            if not mode_requested:
                self.rearm_required = False
                self.detach_requested = False
            elif (
                not self.rearm_required
                and flight_ready
                and range_fresh
                and odom_fresh
            ):
                self._capture_hover_attitude()
                self._enter_state(
                    CeilingAttachmentStatus.STATE_CEILING_ARMED,
                    now,
                    "ceiling mode armed",
                )

        elif self.state == CeilingAttachmentStatus.STATE_CEILING_ARMED:
            if not mode_requested:
                self._enter_state(
                    CeilingAttachmentStatus.STATE_NORMAL_FLIGHT,
                    now,
                    "ceiling mode released",
                )
            elif (
                not arm_requested
                or not flight_ready
                or not range_fresh
                or not odom_fresh
            ):
                self.fault_count += 1
                self._enter_state(
                    CeilingAttachmentStatus.STATE_FAULT,
                    now,
                    "armed input or flight state lost",
                )
            else:
                vertical_speed = abs(
                    vertical_speed_enu_mps(
                        self.odom,
                        getattr(self, "odom_twist_in_child_frame", True),
                    )
                )
                if (
                    self.distance_filtered_m < self.activation_distance_m
                    and vertical_speed < self.vertical_speed_gate_mps
                ):
                    self._enter_state(
                        CeilingAttachmentStatus.STATE_APPROACH,
                        now,
                        "ceiling acquired and vertical speed is low",
                    )

        elif self.state == CeilingAttachmentStatus.STATE_APPROACH:
            if not mode_requested:
                self._enter_state(
                    CeilingAttachmentStatus.STATE_FAULT,
                    now,
                    "ceiling mode released during approach",
                )
            elif (
                not arm_requested
                or not flight_ready
                or not range_fresh
                or not odom_fresh
            ):
                self.fault_count += 1
                self._enter_state(
                    CeilingAttachmentStatus.STATE_FAULT,
                    now,
                    "approach input timeout",
                )
            elif self._state_age_s(now) > self.approach_timeout_s:
                self.fault_count += 1
                self._enter_state(
                    CeilingAttachmentStatus.STATE_NORMAL_FLIGHT,
                    now,
                    "approach timeout",
                )
            elif (
                self._state_age_s(now) >= self.approach_settle_time_s
                and self.distance_filtered_m
                <= self.contact_distance_m + self.contact_margin_m
            ):
                self._enter_state(
                    CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
                    now,
                    "contact distance reached",
                )

        elif self.state == CeilingAttachmentStatus.STATE_ATTACH_CONTROL:
            confirmed = self._attach_confirmed(now)
            stable = self._distance_stable(now)
            time_ready = self._state_age_s(now) >= self.attach_control_time_s
            if (
                time_ready
                and stable
                and (confirmed or not self.require_attach_confirmation)
            ):
                self._enter_state(
                    CeilingAttachmentStatus.STATE_SURFACE_HOLD,
                    now,
                    "attachment confirmed and ceiling range stable",
                )
            elif self._state_age_s(now) >= self.attach_control_timeout_s:
                self.fault_count += 1
                self._enter_state(
                    CeilingAttachmentStatus.STATE_DETACH,
                    now,
                    "attachment failed to reach stable ceiling range",
                )

        elif self.state == CeilingAttachmentStatus.STATE_SURFACE_HOLD:
            self._attach_confirmed(now)
            self._distance_stable(now)

        elif self.state == CeilingAttachmentStatus.STATE_DETACH:
            if not flight_ready or not range_fresh or not odom_fresh:
                self.detach_failed = True
                self.fault_count += 1
                self._enter_state(
                    CeilingAttachmentStatus.STATE_FAULT,
                    now,
                    "detach input or flight state lost",
                )
            elif (
                self.require_mechanism_ack
                and not self._mechanism_release_confirmed(now)
                and self._state_age_s(now) >= self.mechanism_timeout_s
            ):
                self.detach_failed = True
                self.fault_count += 1
                self._enter_state(
                    CeilingAttachmentStatus.STATE_FAULT,
                    now,
                    "attachment mechanism release timeout",
                )
            elif self._mechanism_release_confirmed(now) and self._detach_clear(now):
                self._enter_state(
                    CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
                    now,
                    "safe separation confirmed",
                )
            elif self._state_age_s(now) >= self.detach_timeout_s:
                self.detach_failed = True
                self.fault_count += 1
                self._enter_state(
                    CeilingAttachmentStatus.STATE_FAULT,
                    now,
                    "detach failed: safe separation timeout",
                )

        elif self.state == CeilingAttachmentStatus.STATE_RECOVERY_HOVER:
            if self._state_age_s(now) >= self.recovery_hover_time_s:
                self._enter_state(
                    CeilingAttachmentStatus.STATE_NORMAL_FLIGHT,
                    now,
                    "recovery hover complete",
                )

        elif self.state == CeilingAttachmentStatus.STATE_FAULT:
            # Never resume a maneuver from a recovered input stream. The pilot
            # must first release the ceiling switch and take over/re-arm.
            if not mode_requested:
                self._enter_state(
                    CeilingAttachmentStatus.STATE_NORMAL_FLIGHT,
                    now,
                    "fault acknowledged by switch release",
                )

    def _detach_thrust(self, dt):
        self.target_distance_m = min(
            self.detach_safe_distance_m,
            self.target_distance_m + self.detach_distance_ramp_mps * dt,
        )
        error = self.target_distance_m - self.distance_filtered_m
        self.distance_error_integral = clamp(
            self.distance_error_integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        derivative = (
            (error - self.previous_distance_error) / dt if dt > 1.0e-4 else 0.0
        )
        self.previous_distance_error = error
        pid_output = (
            self.distance_kp * error
            + self.distance_ki * self.distance_error_integral
            + self.distance_kd * derivative
        )
        # Positive target error means that the vehicle is still too close to
        # the ceiling, so collective thrust must be reduced below hover.
        thrust = self.hover_thrust - pid_output
        return -clamp(thrust, self.detach_min_thrust, self.hover_thrust)

    def _publish_mechanism_candidate(self, now):
        command = AttachmentMechanismCommand()
        command.header.stamp = now
        command.source = AttachmentMechanismCommand.SOURCE_CEILING
        command.sequence = self.mechanism_sequence
        if self.state == CeilingAttachmentStatus.STATE_DETACH:
            command.action = AttachmentMechanismCommand.ACTION_RELEASE
            command.reason = "ceiling detach"
        elif self.state in (
            CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
            CeilingAttachmentStatus.STATE_SURFACE_HOLD,
        ):
            command.action = AttachmentMechanismCommand.ACTION_HOLD
            command.reason = "ceiling attachment hold"
        else:
            command.action = AttachmentMechanismCommand.ACTION_NONE
            command.reason = "no mechanism action"
        self.mechanism_candidate_publisher.publish(command)

    def _build_status(self, now, dt):
        status = CeilingAttachmentStatus()
        status.header.stamp = now
        status.state = self.state
        status.state_name = self.STATE_NAMES[self.state]
        # Keep ownership through controlled detach/recovery even if the
        # operator releases ceiling mode.  Releasing output immediately while
        # still attached would leave PX4 with the previous thrust setpoint.
        status.enabled = self.state in (
            CeilingAttachmentStatus.STATE_CEILING_ARMED,
            CeilingAttachmentStatus.STATE_APPROACH,
            CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
            CeilingAttachmentStatus.STATE_SURFACE_HOLD,
            CeilingAttachmentStatus.STATE_DETACH,
            CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
            CeilingAttachmentStatus.STATE_FAULT,
        )
        status.acquire_request = bool(
            self.state == CeilingAttachmentStatus.STATE_CEILING_ARMED
            and self._arm_requested(now)
            and self._flight_ready(now)
        )
        status.keep_ownership = status.enabled
        status.range_fresh = self._range_fresh(now)
        status.odom_fresh = self._odom_fresh(now)
        status.flight_state_fresh = self._flight_ready(now)
        status.ceiling_distance_raw_m = (
            self.range_message.range if self.range_message is not None else math.nan
        )
        status.ceiling_distance_filtered_m = (
            self.distance_filtered_m
            if self.distance_filtered_m is not None
            else math.nan
        )
        status.target_distance_m = self.target_distance_m
        status.compression_m = (
            self.contact_distance_m - self.distance_filtered_m
            if self.distance_filtered_m is not None
            else math.nan
        )
        status.target_compression_m = self.target_compression_m
        status.attach_confirmed = self._attach_confirmed(now)
        status.distance_stable = self._distance_stable(now)
        status.fault_detected = bool(
            not status.range_fresh
            or not status.odom_fresh
            or self._tilt_fault(now)
            or not status.flight_state_fresh
            or self.state == CeilingAttachmentStatus.STATE_FAULT
            or (
                self.state
                in (
                    CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
                    CeilingAttachmentStatus.STATE_SURFACE_HOLD,
                )
                and self._contact_lost()
            )
        )
        status.detach_failed = self.detach_failed
        status.integral_reset_request = self.state in (
            CeilingAttachmentStatus.STATE_APPROACH,
            CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
            CeilingAttachmentStatus.STATE_SURFACE_HOLD,
            CeilingAttachmentStatus.STATE_DETACH,
        )
        status.wheel_stop_request = bool(
            self.state == CeilingAttachmentStatus.STATE_DETACH
            or status.fault_detected
        )
        status.mechanism_release_requested = bool(
            self.state == CeilingAttachmentStatus.STATE_DETACH
        )
        status.mechanism_release_confirmed = self._mechanism_release_confirmed(now)
        status.approach_velocity_z_enu_mps = math.nan
        status.thrust_body_z_normalized = math.nan
        status.attitude_setpoint_valid = False

        if self.state == CeilingAttachmentStatus.STATE_APPROACH:
            status.approach_velocity_z_enu_mps = (
                0.0
                if self._state_age_s(now) < self.approach_settle_time_s
                else abs(self.approach_speed_mps)
            )
        elif self.state == CeilingAttachmentStatus.STATE_ATTACH_CONTROL:
            status.thrust_body_z_normalized = -self._attach_thrust(dt)
            status.attitude_setpoint_valid = True
            status.attitude_setpoint = self.q_hover
        elif self.state == CeilingAttachmentStatus.STATE_SURFACE_HOLD:
            status.thrust_body_z_normalized = -self._surface_hold_thrust()
            status.attitude_setpoint_valid = True
            status.attitude_setpoint = self.q_hover
        elif (
            self.state == CeilingAttachmentStatus.STATE_DETACH
            and status.range_fresh
        ):
            if self._mechanism_release_confirmed(now):
                status.thrust_body_z_normalized = self._detach_thrust(dt)
            else:
                status.thrust_body_z_normalized = -self._surface_hold_thrust()
            status.attitude_setpoint_valid = True
            status.attitude_setpoint = self.q_hover
        elif self.state == CeilingAttachmentStatus.STATE_FAULT:
            status.thrust_body_z_normalized = -self.hover_thrust
            status.attitude_setpoint_valid = True
            status.attitude_setpoint = self.q_hover

        status.candidate_valid = bool(
            status.enabled
            and status.flight_state_fresh
            and (
                math.isfinite(status.approach_velocity_z_enu_mps)
                or (
                    status.attitude_setpoint_valid
                    and math.isfinite(status.thrust_body_z_normalized)
                )
                or self.state
                in (
                    CeilingAttachmentStatus.STATE_CEILING_ARMED,
                    CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
                )
            )
        )

        status.fault_count = self.fault_count
        status.reason = self.last_reason
        return status

    @staticmethod
    def _build_control_candidate(status):
        candidate = AttachmentControlCandidate()
        candidate.header = status.header
        candidate.source = AttachmentControlCandidate.SOURCE_CEILING
        candidate.acquire_request = status.acquire_request
        candidate.keep_ownership = status.keep_ownership
        candidate.reason = status.reason
        candidate.velocity_z_enu_mps = math.nan
        candidate.thrust_normalized = math.nan

        if status.state == CeilingAttachmentStatus.STATE_APPROACH and math.isfinite(
            status.approach_velocity_z_enu_mps
        ):
            candidate.kind = AttachmentControlCandidate.KIND_VERTICAL_VELOCITY
            candidate.velocity_z_enu_mps = status.approach_velocity_z_enu_mps
        elif status.state in (
            CeilingAttachmentStatus.STATE_CEILING_ARMED,
            CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
        ):
            candidate.kind = AttachmentControlCandidate.KIND_VERTICAL_VELOCITY
            candidate.velocity_z_enu_mps = 0.0
        elif status.attitude_setpoint_valid and math.isfinite(
            status.thrust_body_z_normalized
        ):
            candidate.kind = AttachmentControlCandidate.KIND_ATTITUDE_THRUST
            candidate.attitude_setpoint = status.attitude_setpoint
            # Ceiling status preserves the PX4 reference's negative body-z
            # convention. The shared candidate uses MAVROS-positive thrust.
            candidate.thrust_normalized = -status.thrust_body_z_normalized
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
            dt = 0.02
        else:
            dt = clamp((now - self.previous_update_at).to_sec(), 0.001, 0.05)
        self.previous_update_at = now

        self._update_state_machine(now)
        status = self._build_status(now, dt)
        self.status_publisher.publish(status)
        self.control_candidate_publisher.publish(
            self._build_control_candidate(status)
        )
        self._publish_mechanism_candidate(now)

if __name__ == "__main__":
    rospy.init_node("ceiling_attachment_decision_node")
    CeilingAttachmentDecisionNode()
    rospy.spin()
