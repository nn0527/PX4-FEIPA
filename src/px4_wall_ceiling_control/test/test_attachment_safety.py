#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest
from unittest import mock

import rospy
from nav_msgs.msg import Odometry

from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    AttachmentMechanismCommand,
    CeilingAttachmentStatus,
    OperatorCommand,
    SensorHealth,
    WallPerchStatus,
)
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from px4_wall_ceiling_control.adapters.odometry import vertical_speed_enu_mps
from px4_wall_ceiling_control.adapters.takeoff_height import TakeoffHeightReference
from px4_wall_ceiling_control.domain.geometry import (
    follow_rotation_about_body_z,
    quaternion_angle,
    quaternion_multiply,
    rotate_vector,
)
from px4_wall_ceiling_control.domain.owner_arbiter import (
    CandidateIntent,
    OWNER_CEILING,
    OWNER_NONE,
    OwnerArbiter,
)


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))

SCRIPT_GROUPS = {
    "attachment_mechanism_output_node": "outputs",
    "attachment_output_mux_node": "outputs",
    "ceiling_attachment_decision_node": "decisions",
    "sensor_range_manager_node": "sensors",
    "wall_perch_decision_node": "decisions",
}


def load_script(name):
    script_path = os.path.join(SCRIPT_DIR, SCRIPT_GROUPS[name], name + ".py")
    spec = importlib.util.spec_from_file_location(name, script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CeilingDecision = load_script(
    "ceiling_attachment_decision_node"
).CeilingAttachmentDecisionNode
WallDecision = load_script("wall_perch_decision_node").WallPerchDecisionNode
SensorManagerModule = load_script("sensor_range_manager_node")
SensorManager = SensorManagerModule.SensorRangeManagerNode
OutputMux = load_script("attachment_output_mux_node").AttachmentOutputMuxNode
MechanismOutput = load_script(
    "attachment_mechanism_output_node"
).AttachmentMechanismOutputNode


class AttachmentSafetyTest(unittest.TestCase):
    def test_wall_detach_command_is_only_accepted_in_hold(self):
        node = WallDecision.__new__(WallDecision)
        node.detach_requested = False
        node.state = WallPerchStatus.STATE_WALL_CAPTURE
        node._on_detach(Bool(data=True))
        self.assertFalse(node.detach_requested)
        node.state = WallPerchStatus.STATE_WALL_HOLD
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10.0)):
            node._on_detach(Bool(data=True))
        self.assertTrue(node.detach_requested)

    def test_wall_hold_captures_measured_reference_without_setpoint_step(self):
        node = WallDecision.__new__(WallDecision)
        reference = WallDecision._hover_and_wall_quaternions(
            0.0, math.radians(90.0)
        )[1]
        tilt = (math.sin(math.radians(3.0) / 2.0), 0.0, 0.0,
                math.cos(math.radians(3.0) / 2.0))
        measured = quaternion_multiply(reference, tilt)
        odom = Odometry()
        odom.pose.pose.orientation.x = measured[0]
        odom.pose.pose.orientation.y = measured[1]
        odom.pose.pose.orientation.z = measured[2]
        odom.pose.pose.orientation.w = measured[3]
        node.odom = odom
        node.state = WallPerchStatus.STATE_WALL_CAPTURE
        node.q_contact = reference
        node.hover_thrust = 0.5
        node.hold_thrust_multiplier = 1.5
        node.hold_entry_blend_s = 0.5
        node.max_thrust = 0.9
        now = rospy.Time.from_sec(10.0)
        node._enter_state(WallPerchStatus.STATE_WALL_HOLD, now, "contact")
        self.assertAlmostEqual(
            quaternion_angle(node.q_hold_reference, measured), 0.0, places=6
        )
        at_entry = node._candidate(now)[1]
        after_blend = node._candidate(rospy.Time.from_sec(10.5))[1]
        self.assertAlmostEqual(quaternion_angle(at_entry, reference), 0.0, places=6)
        self.assertAlmostEqual(quaternion_angle(after_blend, measured), 0.0, places=6)

    def test_wall_follow_allows_wheel_turn_without_turning_thrust_away(self):
        reference = WallDecision._hover_and_wall_quaternions(
            0.0, math.radians(90.0)
        )[1]
        turn = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
        measured = quaternion_multiply(reference, turn)
        followed = follow_rotation_about_body_z(reference, measured)
        self.assertAlmostEqual(quaternion_angle(followed, measured), 0.0, places=6)
        for target, original in zip(
            rotate_vector(followed, (0.0, 0.0, -1.0)),
            rotate_vector(reference, (0.0, 0.0, -1.0)),
        ):
            self.assertAlmostEqual(target, original, places=6)

        tilt = (math.sin(math.radians(10.0) / 2.0), 0.0, 0.0,
                math.cos(math.radians(10.0) / 2.0))
        tilted = quaternion_multiply(measured, tilt)
        corrected = follow_rotation_about_body_z(reference, tilted)
        self.assertAlmostEqual(quaternion_angle(corrected, measured), 0.0, places=6)
        self.assertAlmostEqual(
            math.degrees(quaternion_angle(corrected, tilted)), 10.0, places=6
        )

    def test_wall_detach_waits_for_return_to_reference_and_stability(self):
        node = WallDecision.__new__(WallDecision)
        reference = WallDecision._hover_and_wall_quaternions(
            0.0, math.radians(90.0)
        )[1]
        node.state = WallPerchStatus.STATE_WALL_HOLD
        node.state_entered_at = rospy.Time.from_sec(1.0)
        node.q_contact = reference
        node.q_hover = (0.0, 0.0, 0.0, 1.0)
        node.q_detach_start = reference
        node.detach_requested = True
        node.detach_requested_at = rospy.Time.from_sec(10.0)
        node.cancel_requested = False
        node.detach_stable_since = None
        node.detach_stable_samples = 0
        node.last_reason = "wall hold"
        node.top_filtered_m = 0.04
        node.top_contact_distance_m = 0.05
        node.hold_time_s = 0.0
        node.hold_entry_blend_s = 0.5
        node.detach_reference_angle_rad = math.radians(10.0)
        node.detach_axis_error_rad = math.radians(5.0)
        node.detach_max_rate_rps = 0.15
        node.detach_max_abs_vertical_speed_mps = 0.15
        node.detach_stable_hold_s = 0.5
        node.odom_twist_in_child_frame = False
        node.odom_timeout_s = 0.50
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True
        node._pin_triggered = lambda: False
        node._odom_fresh = lambda _now: True

        def publish_attitude(at, quaternion):
            odom = Odometry()
            odom.header.stamp = rospy.Time.from_sec(at)
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]
            with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(at)):
                node._on_odom(odom)
            node._update_state_machine(rospy.Time.from_sec(at))

        turn = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
        publish_attitude(10.0, quaternion_multiply(reference, turn))
        self.assertEqual(node.state, WallPerchStatus.STATE_WALL_HOLD)
        self.assertIsNone(node.detach_stable_since)
        self.assertTrue(node.detach_requested)

        near_reference = quaternion_multiply(
            reference,
            (math.sin(math.radians(2.0) / 2.0), 0.0, 0.0,
             math.cos(math.radians(2.0) / 2.0)),
        )
        publish_attitude(11.0, near_reference)
        self.assertEqual(node.state, WallPerchStatus.STATE_WALL_HOLD)
        publish_attitude(11.6, near_reference)
        self.assertEqual(node.state, WallPerchStatus.STATE_DETACH_ROTATE)
        self.assertAlmostEqual(node.detach_altitude_m, 0.0)
        self.assertAlmostEqual(
            quaternion_angle(node.q_detach_start, near_reference), 0.0, places=6
        )

    def test_wall_positive_pitch_matches_verified_pitch_test_quaternion(self):
        # pitch_45_test.py uses euler_to_quaternion(0, +45 deg, yaw).
        yaw = 0.6
        pitch = math.radians(45.0)
        q_hover, q_wall = WallDecision._hover_and_wall_quaternions(
            yaw, math.radians(90.0)
        )
        q_mid = WallDecision._quat_slerp(q_hover, q_wall, 0.5)
        expected = (
            -math.sin(pitch / 2.0) * math.sin(yaw / 2.0),
            math.sin(pitch / 2.0) * math.cos(yaw / 2.0),
            math.cos(pitch / 2.0) * math.sin(yaw / 2.0),
            math.cos(pitch / 2.0) * math.cos(yaw / 2.0),
        )
        for actual, reference in zip(q_mid, expected):
            self.assertAlmostEqual(actual, reference, places=6)
        self.assertAlmostEqual(
            math.degrees(WallDecision._euler_from_quaternion(q_mid)[1]),
            45.0,
            places=6,
        )

    def test_early_wall_contact_keeps_attitude_continuous_through_detach(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_FLIP_TO_WALL
        node.state_entered_at = rospy.Time.from_sec(10.0)
        node.flip_time_s = 2.0
        node.capture_time_s = 0.5
        node.hold_entry_blend_s = 0.5
        node.detach_time_s = 2.0
        node.hover_thrust = 0.5
        node.flip_thrust_multiplier = 1.05
        node.hold_thrust_multiplier = 1.5
        node.max_thrust = 0.9
        node.detach_altitude_kp = 0.5
        node.detach_vertical_speed_kd = 0.3
        node.detach_compensation_start_rad = math.radians(75.0)
        node.q_hover, node.q_wall = WallDecision._hover_and_wall_quaternions(
            0.0, math.radians(90.0)
        )
        now = rospy.Time.from_sec(11.0)
        q_before = node._candidate(now)[1]
        node.odom = Odometry()
        node.odom.pose.pose.position.z = 1.0
        node.odom.pose.pose.orientation.x = q_before[0]
        node.odom.pose.pose.orientation.y = q_before[1]
        node.odom.pose.pose.orientation.z = q_before[2]
        node.odom.pose.pose.orientation.w = q_before[3]

        node._enter_state(WallPerchStatus.STATE_WALL_CAPTURE, now, "contact")
        q_capture = node._candidate(now)[1]
        node._enter_state(WallPerchStatus.STATE_WALL_HOLD, now, "confirmed")
        q_hold = node._candidate(now)[1]
        node._enter_state(WallPerchStatus.STATE_DETACH_ROTATE, now, "detach")
        q_detach = node._candidate(now)[1]

        for target in (q_capture, q_hold, q_detach):
            for actual, expected in zip(target, q_before):
                self.assertAlmostEqual(actual, expected, places=6)
        self.assertAlmostEqual(
            math.degrees(WallDecision._euler_from_quaternion(q_capture)[1]),
            45.0,
            places=6,
        )

    def test_ceiling_candidate_converts_reference_thrust_sign(self):
        status = CeilingAttachmentStatus(
            state=CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
            acquire_request=False,
            keep_ownership=True,
            candidate_valid=True,
            attitude_setpoint_valid=True,
            thrust_body_z_normalized=-0.6,
            reason="attach",
        )
        status.attitude_setpoint.w = 1.0

        node = CeilingDecision.__new__(CeilingDecision)
        node.motion_controller = mock.Mock(yaw=0.0)
        node.motion_sequence = 0
        node.motion_id = 'test'
        node.motion_ready = True
        node.motion_fatal = ''
        node.motion_paused = False
        node.motion_contact_released = False
        node._mechanism_release_confirmed = lambda _now: True
        candidate = CeilingDecision._build_control_candidate(node, status)

        self.assertEqual(
            candidate.kind,
            AttachmentControlCandidate.KIND_ATTITUDE_THRUST,
        )
        self.assertTrue(candidate.valid)
        self.assertAlmostEqual(candidate.thrust_normalized, 0.6)

    def test_wall_candidate_uses_generic_neutral_velocity(self):
        status = WallPerchStatus(
            acquire_request=True,
            keep_ownership=True,
            candidate_valid=True,
            neutral_setpoint_requested=True,
            reason="wait",
        )

        candidate = WallDecision._build_control_candidate(status)

        self.assertEqual(
            candidate.kind,
            AttachmentControlCandidate.KIND_VERTICAL_VELOCITY,
        )
        self.assertTrue(candidate.valid)
        self.assertEqual(candidate.velocity_z_enu_mps, 0.0)

    def test_odometry_adapter_rotates_child_velocity_into_enu(self):
        odom = Odometry()
        odom.pose.pose.orientation.y = math.sin(math.pi / 4.0)
        odom.pose.pose.orientation.w = math.cos(math.pi / 4.0)
        odom.twist.twist.linear.x = 1.0

        self.assertAlmostEqual(vertical_speed_enu_mps(odom), -1.0, places=6)
        self.assertAlmostEqual(
            vertical_speed_enu_mps(odom, twist_in_child_frame=False),
            0.0,
            places=6,
        )

    def test_owner_arbiter_is_ros_independent(self):
        arbiter = OwnerArbiter(handoff_dwell_s=0.5)
        ceiling = CandidateIntent(
            enabled=True,
            fresh=True,
            acquire_request=True,
            candidate_valid=True,
            required_operator_mode=2,
        )

        event = arbiter.update(
            now_s=1.0,
            operator_mode=2,
            operator_neutral=False,
            ceiling=ceiling,
            wall=CandidateIntent(required_operator_mode=3),
        )
        self.assertEqual(event.owner, OWNER_CEILING)

        event = arbiter.update(
            now_s=1.1,
            operator_mode=2,
            operator_neutral=False,
            ceiling=CandidateIntent(enabled=True, fresh=False),
            wall=CandidateIntent(required_operator_mode=3),
        )
        self.assertEqual(event.owner, OWNER_NONE)
        self.assertTrue(event.interlock_active)

    def test_ceiling_does_not_start_without_flight_ready(self):
        node = CeilingDecision.__new__(CeilingDecision)
        node.state = CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
        node.rearm_required = False
        node.detach_requested = False
        node._mode_requested = lambda _now: True
        node._arm_requested = lambda _now: True
        node._range_fresh = lambda _now: True
        node._odom_fresh = lambda _now: True
        node._flight_ready = lambda _now: False

        node._update_state_machine(rospy.Time.from_sec(10.0))

        self.assertEqual(
            node.state, CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
        )

    def test_ceiling_attached_odom_loss_latches_fault(self):
        node = CeilingDecision.__new__(CeilingDecision)
        node.state = CeilingAttachmentStatus.STATE_ATTACH_CONTROL
        node.detach_requested = False
        node.fault_count = 0
        node._mode_requested = lambda _now: True
        node._arm_requested = lambda _now: True
        node._range_fresh = lambda _now: True
        node._odom_fresh = lambda _now: False
        node._flight_ready = lambda _now: True
        node._tilt_fault = lambda _now: False
        node._contact_lost = lambda: False
        node._enter_state = lambda new_state, _now, _reason: setattr(
            node, "state", new_state
        )

        node._update_state_machine(rospy.Time.from_sec(10.0))

        self.assertEqual(node.state, CeilingAttachmentStatus.STATE_ABORT)




    def test_wall_state_entry_resets_hold_timer(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_STABILIZE_HOVER
        node.state_entered_at = rospy.Time.from_sec(1.0)
        node.last_reason = "test"
        node.flip_ready_since = rospy.Time.from_sec(0.1)
        node.front_ready_since = None
        node.top_contact_since = None
        node.top_clear_since = None

        node._enter_state(
            WallPerchStatus.STATE_SLOW_APPROACH,
            rospy.Time.from_sec(2.0),
            "test transition",
        )

        self.assertIsNone(node.flip_ready_since)

    def test_wall_fixed_rig_can_enter_detect_below_minimum_altitude(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_IDLE
        node.rearm_required = False
        node.cancel_requested = False
        node.detach_requested = False
        node.last_reason = "initialized"
        node.odom = Odometry()
        node.odom.pose.pose.position.z = -1.96
        node.min_altitude_m = -0.10
        node.require_min_altitude = False
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True
        node._enter_state = lambda new_state, _now, _reason: setattr(
            node, "state", new_state
        )

        node._update_state_machine(rospy.Time.from_sec(2.0))

        self.assertEqual(node.state, WallPerchStatus.STATE_FRONT_WALL_DETECT)

    def test_wall_flight_altitude_gate_still_blocks_low_odometry(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_IDLE
        node.rearm_required = False
        node.cancel_requested = False
        node.detach_requested = False
        node.last_reason = "initialized"
        node.odom = Odometry()
        node.odom.pose.pose.position.z = -1.96
        node.min_altitude_m = -0.10
        node.require_min_altitude = True
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True

        node._update_state_machine(rospy.Time.from_sec(2.0))

        self.assertEqual(node.state, WallPerchStatus.STATE_IDLE)
        self.assertIn("minimum altitude gate blocked", node.last_reason)

    def test_wall_uses_arming_ground_height_for_entry(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_IDLE
        node.rearm_required = node.cancel_requested = node.detach_requested = False
        node.odom = Odometry()
        node.odom.pose.pose.position.z = -0.149
        node.min_altitude_m = 0.3
        node.require_min_altitude = True
        node.altitude_reference = "arming_ground"
        node.takeoff_height = TakeoffHeightReference()
        node.takeoff_height.observe_state(True, False, 1)
        node.takeoff_height.observe_odom(-1.50466, 1.1, True)
        node.takeoff_height.observe_state(True, True, 1.2)
        node._mode_requested = node._safety_ok = lambda now: True
        node._enter_state = lambda state, now, reason: setattr(node, "state", state)
        node._update_state_machine(rospy.Time.from_sec(2))
        self.assertEqual(node.state, WallPerchStatus.STATE_FRONT_WALL_DETECT)
        node.state = WallPerchStatus.STATE_IDLE
        node.odom.pose.pose.position.z = -1.3
        node._update_state_machine(rospy.Time.from_sec(2))
        self.assertEqual(node.state, WallPerchStatus.STATE_IDLE)

    def test_wall_front_ready_enters_three_degree_approach_directly(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_FRONT_WALL_DETECT
        node.cancel_requested = False
        node.fault_count = 0
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True
        node._front_ready = lambda _now: True
        node._enter_state = lambda new_state, _now, _reason: setattr(
            node, "state", new_state
        )

        node._update_state_machine(rospy.Time.from_sec(2.0))

        self.assertEqual(node.state, WallPerchStatus.STATE_SLOW_APPROACH)

    def test_bench_detection_waits_for_real_ranges_before_approach(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_FRONT_WALL_DETECT
        node.bench_wait_for_ranges = True
        node.cancel_requested = False
        node.fault_count = 0
        node._mode_requested = node._safety_ok = node._front_ready = lambda now: True
        node._bench_ranges_ready = lambda now: False
        node._enter_state = lambda state, now, reason: setattr(node, "state", state)
        node._update_state_machine(rospy.Time.from_sec(2))
        self.assertEqual(node.state, WallPerchStatus.STATE_FRONT_WALL_DETECT)
        self.assertIn("bench hover", node.last_reason)
        node._bench_ranges_ready = lambda now: True
        node._update_state_machine(rospy.Time.from_sec(3))
        self.assertEqual(node.state, WallPerchStatus.STATE_SLOW_APPROACH)
        node.q_hover = (0.0, 0.0, 0.0, 1.0)
        node.approach_pitch_rad = math.radians(3.0)
        node.approach_thrust = 0.47
        node.max_thrust = 0.90
        neutral, quaternion, thrust, _progress = node._candidate(
            rospy.Time.from_sec(2.0)
        )
        self.assertFalse(neutral)
        self.assertAlmostEqual(quaternion[1], math.sin(math.radians(1.5)))
        self.assertAlmostEqual(quaternion[3], math.cos(math.radians(1.5)))
        self.assertAlmostEqual(thrust, 0.47)

    def test_wall_capture_uses_separate_timeout_without_delaying_safety_abort(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_WALL_CAPTURE
        node.state_entered_at = rospy.Time.from_sec(1.0)
        node.cancel_requested = False
        node.fault_count = 0
        node.capture_timeout_s = 5.0
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True
        node._pin_triggered = lambda: False
        node._top_contact_ready = lambda _now: False
        node._enter_state = lambda new_state, _now, _reason: setattr(
            node, "state", new_state
        )

        node._update_state_machine(rospy.Time.from_sec(5.9))
        self.assertEqual(node.state, WallPerchStatus.STATE_WALL_CAPTURE)
        self.assertEqual(node.fault_count, 0)

        node._safety_ok = lambda _now: False
        node._update_state_machine(rospy.Time.from_sec(5.95))
        self.assertEqual(node.state, WallPerchStatus.STATE_ABORT)
        self.assertEqual(node.fault_count, 1)

        node.state = WallPerchStatus.STATE_WALL_CAPTURE
        node.fault_count = 0
        node._safety_ok = lambda _now: True
        node._update_state_machine(rospy.Time.from_sec(6.1))
        self.assertEqual(node.state, WallPerchStatus.STATE_ABORT)
        self.assertEqual(node.fault_count, 1)

    def test_wall_front_lidar_is_required_only_before_flip(self):
        node = WallDecision.__new__(WallDecision)
        node._flight_ready = lambda _now: True
        node._odom_fresh = lambda _now: True
        node._top_preflight_ready = lambda _now: True
        node._front_fresh = lambda _now: False
        node._top_fresh = lambda _now: True
        node._rates_safe = lambda _limit=None: True
        node._vertical_speed_safe = lambda: True
        node.max_rate_rps = 3.0
        node.flip_max_rate_rps = 6.0
        node.capture_max_rate_rps = 4.5
        node.recover_max_rate_rps = 3.0
        now = rospy.Time.from_sec(10.0)

        for state in (
            WallPerchStatus.STATE_IDLE,
            WallPerchStatus.STATE_FRONT_WALL_DETECT,
            WallPerchStatus.STATE_STABILIZE_HOVER,
            WallPerchStatus.STATE_SLOW_APPROACH,
        ):
            node.state = state
            self.assertFalse(node._safety_ok(now), state)

        for state in (
            WallPerchStatus.STATE_FLIP_TO_WALL,
            WallPerchStatus.STATE_WALL_CAPTURE,
            WallPerchStatus.STATE_WALL_HOLD,
            WallPerchStatus.STATE_DETACH_ROTATE,
            WallPerchStatus.STATE_RECOVER,
        ):
            node.state = state
            self.assertTrue(node._safety_ok(now), state)

        node.state = WallPerchStatus.STATE_WALL_CAPTURE
        node._top_fresh = lambda _now: False
        self.assertTrue(node._safety_ok(now))

    def test_wall_status_does_not_flag_stale_front_after_flip(self):
        node = WallDecision.__new__(WallDecision)
        node.odom = None
        node.min_altitude_m = 0.3
        node.state = WallPerchStatus.STATE_WALL_CAPTURE
        node.front_range = None
        node.front_filtered_m = None
        node.top_range = None
        node.top_filtered_m = 0.04
        node.top_contact_distance_m = 0.05
        node.top_clear_distance_m = 0.10
        node.front_ready_distance_m = 0.50
        node.flip_trigger_distance_m = 0.20
        node.front_ready_hold_s = 0.40
        node.flip_trigger_hold_s = 0.30
        node.top_contact_hold_s = 1.0
        node.top_clear_hold_s = 0.50
        node.front_ready_since = None
        node.front_ready_last_sample_at = None
        node.flip_ready_since = None
        node.flip_ready_last_sample_at = None
        node.top_contact_since = None
        node.top_contact_last_sample_at = None
        node.top_clear_since = None
        node.top_clear_last_sample_at = None
        node.fault_count = 0
        node.last_reason = "test"
        node._front_fresh = lambda _now: False
        node._top_fresh = lambda _now: True
        node._sensor_health_fresh = lambda _now: True
        node._top_preflight_ready = lambda _now: True
        node._odom_fresh = lambda _now: True
        node._flight_ready = lambda _now: True
        node._held_status = lambda *_args: False
        node._candidate = lambda _now: (False, None, float("nan"), 0.0)

        status = node._build_status(rospy.Time.from_sec(10.0))

        self.assertFalse(status.front_range_fresh)
        self.assertFalse(status.fault_detected)

        node.state = WallPerchStatus.STATE_SLOW_APPROACH
        status = node._build_status(rospy.Time.from_sec(10.0))
        self.assertTrue(status.fault_detected)

    def test_wall_hold_recovers_small_gap_and_aborts_at_clear_distance(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_WALL_HOLD
        node.state_entered_at = rospy.Time.from_sec(1.0)
        node.cancel_requested = False
        node.detach_requested = False
        node.fault_count = 0
        node.top_contact_distance_m = 0.05
        node.top_clear_distance_m = 0.10
        node.top_filtered_m = 0.05
        node._top_fresh = lambda now: True
        node.top_clear_hold_s = .5
        node.top_clear_since = node.top_clear_last_sample_at = None
        node.hold_time_s = 1.0
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True
        node._pin_triggered = lambda: False
        node._enter_state = lambda new_state, _now, _reason: setattr(
            node, "state", new_state
        )

        node._update_state_machine(rospy.Time.from_sec(1.1))
        self.assertEqual(node.state, WallPerchStatus.STATE_WALL_HOLD)

        node.top_filtered_m = 0.051
        node._update_state_machine(rospy.Time.from_sec(1.2))
        self.assertEqual(node.state, WallPerchStatus.STATE_WALL_HOLD)

        node.top_filtered_m = 0.101
        node.top_clear_since = node.top_clear_last_sample_at = rospy.Time.from_sec(1.3)
        node._update_state_machine(rospy.Time.from_sec(1.3))
        self.assertEqual(node.state, WallPerchStatus.STATE_WALL_HOLD)
        node.top_clear_last_sample_at = rospy.Time.from_sec(1.8)
        node._update_state_machine(rospy.Time.from_sec(1.8))
        self.assertEqual(node.state, WallPerchStatus.STATE_ABORT)
        self.assertEqual(node.fault_count, 1)

    def test_wall_milestones_survive_abort_but_reset_for_next_acquire(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_IDLE
        node._capture_attitude_references = lambda: None
        node._current_quaternion = lambda: (0.0, 0.0, 0.0, 1.0)
        node._wall_pressure_start_thrust = lambda: 0.5
        node._wall_pressure_nominal_thrust = lambda: 0.6
        node.max_thrust = 0.85
        node.q_contact = (0.0, 0.0, 0.0, 1.0)
        node._odom_fresh = lambda _now: False
        now = rospy.Time.from_sec(10)
        node._enter_state(WallPerchStatus.STATE_FRONT_WALL_DETECT, now, "acquire")
        node._enter_state(WallPerchStatus.STATE_WALL_HOLD, now, "contact")
        self.assertTrue(node.hold_achieved)
        node._enter_state(WallPerchStatus.STATE_ABORT, now, "contact lost")
        self.assertTrue(node.hold_achieved)
        self.assertEqual(node.reached_state, WallPerchStatus.STATE_WALL_HOLD)
        node._enter_state(WallPerchStatus.STATE_EXIT, now, "abort recovered")
        self.assertFalse(node.detach_completed)
        node._enter_state(WallPerchStatus.STATE_IDLE, now, "release")
        node._enter_state(WallPerchStatus.STATE_FRONT_WALL_DETECT, now, "new acquire")
        self.assertFalse(node.hold_achieved)
        node._enter_state(WallPerchStatus.STATE_RECOVER, now, "detach recovery")
        node._enter_state(WallPerchStatus.STATE_EXIT, now, "recovery complete")
        self.assertTrue(node.detach_completed)

    def test_bench_detach_ignores_finite_vertical_speed_but_keeps_angular_limits(self):
        node = WallDecision.__new__(WallDecision)
        node.detach_vertical_speed_check_enabled = True
        node.odom = Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom.twist.twist.linear.z = 0.4
        node.odom_twist_in_child_frame = True
        node.q_contact = node.q_hold_reference = (0.0, 0.0, 0.0, 1.0)
        node.detach_reference_angle_rad = math.radians(10)
        node.detach_axis_error_rad = math.radians(5)
        node.detach_max_rate_rps = 0.15
        node.detach_max_abs_vertical_speed_mps = 0.15
        node.max_down_velocity_mps = .8
        self.assertFalse(node._detach_pose_valid())
        node.bench_wait_for_ranges = True
        self.assertTrue(node._detach_pose_valid())
        node.odom.twist.twist.angular.x = 0.2
        self.assertFalse(node._detach_pose_valid())
        node.odom.twist.twist.angular.x = 0
        node.odom.twist.twist.linear.z = float("nan")
        self.assertFalse(node._detach_pose_valid())

    def test_detach_request_waits_then_expires_without_rotating(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_WALL_HOLD
        node.cancel_requested = False
        node.detach_requested = True
        node.detach_requested_at = rospy.Time.from_sec(1)
        node.top_filtered_m = 0.04
        node.top_clear_distance_m = 0.10
        node._top_fresh = lambda now: False
        node.top_clear_hold_s = .5
        node.top_clear_since = node.top_clear_last_sample_at = None
        node.hold_time_s = 0
        node._mode_requested = node._safety_ok = lambda now: True
        node._pin_triggered = lambda: False
        node._detach_pose_ready = lambda now: False
        node._detach_block_reason = lambda now: "角速度未稳定"
        node._update_state_machine(rospy.Time.from_sec(2))
        self.assertTrue(node.detach_requested)
        self.assertEqual(node.state, WallPerchStatus.STATE_WALL_HOLD)
        node._update_state_machine(rospy.Time.from_sec(10))
        self.assertFalse(node.detach_requested)
        self.assertIn("超时", node.last_reason)

    def test_wall_pressure_ramps_with_top_range_and_returns_to_nominal(self):
        node = WallDecision.__new__(WallDecision)
        node.hover_thrust = 0.45
        node.flip_thrust_multiplier = 1.35
        node.hold_thrust_multiplier = 1.50
        node.max_thrust = 0.90
        node.wall_pressure_max_thrust = 0.75
        node.wall_pressure_ramp_up_per_s = 0.03
        node.wall_pressure_ramp_down_per_s = 0.05
        node.wall_pressure_target_distance_m = 0.05
        node.wall_pressure_tolerance_m = 0.01
        node.control_dt = 1.0
        node.top_filtered_m = 0.08
        node._top_fresh = lambda now: True
        node.wall_pressure_thrust_command = node._wall_pressure_start_thrust()

        increased = node._wall_pressure_thrust(rospy.Time.from_sec(10.))
        self.assertAlmostEqual(increased, 0.6375)

        for _ in range(10):
            capped = node._wall_pressure_thrust(rospy.Time.from_sec(10.))
        self.assertAlmostEqual(capped, 0.75)

        node.top_filtered_m = 0.05
        returned = node._wall_pressure_thrust(rospy.Time.from_sec(10.))
        self.assertAlmostEqual(returned, 0.70)
        returned = node._wall_pressure_thrust(rospy.Time.from_sec(10.))
        self.assertAlmostEqual(returned, 0.675)

    def test_wall_detach_uses_height_and_tilt_feedback(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_DETACH_ROTATE
        node.state_entered_at = rospy.Time.from_sec(1.0)
        node.detach_time_s = 1.0
        node.q_wall = (0.0, 0.70710678, 0.0, 0.70710678)
        node.q_contact = node.q_wall
        node.q_detach_start = node.q_contact
        node.q_hover = (0.0, 0.0, 0.0, 1.0)
        node.hover_thrust = 0.50
        node.hold_thrust_multiplier = 1.50
        node.max_thrust = 0.90
        node.detach_altitude_m = 1.0
        node.detach_altitude_kp = 0.50
        node.detach_vertical_speed_kd = 0.30
        node.detach_compensation_start_rad = math.radians(75.0)
        node.odom_twist_in_child_frame = False
        node.odom = Odometry()
        node.odom.pose.pose.position.z = 1.0

        orientation = node.odom.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = node.q_wall
        self.assertAlmostEqual(node._candidate(rospy.Time.from_sec(1.5))[2], 0.75)

        samples = []
        for timestamp in (1.0, 1.5, 2.0):
            progress = timestamp - 1.0
            measured = node._quat_slerp(node.q_wall, node.q_hover, progress)
            orientation.x, orientation.y, orientation.z, orientation.w = measured
            _neutral, _quaternion, thrust, _progress = node._candidate(
                rospy.Time.from_sec(timestamp)
            )
            samples.append(thrust)

        self.assertAlmostEqual(samples[0], 0.75)
        self.assertAlmostEqual(samples[1], 0.50 / math.cos(math.radians(45.0)))
        self.assertAlmostEqual(samples[2], 0.50)

        measured = node._quat_slerp(node.q_wall, node.q_hover, 0.5)
        orientation.x, orientation.y, orientation.z, orientation.w = measured
        node.odom.pose.pose.position.z = 0.95
        node.odom.twist.twist.linear.z = -0.10
        corrected = node._candidate(rospy.Time.from_sec(1.5))[2]
        self.assertGreater(corrected, samples[1])
        self.assertAlmostEqual(corrected, (0.50 + 0.025 + 0.030) / math.cos(math.radians(45.0)))

        node.odom.pose.pose.position.z = 0.50
        node.odom.twist.twist.linear.z = -0.50
        self.assertAlmostEqual(node._candidate(rospy.Time.from_sec(1.5))[2], 0.90)

        node.state = WallPerchStatus.STATE_RECOVER
        node.state_entered_at = rospy.Time.from_sec(2.0)
        orientation.x, orientation.y, orientation.z, orientation.w = node.q_hover
        node.odom.pose.pose.position.z = 1.0
        node.odom.twist.twist.linear.z = 0.0
        self.assertAlmostEqual(node._candidate(rospy.Time.from_sec(2.0))[2], 0.50)

    def test_wall_detach_aborts_when_safety_input_is_lost(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_DETACH_ROTATE
        node.state_entered_at = rospy.Time.from_sec(1.0)
        node.cancel_requested = False
        node.fault_count = 0
        node._mode_requested = lambda _now: True
        node._flight_ready = lambda _now: True
        node._odom_fresh = lambda _now: True
        node._top_preflight_ready = lambda _now: True
        node._front_required = lambda: False
        node._top_required = lambda: True
        node._top_fresh = lambda _now: False
        node._rates_safe = lambda _limit=None: True
        node._vertical_speed_safe = lambda: True
        node._height_feedback = lambda: (1.0, 0.0)
        node.recover_max_rate_rps = 3.0
        node._enter_state = lambda new_state, _now, _reason: setattr(node, "state", new_state)

        node._update_state_machine(rospy.Time.from_sec(1.2))
        self.assertEqual(node.state, WallPerchStatus.STATE_ABORT)
        self.assertEqual(node.fault_count, 1)

    def test_wall_recovery_waits_for_captured_height_speed_and_clearance(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_RECOVER
        node.state_entered_at = rospy.Time.from_sec(1.0)
        node.detach_altitude_m = 1.0
        node.recovery_altitude_m = 1.0
        node.detach_altitude_tolerance_m = 0.10
        node.detach_vertical_speed_tolerance_mps = 0.20
        node.recover_time_s = 1.0
        node.recover_max_rate_rps = 3.0
        node.cancel_requested = False
        node.fault_count = 0
        node.odom_twist_in_child_frame = False
        node.odom = Odometry()
        node.odom.pose.pose.position.z = 0.80
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True
        node._attitude_recovered = lambda: True
        node._rates_safe = lambda _limit=None: True
        node._vertical_speed_safe = lambda: True
        node._top_clear_ready = lambda _now: True
        node._departure_clearance_ready = lambda _now: False
        node._enter_state = lambda new_state, _now, _reason: setattr(node, "state", new_state)

        node._update_state_machine(rospy.Time.from_sec(2.0))
        self.assertEqual(node.state, WallPerchStatus.STATE_RECOVER)
        node.odom.pose.pose.position.z = 0.95
        node.odom.twist.twist.linear.z = -0.25
        node._update_state_machine(rospy.Time.from_sec(2.1))
        self.assertEqual(node.state, WallPerchStatus.STATE_RECOVER)
        node.odom.twist.twist.linear.z = -0.05
        node._update_state_machine(rospy.Time.from_sec(2.2))
        self.assertEqual(node.state, WallPerchStatus.STATE_RECOVER)
        node._departure_clearance_ready = lambda _now: True
        node._update_state_machine(rospy.Time.from_sec(2.3))
        self.assertEqual(node.state, WallPerchStatus.STATE_EXIT)

    def test_bench_recovery_ignores_radar_and_height_drift(self):
        node = WallDecision.__new__(WallDecision)
        node.bench_wait_for_ranges = True
        node.state = WallPerchStatus.STATE_RECOVER
        node.state_entered_at = rospy.Time.from_sec(1.0)
        node.recover_time_s = 1.0
        node.recover_max_rate_rps = 3.0
        node.cancel_requested = False
        node.fault_count = 0
        node.odom_twist_in_child_frame = False
        node.odom = Odometry()
        node.odom.pose.pose.position.z = -20.0
        node.odom.twist.twist.linear.z = -2.0
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True
        node._attitude_recovered = lambda: True
        node._rates_safe = lambda _limit=None: True
        node._top_clear_ready = lambda _now: False
        node._enter_state = lambda state, _now, _reason: setattr(node, "state", state)
        now = rospy.Time.from_sec(2.0)
        self.assertTrue(node._top_preflight_ready(now))
        self.assertFalse(node._top_required())
        node._update_state_machine(now)
        self.assertEqual(node.state, WallPerchStatus.STATE_EXIT)

    def test_bench_departure_safety_does_not_depend_on_either_radar(self):
        node = WallDecision.__new__(WallDecision)
        node.bench_wait_for_ranges = True
        node._flight_ready = node._odom_fresh = lambda _now: True
        node._front_fresh = node._top_fresh = lambda _now: False
        node._rates_safe = lambda _limit: True
        node._vertical_speed_safe = lambda: True
        node.recover_max_rate_rps = 3.0
        node.sensor_health = None
        node.max_rate_rps = 3.0
        for state in (WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER,
                      WallPerchStatus.STATE_ABORT, WallPerchStatus.STATE_EXIT):
            node.state = state
            self.assertTrue(node._safety_ok(rospy.Time.from_sec(10.0)))

    def test_bench_approach_pauses_on_invalid_range_without_flipping(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_SLOW_APPROACH
        node.bench_wait_for_ranges = True
        node.cancel_requested = False
        node._mode_requested = lambda _now: True
        node._safety_ok = lambda _now: True
        node._bench_ranges_ready = lambda _now: False
        node._enter_state = mock.Mock()
        now = rospy.Time.from_sec(10.0)
        node._update_state_machine(now)
        self.assertEqual(node.state, WallPerchStatus.STATE_SLOW_APPROACH)
        node._enter_state.assert_not_called()
        self.assertTrue(node._candidate(now)[0])
        self.assertIsNone(node._candidate(now)[1])
        self.assertIsNone(node.flip_ready_since)

    def test_exit_holds_output_until_fcu_leaves_offboard(self):
        from mavros_msgs.msg import State
        node = WallDecision.__new__(WallDecision)
        now = rospy.Time.from_sec(10.0)
        node.state = WallPerchStatus.STATE_EXIT
        node.cancel_requested = False
        node._mode_requested = lambda _now: False
        node._flight_ready = lambda _now: True
        node.flight_state = State(connected=True, armed=True, mode='OFFBOARD')
        node._departure_clearance_ready = lambda _now: True
        node._enter_state = mock.Mock()
        node._update_state_machine(now)
        node._enter_state.assert_not_called()
        node.q_hover = (0.0, 0.0, 0.0, 1.0)
        node.hover_thrust = 0.45
        node.max_thrust = 0.85
        node.recovery_max_thrust = 0.6
        node._recovery_height_thrust = mock.Mock(return_value=0.45)
        self.assertEqual(node._candidate(now)[1:3], (node.q_hover, 0.45))
        node._flight_ready = lambda _now: False
        node.flight_state.mode = 'POSCTL'
        node.flight_state_received_at = now
        node.state_timeout_s = 1.5
        node._update_state_machine(now)
        self.assertEqual(node._enter_state.call_args[0][0], WallPerchStatus.STATE_IDLE)

    def test_stale_top_range_cannot_confirm_contact(self):
        node = WallDecision.__new__(WallDecision)
        node.top_filtered_m = 0.04
        node.top_received_at = rospy.Time.from_sec(1.0)
        node.sensor_timeout_s = 0.60
        node.top_contact_distance_m = 0.05
        node.top_contact_hold_s = 1.0
        node.top_contact_since = rospy.Time.from_sec(1.0)
        node.top_contact_last_sample_at = rospy.Time.from_sec(1.2)

        self.assertFalse(node._top_contact_ready(rospy.Time.from_sec(10.0)))
        self.assertIsNone(node.top_contact_since)

    def test_hold_confirmation_requires_distinct_samples(self):
        since, last = WallDecision._record_hold_sample(
            True, rospy.Time.from_sec(1.0), None
        )
        self.assertFalse(
            WallDecision._held_samples(True, since, last, 1.0)
        )

        since, last = WallDecision._record_hold_sample(
            True, rospy.Time.from_sec(1.9), since
        )
        self.assertFalse(WallDecision._held_samples(True, since, last, 1.0))

        since, last = WallDecision._record_hold_sample(
            True, rospy.Time.from_sec(2.0), since
        )
        self.assertTrue(
            WallDecision._held_samples(True, since, last, 1.0)
        )

    def test_sensor_health_tracks_channels_independently(self):
        node = SensorManager.__new__(SensorManager)
        node.input_timeout_s = 0.60
        node.ceiling_received_at = rospy.Time.from_sec(9.8)
        node.front_received_at = rospy.Time.from_sec(8.0)
        node.ceiling_latest_valid = True
        node.front_latest_valid = True
        node.ceiling_sequence = 3
        node.front_sequence = 7
        node.ceiling_invalid_count = 1
        node.front_invalid_count = 2

        status = node._build_health(rospy.Time.from_sec(10.0))

        self.assertIsInstance(status, SensorHealth)
        self.assertTrue(status.ceiling_valid)
        self.assertFalse(status.front_valid)
        self.assertEqual(status.ceiling_sequence, 3)
        self.assertEqual(status.front_sequence, 7)

    def test_sensor_manager_rejects_invalid_range(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = SensorManager.__new__(SensorManager)
        node.front_publisher = Publisher()
        node.front_received_at = None
        node.front_latest_valid = False
        node.front_sequence = 0
        node.front_invalid_count = 0

        with mock.patch.object(
            SensorManagerModule.rospy.Time,
            "now",
            return_value=rospy.Time.from_sec(10.0),
        ):
            node._on_front(
                Range(range=float("nan"), min_range=0.03, max_range=40.0)
            )

        self.assertEqual(node.front_publisher.messages, [])
        self.assertFalse(node.front_latest_valid)
        self.assertEqual(node.front_sequence, 0)
        self.assertEqual(node.front_invalid_count, 1)

    def test_health_timer_does_not_republish_range(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = SensorManager.__new__(SensorManager)
        node.input_timeout_s = 0.60
        node.ceiling_received_at = None
        node.front_received_at = None
        node.ceiling_latest_valid = False
        node.front_latest_valid = False
        node.ceiling_sequence = 0
        node.front_sequence = 0
        node.ceiling_invalid_count = 0
        node.front_invalid_count = 0
        node.front_publisher = Publisher()
        node.health_publisher = Publisher()

        with mock.patch.object(
            SensorManagerModule.rospy.Time,
            "now",
            return_value=rospy.Time.from_sec(10.0),
        ):
            node._on_front(Range(range=1.0, min_range=0.03, max_range=40.0))
            node._on_timer(None)
            node._on_timer(None)

        self.assertEqual(len(node.front_publisher.messages), 1)
        self.assertEqual(len(node.health_publisher.messages), 2)
        self.assertEqual(node.front_sequence, 1)

    def test_owner_handover_requires_neutral_interlock(self):
        node = OutputMux.__new__(OutputMux)
        node.owner = OutputMux.OWNER_CEILING
        node.interlock_active = False
        node.interlock_clear_since = None
        node.handoff_dwell_s = 0.50
        node.ceiling_output_enabled = True
        node.wall_output_enabled = True
        node.ceiling_candidate_timeout_s = 0.20
        node.wall_candidate_timeout_s = 0.20
        node.operator_timeout_s = 0.50
        node.ceiling_candidate = AttachmentControlCandidate(keep_ownership=False)
        node.wall_candidate = AttachmentControlCandidate(
            acquire_request=True, valid=True
        )
        node.ceiling_candidate_at = rospy.Time.from_sec(10.0)
        node.wall_candidate_at = rospy.Time.from_sec(10.0)
        node.operator = OperatorCommand(
            mode=OperatorCommand.MODE_WALL, enable_control=True
        )
        node.operator_at = rospy.Time.from_sec(10.0)

        node._update_owner(rospy.Time.from_sec(10.0))
        self.assertEqual(node.owner, OutputMux.OWNER_NONE)
        self.assertTrue(node.interlock_active)

        node._update_owner(rospy.Time.from_sec(10.2))
        self.assertEqual(node.owner, OutputMux.OWNER_NONE)

        node.operator = OperatorCommand(
            mode=OperatorCommand.MODE_STANDBY, enable_control=False
        )
        node.operator_at = rospy.Time.from_sec(10.2)
        node._update_owner(rospy.Time.from_sec(10.2))
        node.operator_at = rospy.Time.from_sec(10.7)
        node._update_owner(rospy.Time.from_sec(10.7))
        self.assertFalse(node.interlock_active)
        self.assertEqual(node.owner, OutputMux.OWNER_NONE)

    def test_owner_acquisition_must_match_operator_mode(self):
        node = OutputMux.__new__(OutputMux)
        node.owner = OutputMux.OWNER_NONE
        node.interlock_active = False
        node.interlock_clear_since = None
        node.ceiling_output_enabled = True
        node.wall_output_enabled = True
        node.ceiling_candidate_timeout_s = 0.20
        node.wall_candidate_timeout_s = 0.20
        node.operator_timeout_s = 0.50
        node.ceiling_candidate = AttachmentControlCandidate(
            acquire_request=True, valid=True
        )
        node.wall_candidate = AttachmentControlCandidate(
            acquire_request=False, valid=True
        )
        node.ceiling_candidate_at = rospy.Time.from_sec(10.0)
        node.wall_candidate_at = rospy.Time.from_sec(10.0)
        node.operator = OperatorCommand(
            mode=OperatorCommand.MODE_WALL, enable_control=True
        )
        node.operator_at = rospy.Time.from_sec(10.0)

        node._update_owner(rospy.Time.from_sec(10.0))

        self.assertEqual(node.owner, OutputMux.OWNER_NONE)

    def test_output_mux_rejects_stale_odometry(self):
        node = OutputMux.__new__(OutputMux)
        node.odom = Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom_at = rospy.Time.from_sec(9.0)
        node.odom_timeout_s = 0.50

        self.assertFalse(node._odom_fresh(rospy.Time.from_sec(10.0)))

        node.odom_at = rospy.Time.from_sec(9.8)
        self.assertTrue(node._odom_fresh(rospy.Time.from_sec(10.0)))

        node.odom.twist.twist.linear.z = float("nan")
        self.assertFalse(node._odom_fresh(rospy.Time.from_sec(10.0)))

    def test_invalid_candidate_retains_existing_owner(self):
        node = OutputMux.__new__(OutputMux)
        node.owner = OutputMux.OWNER_WALL
        node.interlock_active = False
        node.interlock_clear_since = None
        node.wall_output_enabled = True
        node.wall_candidate_timeout_s = 0.20
        node.wall_candidate = AttachmentControlCandidate(
            source=AttachmentControlCandidate.SOURCE_WALL,
            keep_ownership=True, valid=False, phase_valid=True,
            control_phase=WallPerchStatus.STATE_FLIP_TO_WALL,
            maneuver_id="invalid-payload-existing-owner", control_sequence=2,
        )
        node.wall_candidate.header.stamp = rospy.Time.from_sec(10.0)
        node.wall_candidate_at = rospy.Time.from_sec(10.0)
        node.wall_candidate_snapshot = (node.wall_candidate, node.wall_candidate_at)

        node._update_owner(rospy.Time.from_sec(10.0))

        self.assertEqual(node.owner, OutputMux.OWNER_WALL)

    def test_mechanism_output_requires_fresh_ceiling_owner(self):
        node = MechanismOutput.__new__(MechanismOutput)
        node.output_enabled = True
        node.require_ceiling_owner = True
        node.candidate_timeout_s = 0.20
        node.owner_timeout_s = 0.30
        node.candidate = AttachmentMechanismCommand(
            source=AttachmentMechanismCommand.SOURCE_CEILING,
            action=AttachmentMechanismCommand.ACTION_HOLD,
            sequence=4,
        )
        node.candidate_at = rospy.Time.from_sec(9.9)
        node.owner = MechanismOutput.OWNER_CEILING
        node.owner_at = rospy.Time.from_sec(9.9)

        authorized, _reason = node._authorized(rospy.Time.from_sec(10.0))
        self.assertTrue(authorized)

        node.owner = 0
        authorized, _reason = node._authorized(rospy.Time.from_sec(10.0))
        self.assertFalse(authorized)

    def test_wall_preflight_does_not_require_top_sensor_health(self):
        node = WallDecision.__new__(WallDecision)
        node.state = WallPerchStatus.STATE_IDLE
        node.require_top_sensor_on_acquire = True
        node.sensor_health_timeout_s = 0.30
        node.sensor_health = SensorHealth(ceiling_valid=True)
        node.sensor_health_received_at = rospy.Time.from_sec(9.9)

        self.assertTrue(node._top_preflight_ready(rospy.Time.from_sec(10.0)))

        node.sensor_health.ceiling_valid = False
        self.assertTrue(node._top_preflight_ready(rospy.Time.from_sec(10.0)))


if __name__ == "__main__":
    unittest.main()
