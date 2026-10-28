#!/usr/bin/env python3

import importlib.util
import math
import os
import time
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, State, RCIn
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    CeilingAttachmentStatus,
    OperatorCommand,
    SensorHealth,
    WallPerchStatus,
)
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion


def wall_control_fixture(state, now, paused=False):
    """Atomic phase metadata for helpers; /wall/status is only telemetry now."""
    candidate = AttachmentControlCandidate(
        source=AttachmentControlCandidate.SOURCE_WALL,
        kind=AttachmentControlCandidate.KIND_ATTITUDE_THRUST,
        phase_valid=True, control_phase=state, maneuver_id="wall-output-fixture",
        control_sequence=1, approach_paused=paused, valid=True,
        acquire_request=state in (WallPerchStatus.STATE_FRONT_WALL_DETECT,
                                 WallPerchStatus.STATE_SLOW_APPROACH,
                                 WallPerchStatus.STATE_FLIP_TO_WALL),
        keep_ownership=state != WallPerchStatus.STATE_IDLE,
        attitude_setpoint=Quaternion(w=1), thrust_normalized=.47)
    candidate.header.stamp = now
    return candidate


def load_script(name):
    path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "scripts", "operator", name + ".py")
    )
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_output_script():
    path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "scripts",
            "outputs",
            "attachment_output_mux_node.py",
        )
    )
    spec = importlib.util.spec_from_file_location("attachment_output_mux_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


Manager = load_script("flight_mode_manager_node").FlightModeManagerNode
Terminal = load_script("terminal_operator_node").TerminalOperatorNode
OutputMux = load_output_script().AttachmentOutputMuxNode


class FlightHandoverTest(unittest.TestCase):
    def test_vertical_candidate_never_uses_local_velocity_output(self):
        node = OutputMux.__new__(OutputMux)
        now = rospy.Time.from_sec(10.0)
        node.odom = Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom.twist.twist.linear.z = -0.2
        node.odom_at = now
        node.odom_timeout_s = 0.5
        node.odom_twist_in_child_frame = True
        node.hover_thrust = 0.5
        node.vertical_speed_kp = 0.15
        node.max_velocity_z_mps = 0.3
        node.min_thrust = 0.1
        node.max_thrust = 0.9
        node.local_frame_id = "map"
        node.rc_assist_enabled = False
        node.attitude_publisher = mock.Mock()
        node.local_publisher = mock.Mock()
        candidate = AttachmentControlCandidate(
            source=AttachmentControlCandidate.SOURCE_CEILING,
            kind=AttachmentControlCandidate.KIND_VERTICAL_VELOCITY,
            velocity_z_enu_mps=0.0,
        )
        result = node._publish_candidate(now, candidate)
        self.assertEqual(result, OutputMux.OUTPUT_ATTITUDE_THRUST)
        node.local_publisher.publish.assert_not_called()
        target = node.attitude_publisher.publish.call_args.args[0]
        self.assertAlmostEqual(target.thrust, 0.53)

    def test_direct_rc_trim_is_bounded_and_lost_input_recenters(self):
        node = OutputMux.__new__(OutputMux)
        node.rc_assist_enabled = True
        node.rc_roll_channel_index = 0
        node.rc_pitch_channel_index = 1
        node.rc_pwm_low = 1000
        node.rc_pwm_center = 1500
        node.rc_pwm_high = 2000
        node.rc_deadzone = 0.06
        node.rc_roll_reverse = False
        node.rc_pitch_reverse = False
        node.rc_timeout_s = 0.2
        node.rc_slew_deg_s = 90.0
        node.rc_free_max_angle_deg = 8.0
        node.rc_contact_max_angle_deg = 2.0
        node.rc_wall_detach_roll_deg = 2.0
        node.rc_wall_detach_pitch_deg = 0.0
        node.status_timeout_s = 0.3
        node.rc_channels = (2000, 1500)
        node.rc_at = rospy.Time.from_sec(10.0)
        node.rc_roll_rad = node.rc_pitch_rad = 0.0
        node.rc_trim_at = rospy.Time.from_sec(9.9)
        node.rc_age_publisher = mock.Mock()
        orientation = Quaternion(w=1.0)

        output = node._assisted_attitude(
            rospy.Time.from_sec(10.0), orientation,
            AttachmentControlCandidate.SOURCE_NONE,
        )
        roll = euler_from_quaternion((output.x, output.y, output.z, output.w))[0]
        self.assertAlmostEqual(math.degrees(roll), 8.0)

        node.wall_status = WallPerchStatus(state=WallPerchStatus.STATE_WALL_HOLD)
        node.wall_status_at = rospy.Time.from_sec(10.01)
        output = node._assisted_attitude(
            rospy.Time.from_sec(10.01), orientation,
            AttachmentControlCandidate.SOURCE_WALL,
            wall_control_fixture(WallPerchStatus.STATE_WALL_HOLD, rospy.Time.from_sec(10.01)),
        )
        roll = euler_from_quaternion((output.x, output.y, output.z, output.w))[0]
        self.assertLessEqual(math.degrees(roll), 2.001)

        node.rc_at = rospy.Time.from_sec(9.0)
        with mock.patch.object(rospy, "logwarn_throttle"):
            node._assisted_attitude(
                rospy.Time.from_sec(10.11), orientation,
                AttachmentControlCandidate.SOURCE_NONE,
            )
        self.assertLess(node.rc_roll_rad, math.radians(2.0))

    def test_handover_requires_centered_rc_and_level_vertical_state(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.output_mode = 2
        node.output_at = now
        node.output_timeout_s = 0.3
        node.estimator = EstimatorStatus(
            attitude_status_flag=True, velocity_vert_status_flag=True, velocity_horiz_status_flag=True, pos_horiz_rel_status_flag=True
        )
        node.estimator_at = now
        node.estimator_timeout_s = 0.5
        node.odom = Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom_at = now
        node.odom_timeout_s = 0.5
        node.odom_twist_in_child_frame = True
        node.handover_max_tilt_deg = 10.0
        node.handover_max_vertical_speed_mps = 0.2
        node.require_rc_center = True
        node.rc_channels = (1500, 1500, 1500)
        node.rc_at = now
        node.rc_roll_channel_index = 0
        node.rc_pitch_channel_index = 1
        node.rc_throttle_channel_index = 2
        node.rc_pwm_center = 1500
        node.rc_center_tolerance_pwm = 70
        node.rc_timeout_s = 0.2
        self.assertTrue(node._handover_ready(now))
        node.rc_channels = (1500, 1500, 1700)
        self.assertFalse(node._handover_ready(now))
        node.rc_channels = (1500, 1500, 1500)
        node.odom.twist.twist.linear.z = -0.3
        self.assertFalse(node._handover_ready(now))
        node.odom.twist.twist.linear.z = 0.0
        node.rc_at = rospy.Time.from_sec(9.0)
        self.assertFalse(node._handover_ready(now))

    def test_mux_prestreams_only_on_fresh_manager_request(self):
        node = OutputMux.__new__(OutputMux)
        now = rospy.Time.from_sec(10.0)
        node.owner = OutputMux.OWNER_NONE
        node.output_enabled = True
        node.prestream_requested = True
        node.prestream_at = now
        node.prestream_timeout_s = 0.3
        node.flight_state = State(connected=True, armed=True, mode="ALTCTL")
        node.flight_state_at = now
        node.state_timeout_s = 0.5
        node._update_owner = mock.Mock()
        node._odom_fresh = lambda _now: True
        node._vertical_attitude_target = lambda _now: object()
        node._rc_ready = lambda _now: True
        node.owner_publisher = mock.Mock()
        node.local_publisher = mock.Mock()
        node.attitude_publisher = mock.Mock()
        node.mode_publisher = mock.Mock()

        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        node.attitude_publisher.publish.assert_called_once()
        node.local_publisher.publish.assert_not_called()

        node.prestream_at = rospy.Time.from_sec(9.0)
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        node.attitude_publisher.publish.assert_called_once()

    def test_mux_matches_live_altctl_thrust_then_smoothly_reaches_hover(self):
        node = OutputMux.__new__(OutputMux)
        now = rospy.Time.from_sec(10.0)
        node.match_fcu_thrust_on_handover = True
        node.fcu_target_timeout_s = 0.20
        node.thrust_handover_ramp_s = 1.0
        node.min_thrust = 0.10
        node.max_thrust = 0.45
        node.max_velocity_z_mps = 0.30
        node.local_frame_id = "map"
        node.setpoint_builder = None
        node.fcu_attitude_target = AttitudeTarget(type_mask=0, thrust=0.62)
        node.fcu_attitude_target_at = now
        node.handover_start_thrust = None
        node.handover_started_at = None
        node.flight_state = State(mode="ALTCTL")

        self.assertTrue(node._handover_source_ready(now))
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_state(State(connected=True, armed=True, mode="OFFBOARD"))

        self.assertAlmostEqual(node.handover_start_thrust, 0.62)
        self.assertIsNone(node.handover_started_at)
        orientation = Quaternion(w=1.0)
        first = node._attitude_target(now, orientation, 0.45)
        halfway = node._attitude_target(
            rospy.Time.from_sec(10.5), orientation, 0.45
        )
        settled = node._attitude_target(
            rospy.Time.from_sec(11.0), orientation, 0.45
        )
        self.assertAlmostEqual(first.thrust, 0.62)
        self.assertAlmostEqual(halfway.thrust, 0.535)
        self.assertAlmostEqual(settled.thrust, 0.45)

    def test_mux_keeps_captured_thrust_after_offboard_feedback_becomes_stale(self):
        node = OutputMux.__new__(OutputMux)
        now = rospy.Time.from_sec(10.5)
        node.match_fcu_thrust_on_handover = True
        node.fcu_target_timeout_s = 0.20
        node.flight_state = State(mode="OFFBOARD")
        node.fcu_attitude_target = AttitudeTarget(type_mask=0, thrust=0.20)
        node.fcu_attitude_target_at = rospy.Time.from_sec(9.0)
        node.handover_start_thrust = 0.62
        orientation = Quaternion(w=1.0)
        expected = AttitudeTarget(thrust=0.62)
        node._level_attitude = mock.Mock(return_value=orientation)
        node._assisted_attitude = mock.Mock(return_value=orientation)
        node._attitude_target = mock.Mock(return_value=expected)

        target = node._prestream_attitude_target(now)

        self.assertIs(target, expected)
        node._attitude_target.assert_called_once_with(
            now, orientation, 0.62, apply_handover=False
        )

    def test_mux_rejects_stale_ignored_or_out_of_bounds_handover_thrust(self):
        node = OutputMux.__new__(OutputMux)
        now = rospy.Time.from_sec(10.0)
        node.match_fcu_thrust_on_handover = True
        node.fcu_target_timeout_s = 0.20
        node.min_thrust = 0.10
        node.max_thrust = 0.45
        node.fcu_attitude_target = AttitudeTarget(type_mask=0, thrust=0.46)
        node.fcu_attitude_target_at = now
        self.assertTrue(node._handover_source_ready(now))
        node.fcu_attitude_target.thrust = 0.001
        self.assertTrue(node._handover_source_ready(now))
        node.fcu_attitude_target.thrust = 1.01
        self.assertFalse(node._handover_source_ready(now))

        node.fcu_attitude_target.thrust = 0.32
        node.fcu_attitude_target.type_mask = AttitudeTarget.IGNORE_THRUST
        self.assertFalse(node._handover_source_ready(now))

        node.fcu_attitude_target.type_mask = 0
        node.fcu_attitude_target_at = rospy.Time.from_sec(9.0)
        self.assertFalse(node._handover_source_ready(now))

    def test_preflight_requires_both_ranges_for_wall(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.enabled = True
        node.flight = State(connected=True, armed=True, mode="ALTCTL")
        node.flight_at = now
        node.odom = Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom_at = now
        node.estimator = EstimatorStatus(
            attitude_status_flag=True,
            velocity_horiz_status_flag=True,
            pos_horiz_rel_status_flag=True,
            velocity_vert_status_flag=True,
        )
        node.estimator_at = now
        node.sensor = SensorHealth(ceiling_valid=True, front_valid=False)
        node.sensor_at = now
        node.state_timeout_s = node.odom_timeout_s = 0.5
        node.estimator_timeout_s = node.sensor_timeout_s = 0.5

        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_CEILING))
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        self.assertEqual(
            node._preflight_failure_reason(now, OperatorCommand.MODE_WALL),
            "front sensor invalid",
        )
        node.ignore_range_loss_during_attachment = True
        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.sensor = None
        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_CEILING))
        node.sensor = SensorHealth(ceiling_valid=True, front_valid=False)
        node.ignore_range_loss_during_attachment = False
        node.wall_bench_wait_for_ranges = True
        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.flight.armed = False
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.flight.armed = True
        node.estimator.velocity_vert_status_flag = False
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.estimator.velocity_vert_status_flag = True
        node.wall_bench_wait_for_ranges = False
        node.sensor.front_valid = True
        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.sensor_at = rospy.Time.from_sec(9.0)
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        self.assertEqual(
            node._preflight_failure_reason(now, OperatorCommand.MODE_WALL),
            "sensor health timeout",
        )

        node.sensor_at = now
        node.estimator.velocity_horiz_status_flag = False
        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.estimator.velocity_vert_status_flag = False
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        self.assertEqual(
            node._preflight_failure_reason(now, OperatorCommand.MODE_WALL),
            "attitude or vertical velocity estimate invalid",
        )
        node.estimator.velocity_vert_status_flag = True
        node.estimator.velocity_horiz_status_flag = True
        node.estimator_at = rospy.Time.from_sec(9.0)
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))

    def test_offboard_request_waits_for_neutral_stream(self):
        node = Manager.__new__(Manager)
        node.phase = Manager.PRESTREAM
        node.phase_since = time.monotonic()
        node.selected_mode = OperatorCommand.MODE_CEILING
        node.stream_since = time.monotonic() - 2.0
        node.prestream_s = 1.5
        node.flight = State(mode="POSCTL")
        now = rospy.Time.from_sec(10.0)
        node.flight_at = now
        node.state_timeout_s = 0.5
        node._requested_mode = lambda _now: OperatorCommand.MODE_CEILING
        node._preflight_ready = lambda _now, _selected: True
        node._stream_ready = lambda _now: False
        node._request_mode = mock.Mock()
        node.prestream_pub = mock.Mock()
        node.phase_pub = mock.Mock()

        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.PRESTREAM)
        node._request_mode.assert_not_called()
        self.assertIsNone(node.stream_since)

        node._stream_ready = lambda _now: True
        node.stream_since = time.monotonic() - 2.0
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.ENTERING)
        node._request_mode.assert_called_once_with("OFFBOARD")

    def test_manual_takeover_waits_for_ranges_after_offboard_confirmation(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.phase = Manager.ENTERING
        node.phase_since = time.monotonic()
        node.selected_mode = OperatorCommand.MODE_WALL
        node.flight = State(mode="OFFBOARD")
        node.flight_at = now
        node.state_timeout_s = 0.5
        node.cancel_pub = mock.Mock()
        node.wall = None
        node.enter_timeout_s = 6.
        node._preflight_ready = mock.Mock(return_value=True)
        node._stream_ready = mock.Mock(return_value=True)
        node._current_offboard_request = mock.Mock(return_value=True)
        node.ignore_range_loss_during_attachment = True
        node._requested_mode = lambda _: OperatorCommand.MODE_WALL
        node._manual_wall_selected = lambda: True
        node._manual_wall_ready = lambda _: False
        node.prestream_pub = mock.Mock()
        node.phase_pub = mock.Mock()
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.ENTERING)
        self.assertIn("接管等待", node.phase_reason)
        node._manual_wall_ready = lambda _: True
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.ACTIVE)
        node.phase = Manager.ENTERING
        node.ignore_range_loss_during_attachment = False
        node._manual_wall_ready = lambda _: False
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.CANCELING_ENTRY)

    def _waiting_offboard_fixture(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.phase = Manager.WAITING_WALL
        node.phase_since = time.monotonic()
        node.selected_mode = OperatorCommand.MODE_WALL
        node.flight = State(mode="OFFBOARD")
        node.flight_at = now
        node.state_timeout_s = node.status_timeout_s = 0.5
        node.enter_timeout_s = 6.0
        node.takeover_attempt = 2
        node.offboard_request = (2, OperatorCommand.MODE_WALL, time.monotonic())
        node.wall = WallPerchStatus(state=WallPerchStatus.STATE_FLIP_TO_WALL,
                                   candidate_valid=True, fault_detected=False)
        node.wall_at = now
        node._requested_mode = lambda _: OperatorCommand.MODE_WALL
        node._preflight_ready = lambda *_: True
        node._preflight_failure_reason = lambda *_: None
        node._stream_ready = lambda _: True
        node.prestream_pub = mock.Mock()
        node.phase_pub = mock.Mock()
        node.cancel_pub = mock.Mock()
        return node, now

    def test_waiting_wall_synchronizes_this_attempt_without_cancel(self):
        node, now = self._waiting_offboard_fixture()
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.ACTIVE)
        node.cancel_pub.publish.assert_not_called()

    def test_waiting_offboard_without_fresh_flip_waits_in_entering(self):
        for case in ("no_status", "stale", "invalid", "fault", "idle"):
            node, now = self._waiting_offboard_fixture()
            if case == "no_status": node.wall = None
            elif case == "stale": node.wall_at = rospy.Time.from_sec(9.0)
            elif case == "invalid": node.wall.candidate_valid = False
            elif case == "fault": node.wall.fault_detected = True
            else: node.wall.state = WallPerchStatus.STATE_IDLE
            with mock.patch.object(rospy.Time, "now", return_value=now):
                node._on_timer(None)
            self.assertEqual(node.phase, Manager.ENTERING, case)
            node.cancel_pub.publish.assert_not_called()

    def test_waiting_rejects_unrelated_or_unsafe_offboard(self):
        for case in ("none", "previous", "other_mode", "expired", "future", "flight_stale", "preflight", "stream"):
            node, now = self._waiting_offboard_fixture()
            if case == "none": node.offboard_request = None
            elif case == "previous": node.offboard_request = (1, OperatorCommand.MODE_WALL, time.monotonic())
            elif case == "other_mode": node.offboard_request = (2, OperatorCommand.MODE_CEILING, time.monotonic())
            elif case == "expired": node.offboard_request = (2, OperatorCommand.MODE_WALL, time.monotonic()-7)
            elif case == "future": node.offboard_request = (2, OperatorCommand.MODE_WALL, time.monotonic()+7)
            elif case == "flight_stale": node.flight_at = rospy.Time.from_sec(9.0)
            elif case == "preflight": node._preflight_ready = lambda *_: False
            else: node._stream_ready = lambda _: False
            with mock.patch.object(rospy.Time, "now", return_value=now):
                node._on_timer(None)
            self.assertEqual(node.phase, Manager.LOCKED if case == "flight_stale" else Manager.CANCELING_ENTRY, case)
            node.cancel_pub.publish.assert_called_once()
            self.assertIsNone(node.offboard_request)

    def test_canceled_wait_does_not_synchronize_offboard(self):
        node, now = self._waiting_offboard_fixture()
        node.manual_wait_canceled = True
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.LOCKED)
        self.assertIsNone(node.offboard_request)
        node.cancel_pub.publish.assert_called_once()

    def test_mode_request_records_attempt_and_blocks_old_worker(self):
        node, now = self._waiting_offboard_fixture()
        node.phase = Manager.ENTERING
        node._manual_wall_selected = lambda: True
        node._manual_wall_ready = lambda _: True
        node.set_mode = mock.Mock(return_value=mock.Mock(mode_sent=True))
        node.offboard_request = None
        with mock.patch.object(rospy, "wait_for_service"), mock.patch.object(rospy.Time, "now", return_value=now):
            node._send_mode_request("OFFBOARD", attempt=1)
            node.set_mode.assert_not_called()
            self.assertIsNone(node.offboard_request)
            node._send_mode_request("OFFBOARD", attempt=2)
        node.set_mode.assert_called_once()
        self.assertEqual(node.offboard_request[:2], (2, OperatorCommand.MODE_WALL))

    def test_approach_limit_abort_stops_waiting_and_returns_offboard(self):
        for phase, flight_mode, expected in (
            (Manager.WAITING_WALL, "ALTCTL", Manager.LOCKED),
            (Manager.ENTERING, "ALTCTL", Manager.CANCELING_ENTRY),
            (Manager.ENTERING, "OFFBOARD", Manager.CANCELING_ENTRY),
        ):
            node, now = self._waiting_offboard_fixture()
            node.phase = phase
            node.flight.mode = flight_mode
            node.wall.state = WallPerchStatus.STATE_ABORT
            node.wall.reason = "approach front distance 1.200 m exceeds 1.000 m"
            with mock.patch.object(rospy.Time, "now", return_value=now):
                node._on_timer(None)
            self.assertEqual(node.phase, expected)
            node.cancel_pub.publish.assert_called_once()

    def _roll_yaw_fixture(self):
        node = OutputMux.__new__(OutputMux)
        now = rospy.Time.from_sec(10.0)
        node.rc_assist_enabled = node.rc_roll_yaw_enabled = True
        node.rc_calibration = {"roll": (0,1000,1500,2000,False), "yaw": (3,1000,1500,2000,False)}
        node.rc_deadzone = .06
        node.rc_channels = (2000,1500,1500,2000)
        node.rc_at = node.rc_trim_at = now
        node.rc_timeout_s = .2
        node.rc_roll_rad = node.rc_pitch_rad = node.rc_yaw_rad = 0.0
        node.rc_free_max_angle_deg = 5.0
        node.rc_contact_max_angle_deg = 2.0
        node.rc_wall_detach_roll_deg = 3.0
        node.rc_yaw_max_deg = 15.0
        node.rc_yaw_contact_deg = 3.0
        node.rc_yaw_detach_deg = 5.0
        node.rc_slew_deg_s = 10.0
        node.rc_yaw_slew_deg_s = 20.0
        node.rc_age_publisher = node.rc_preview_publisher = mock.Mock()
        node.flight_state = State(mode="OFFBOARD")
        node.flight_state_at = node.wall_status_at = now
        node.state_timeout_s = node.status_timeout_s = .5
        node.wall_status = WallPerchStatus(state=WallPerchStatus.STATE_RECOVER)
        return node, now

    def test_roll_yaw_limits_all_wall_phases_and_keeps_pitch(self):
        for state, limits in ((WallPerchStatus.STATE_RECOVER,(5,15)),
                              (WallPerchStatus.STATE_ABORT,(5,15)),
                              (WallPerchStatus.STATE_EXIT,(5,15)),
                              (WallPerchStatus.STATE_FLIP_TO_WALL,(0,0)),
                              (WallPerchStatus.STATE_WALL_CAPTURE,(0,0)),
                              (WallPerchStatus.STATE_WALL_HOLD,(0,0)),
                              (WallPerchStatus.STATE_DETACH_ROTATE,(0,0))):
            node, now = self._roll_yaw_fixture()
            node.wall_status.state = state
            base = Quaternion(y=math.sin(math.radians(30)/2), w=math.cos(math.radians(30)/2))
            for i in range(1,21):
                t=rospy.Time.from_sec(10+i*.1)
                node.rc_at=node.flight_state_at=node.wall_status_at=t
                out=node._assisted_attitude(t,base,AttachmentControlCandidate.SOURCE_WALL,
                    wall_control_fixture(state,t))
            roll,pitch,yaw=map(math.degrees,euler_from_quaternion((out.x,out.y,out.z,out.w)))
            self.assertAlmostEqual(roll,limits[0],places=4)
            self.assertAlmostEqual(pitch,30,places=4)
            self.assertAlmostEqual(yaw,-limits[1],places=4)
            self.assertEqual(node.rc_pitch_rad,0)

    def test_rc_calibration_retries_cold_mavros_parameter_cache(self):
        node,now=self._roll_yaw_fixture()
        calls=[]
        def get(param_id):
            calls.append(param_id)
            if len(calls)==1:return mock.Mock(success=False)
            value=(1 if param_id=="RC_MAP_ROLL" else 4 if param_id=="RC_MAP_YAW"
                   else 1000 if param_id.endswith("MIN") else 1500 if param_id.endswith("TRIM")
                   else 2000 if param_id.endswith("MAX") else 1)
            return mock.Mock(success=True,value=mock.Mock(integer=value,real=value))
        with mock.patch.object(rospy,"wait_for_service"),mock.patch.object(rospy,"ServiceProxy",return_value=get),mock.patch.object(rospy,"sleep") as sleep,mock.patch.object(rospy,"logwarn_throttle"):
            node._load_rc_calibration()
        sleep.assert_called_once_with(.25)
        self.assertEqual(calls[:2],["RC_MAP_ROLL","RC_MAP_ROLL"])

    def test_roll_yaw_phase_boundary_clears_previous_trim(self):
        node,now=self._roll_yaw_fixture()
        node.wall_status.state=WallPerchStatus.STATE_FLIP_TO_WALL
        node.rc_roll_rad=math.radians(5);node.rc_yaw_rad=math.radians(15)
        base=Quaternion(y=math.sin(math.radians(85)/2),w=math.cos(math.radians(85)/2))
        out=node._assisted_attitude(rospy.Time.from_sec(10.1),base,AttachmentControlCandidate.SOURCE_WALL,
            wall_control_fixture(WallPerchStatus.STATE_FLIP_TO_WALL,rospy.Time.from_sec(10.1)))
        self.assertAlmostEqual(out.y,base.y);self.assertAlmostEqual(out.w,base.w)
        self.assertEqual(node.rc_roll_rad,0);self.assertEqual(node.rc_yaw_rad,0)

    def test_roll_yaw_slew_lost_rc_and_altctl_reset(self):
        node,now=self._roll_yaw_fixture()
        t=rospy.Time.from_sec(10.1)
        out=node._assisted_attitude(t,Quaternion(w=1),AttachmentControlCandidate.SOURCE_WALL,
            wall_control_fixture(WallPerchStatus.STATE_RECOVER,t))
        self.assertAlmostEqual(math.degrees(node.rc_roll_rad),1)
        self.assertAlmostEqual(math.degrees(node.rc_yaw_rad),-2)
        node.rc_channels=(1500,9999,9999,1500)  # Pitch/throttle are not assist inputs.
        node.rc_at=t
        self.assertTrue(node._rc_ready(t))
        node.rc_at=rospy.Time.from_sec(9)
        with mock.patch.object(rospy,"logwarn_throttle"):
            node._assisted_attitude(rospy.Time.from_sec(10.2),Quaternion(w=1),AttachmentControlCandidate.SOURCE_WALL,
                wall_control_fixture(WallPerchStatus.STATE_RECOVER,rospy.Time.from_sec(10.2)))
        self.assertAlmostEqual(node.rc_roll_rad,0)
        self.assertAlmostEqual(node.rc_yaw_rad,0)
        node.rc_roll_rad=.1;node.rc_yaw_rad=.2;node.flight_state.mode="ALTCTL"
        out=node._assisted_attitude(t,Quaternion(w=1),AttachmentControlCandidate.SOURCE_WALL,
            wall_control_fixture(WallPerchStatus.STATE_RECOVER,t))
        self.assertEqual(out,Quaternion(w=1))
        self.assertEqual(node.rc_roll_rad,0);self.assertEqual(node.rc_yaw_rad,0)

    def test_roll_yaw_calibration_uses_actual_px4_channels_and_direction(self):
        node,now=self._roll_yaw_fixture()
        values={"RC_MAP_ROLL":2,"RC_MAP_YAW":4}
        for channel in (2,4):
            values.update({"RC%d_MIN"%channel:1100,"RC%d_TRIM"%channel:1510,
                           "RC%d_MAX"%channel:1900,"RC%d_REV"%channel:-1})
        def get(param_id):
            return mock.Mock(success=param_id in values,
                             value=mock.Mock(integer=values.get(param_id,0),real=values.get(param_id,0)))
        with mock.patch.object(rospy,"wait_for_service"),mock.patch.object(rospy,"ServiceProxy",return_value=get):
            node._load_rc_calibration()
        self.assertEqual(node.rc_calibration["roll"],(1,1100,1510,1900,True))
        node.rc_channels=(1500,1900,1500,1900)
        self.assertEqual(node._calibrated_rc_fraction("roll"),-1)
        self.assertEqual(node._calibrated_rc_fraction("yaw"),1)
        values["RC_MAP_YAW"]=0
        with mock.patch.object(rospy,"wait_for_service"),mock.patch.object(rospy,"ServiceProxy",return_value=get):
            with self.assertRaises(ValueError):node._load_rc_calibration()

    def test_prestream_requires_attitude_output(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.output_mode = 1
        node.output_at = now
        node.output_timeout_s = 0.3
        node.owner = 0
        node.owner_at = now
        self.assertFalse(node._stream_ready(now))
        node.output_mode = 2
        self.assertTrue(node._stream_ready(now))

    def test_ceiling_return_requires_detach_and_recovery(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.selected_mode = OperatorCommand.MODE_CEILING
        node.owner = 1
        node.owner_at = now
        node.owner_seen = False
        node.output_timeout_s = 0.3
        node.status_timeout_s = 0.5
        node.detach_seen = node.recovery_seen = node.completed = False
        node.fault_seen = False
        node.ceiling_at = now
        node.ceiling = CeilingAttachmentStatus(
            state=CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
        )
        node._observe_completion(now)
        self.assertTrue(node.owner_seen)
        self.assertFalse(node.completed)

        for state in (
            CeilingAttachmentStatus.STATE_DETACH,
            CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
            CeilingAttachmentStatus.STATE_NORMAL_FLIGHT,
        ):
            node.ceiling.state = state
            node._observe_completion(now)
        self.assertTrue(node.completed)

    def test_wall_abort_does_not_count_as_completed_detach(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.selected_mode = OperatorCommand.MODE_WALL
        node.owner = 2
        node.owner_at = now
        node.owner_seen = False
        node.output_timeout_s = 0.3
        node.status_timeout_s = 0.5
        node.detach_seen = node.recovery_seen = node.completed = False
        node.fault_seen = False
        node.wall_at = now
        node.wall = WallPerchStatus(state=WallPerchStatus.STATE_ABORT)
        node._observe_completion(now)
        node.wall.state = WallPerchStatus.STATE_EXIT
        node._observe_completion(now)
        self.assertFalse(node.completed)

    def test_wall_fault_after_detach_blocks_automatic_return(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.selected_mode = OperatorCommand.MODE_WALL
        node.owner = 2
        node.owner_at = now
        node.owner_seen = False
        node.output_timeout_s = 0.3
        node.status_timeout_s = 0.5
        node.detach_seen = node.recovery_seen = node.completed = False
        node.fault_seen = False
        node.wall_at = now
        node.wall = WallPerchStatus(state=WallPerchStatus.STATE_DETACH_ROTATE)
        for state in (
            WallPerchStatus.STATE_DETACH_ROTATE,
            WallPerchStatus.STATE_RECOVER,
            WallPerchStatus.STATE_ABORT,
            WallPerchStatus.STATE_EXIT,
        ):
            node.wall.state = state
            node._observe_completion(now)
        self.assertTrue(node.fault_seen)
        self.assertFalse(node.completed)

    def test_wall_successful_detach_requires_recovery_before_release(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.selected_mode = OperatorCommand.MODE_WALL
        node.owner = 2
        node.owner_at = now
        node.owner_seen = False
        node.output_timeout_s = 0.3
        node.status_timeout_s = 0.5
        node.detach_seen = node.recovery_seen = node.completed = False
        node.fault_seen = False
        node.wall_at = now
        node.wall = WallPerchStatus(state=WallPerchStatus.STATE_WALL_HOLD)
        for state in (WallPerchStatus.STATE_WALL_HOLD, WallPerchStatus.STATE_DETACH_ROTATE, WallPerchStatus.STATE_RECOVER):
            node.wall.state = state
            node._observe_completion(now)
            self.assertFalse(node.completed)
        node.wall.state = WallPerchStatus.STATE_EXIT
        node.wall.detach_completed = node.wall.recovery_completed = True
        node._observe_completion(now)
        self.assertTrue(node.completed)
        self.assertTrue(node.owner_seen)

    def test_posctl_is_requested_only_after_completion_and_owner_release(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.phase = Manager.ACTIVE
        node.phase_since = time.monotonic()
        node.selected_mode = OperatorCommand.MODE_CEILING
        node.flight = State(mode="OFFBOARD")
        node.flight_at = now
        node.state_timeout_s = 0.5
        node.owner_seen = True
        node.owner = 1
        node.owner_at = now
        node.output_timeout_s = 0.3
        node.output_mode = 2
        node.output_at = now
        node.estimator = EstimatorStatus(
            attitude_status_flag=True, velocity_vert_status_flag=True, velocity_horiz_status_flag=True, pos_horiz_rel_status_flag=True
        )
        node.estimator_at = now
        node.estimator_timeout_s = 0.5
        node.odom = Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom_at = now
        node.odom_timeout_s = 0.5
        node.handover_max_tilt_deg = 10.0
        node.handover_max_vertical_speed_mps = 0.2
        node.odom_twist_in_child_frame = True
        node.require_rc_center = False
        node.rc_channels = (1500, 1500, 1500, 1500)
        node.rc_at = now
        node.rc_timeout_s = .2
        node.completed = False
        node._requested_mode = lambda _now: OperatorCommand.MODE_CEILING
        node._observe_completion = lambda _now: setattr(node, "completed", True)
        node._request_mode = mock.Mock()
        node.prestream_pub = mock.Mock()
        node.phase_pub = mock.Mock()

        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.ACTIVE)
        node._request_mode.assert_not_called()

        node.owner = 0
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.RETURNING)
        node._request_mode.assert_called_once_with("POSCTL")

    def test_owner_timeout_reports_wall_decision_reason(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.selected_mode = OperatorCommand.MODE_WALL
        node.wall = WallPerchStatus(
            reason="minimum altitude gate blocked: z=-1.960 m <= -0.100 m"
        )
        node.wall_at = now
        node.status_timeout_s = 0.5

        self.assertEqual(
            node._acquire_failure_reason(now),
            "minimum altitude gate blocked: z=-1.960 m <= -0.100 m",
        )

    def test_terminal_kill_clears_request_before_fcu_disarms(self):
        from px4_wall_ceiling_control.adapters.kill_switch import KillSwitchConfig
        node = Terminal.__new__(Terminal)
        node.mode = OperatorCommand.MODE_WALL
        node.detaching = node.detach_pulse = node.was_offboard = True
        node.kill_guard_enabled = True
        node.kill_config = KillSwitchConfig(7)
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10.0)):
            node._on_rc(RCIn(channels=[1500]*6+[1999]))
        self.assertEqual(node.mode, OperatorCommand.MODE_STANDBY)
        self.assertFalse(node.detaching or node.detach_pulse or node.was_offboard)
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10.1)):
            node._on_rc(RCIn(channels=[1500]*6+[999]))
        self.assertEqual(node.mode, OperatorCommand.MODE_STANDBY)

    def test_kill_guard_blocks_mode_request_even_after_service_wait(self):
        node = Manager.__new__(Manager)
        node.kill_guard_enabled = True
        node.kill_status_at = rospy.Time.from_sec(10.0)
        node.kill_status = {"valid": True, "active": True}
        node.set_mode = mock.Mock()
        node.request_in_flight = True
        with mock.patch.object(rospy, "wait_for_service"), mock.patch.object(rospy.Time, "now", return_value=node.kill_status_at):
            node._send_mode_request("OFFBOARD")
        node.set_mode.assert_not_called()

    def test_active_kill_keeps_locked_phase_feedback_fresh(self):
        node = Manager.__new__(Manager)
        node.kill_guard_enabled = True
        node.kill_status_at = rospy.Time.from_sec(10.0)
        node.kill_status = {"valid": True, "active": True}
        node.phase = Manager.LOCKED
        node.phase_reason = "kill switch active or unavailable"
        node._reset = mock.Mock()
        node.prestream_pub = mock.Mock()
        node.phase_pub = mock.Mock()
        node.phase_reason_pub = mock.Mock()
        with mock.patch.object(rospy.Time, "now", return_value=node.kill_status_at):
            node._on_timer(None)
        self.assertFalse(node.prestream_pub.publish.call_args[0][0].data)
        self.assertEqual(node.phase_pub.publish.call_args[0][0].data, Manager.LOCKED)
        self.assertEqual(node.phase_reason_pub.publish.call_args[0][0].data, node.phase_reason)

    def test_abort_recovery_can_handover_while_owner_is_retained(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.selected_mode = OperatorCommand.MODE_WALL
        node.owner = 2
        node.owner_at = now
        node.owner_seen = False
        node.output_timeout_s = 0.3
        node.status_timeout_s = 0.5
        node.completed = False
        node.fault_seen = True
        node.detach_seen = node.recovery_seen = True
        node.wall_at = now
        node.wall = WallPerchStatus(state=WallPerchStatus.STATE_EXIT, fault_detected=True, fault_recovery_completed=True)
        node._observe_completion(now)
        self.assertFalse(node.completed)
        self.assertTrue(node.wall_fault_return_ready)
        self.assertTrue(node.owner_seen)
        self.assertTrue(node.fault_seen)

    def test_return_waits_in_offboard_when_handover_not_ready(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.phase = Manager.RETURNING
        node.flight = State(mode="OFFBOARD", armed=True, connected=True)
        node.flight_at = now
        node.state_timeout_s = 1.5
        node._requested_mode = lambda _now: OperatorCommand.MODE_WALL
        node._handover_ready = lambda _now: False
        node._reset = mock.Mock()
        node._request_mode = mock.Mock()
        node.prestream_pub = mock.Mock()
        node.phase_pub = mock.Mock()
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.RETURNING)
        self.assertTrue(node.prestream_pub.publish.call_args[0][0].data)
        node._reset.assert_not_called()
        node._request_mode.assert_not_called()

    def test_wall_retained_owner_handover_waits_for_altctl_confirmation(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.phase = Manager.ACTIVE
        node.phase_since = time.monotonic()
        node.selected_mode = OperatorCommand.MODE_WALL
        node.flight = State(mode="OFFBOARD", armed=True, connected=True)
        node.flight_at = node.owner_at = now
        node.state_timeout_s = 1.5
        node.output_timeout_s = 0.3
        node.owner = 2
        node.owner_seen = node.completed = True
        node._requested_mode = lambda _now: OperatorCommand.MODE_WALL
        node._observe_completion = lambda _now: None
        node._handover_ready = lambda _now: True
        node._request_mode = mock.Mock()
        node.prestream_pub = mock.Mock()
        node.phase_pub = mock.Mock()
        with mock.patch.object(rospy.Time, "now", return_value=now):
            node._on_timer(None)
            self.assertEqual(node.phase, Manager.RETURNING)
            node._on_timer(None)
            node._request_mode.assert_called_once_with("ALTCTL")
            self.assertEqual(node.phase, Manager.RETURNING)
            self.assertTrue(node.prestream_pub.publish.call_args[0][0].data)
            node.flight.mode = "ALTCTL"
            node._on_timer(None)
        self.assertEqual(node.phase, Manager.LOCKED)
        self.assertFalse(node.prestream_pub.publish.call_args[0][0].data)

    def test_bench_handover_allows_velocity_drift_but_rejects_motion(self):
        node = Manager.__new__(Manager)
        now = rospy.Time.from_sec(10.0)
        node.output_mode = 2
        node.output_at = node.estimator_at = node.odom_at = now
        node.output_timeout_s = node.estimator_timeout_s = node.odom_timeout_s = 0.5
        node.estimator = EstimatorStatus(attitude_status_flag=True, velocity_vert_status_flag=True)
        node.odom = Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom.twist.twist.linear.z = 2.0
        node.odom_twist_in_child_frame = False
        node.handover_max_tilt_deg = 10.0
        node.handover_max_vertical_speed_mps = 0.2
        node.require_rc_center = False
        node.rc_channels = (1500, 1500, 1500, 1500)
        node.rc_at = now
        node.rc_timeout_s = .2
        self.assertFalse(node._handover_ready(now))
        node.wall_bench_wait_for_ranges = True
        self.assertTrue(node._handover_ready(now))
        node.odom.twist.twist.angular.y = 0.5
        self.assertFalse(node._handover_ready(now))

    def test_terminal_detach_keeps_mode_until_posctl(self):
        node = Terminal.__new__(Terminal)
        node.mode = OperatorCommand.MODE_STANDBY
        node.detaching = False
        node.was_offboard = False
        node.flight_mode = "POSCTL"
        node.command_pub = mock.Mock()
        node.ceiling_detach_pub = mock.Mock()
        node.wall_detach_pub = mock.Mock()

        with mock.patch.object(rospy, "set_param"):
            node._on_command(String(data="ceiling"))
        node._on_command(String(data="detach"))
        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time.from_sec(10.0)
        ):
            node._on_timer(None)
        self.assertEqual(node.mode, OperatorCommand.MODE_CEILING)
        self.assertTrue(node.detaching)
        node.ceiling_detach_pub.publish.assert_called_once()
        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time.from_sec(10.1)
        ):
            node._on_timer(None)
        node.ceiling_detach_pub.publish.assert_called_once()
        node._on_command(String(data="detach"))
        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time.from_sec(10.2)
        ):
            node._on_timer(None)
        self.assertEqual(node.ceiling_detach_pub.publish.call_count, 2)
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10.3)):
            node._on_state(State(connected=True, mode="OFFBOARD"))
            with mock.patch.object(rospy, "set_param"):
                node._on_state(State(connected=True, mode="POSCTL"))
        self.assertEqual(node.mode, OperatorCommand.MODE_STANDBY)


class PreflipBiasTest(unittest.TestCase):
    def node(self):
        node = OutputMux.__new__(OutputMux)
        now = rospy.Time.from_sec(10)
        node.preflip_right_roll_deg = 1.0
        node.preflip_roll_slew_deg_s = 5.0
        node.preflip_bias_rad = 0.0
        node.preflip_bias_at = rospy.Time.from_sec(9.9)
        node.preflip_bias_publisher = mock.Mock()
        node.flight_state = State(connected=True, armed=True, mode="OFFBOARD")
        node.flight_state_at = node.wall_status_at = node.operator_at = now
        node.state_timeout_s = 1.5
        node.status_timeout_s = .3
        node.operator_timeout_s = .5
        node.wall_status = WallPerchStatus(state=WallPerchStatus.STATE_IDLE)
        node.operator = OperatorCommand(enable_control=True, mode=OperatorCommand.MODE_WALL)
        node.rc_free_max_angle_deg = 5.0
        return node, now

    def test_right_thrust_direction_and_ramp(self):
        from px4_wall_ceiling_control.domain.geometry import rotate_vector
        node, now = self.node()
        q = Quaternion(w=1)
        first = node._preflip_bias_attitude(now, q, AttachmentControlCandidate.SOURCE_NONE)
        self.assertAlmostEqual(math.degrees(euler_from_quaternion((first.x,first.y,first.z,first.w))[0]), .5)
        later = node._preflip_bias_attitude(rospy.Time.from_sec(10.1), q, AttachmentControlCandidate.SOURCE_NONE)
        v = rotate_vector((later.x,later.y,later.z,later.w), (0,0,1))
        self.assertLess(v[1], 0)  # FLU -Y is aircraft right.
        self.assertAlmostEqual(math.degrees(math.asin(-v[1])), 1)

    def test_only_preflip_phases_and_immediate_clear(self):
        node, now = self.node()
        for state in range(WallPerchStatus.STATE_ABORT+1):
            node.wall_status.state = state
            node.preflip_bias_rad = math.radians(1)
            q = node._preflip_bias_attitude(now, Quaternion(w=1), AttachmentControlCandidate.SOURCE_WALL,
                                           wall_control_fixture(state, now))
            roll = euler_from_quaternion((q.x,q.y,q.z,q.w))[0]
            if state <= WallPerchStatus.STATE_SLOW_APPROACH:
                self.assertAlmostEqual(roll, math.radians(1))
            else:
                self.assertEqual(roll, 0)
                self.assertEqual(node.preflip_bias_rad, 0)

    def test_combined_roll_cap_and_preserves_yaw(self):
        from px4_wall_ceiling_control.domain.geometry import quaternion_multiply
        node, now = self.node()
        node.preflip_bias_rad = math.radians(1)
        q = quaternion_multiply((0,0,math.sin(.7),math.cos(.7)),
                                (math.sin(math.radians(5)/2),0,0,math.cos(math.radians(5)/2)))
        out = node._preflip_bias_attitude(now, Quaternion(x=q[0],y=q[1],z=q[2],w=q[3]), AttachmentControlCandidate.SOURCE_NONE)
        roll,pitch,yaw = euler_from_quaternion((out.x,out.y,out.z,out.w))
        self.assertAlmostEqual(math.degrees(roll), 5)
        self.assertAlmostEqual(pitch, 0)
        self.assertAlmostEqual(yaw, 1.4)

    def test_invalid_feedback_altctl_ceiling_and_released_request_clear(self):
        for case in ("stale_status", "stale_fcu", "ALTCTL", "ceiling", "released", "disarmed"):
            node, now = self.node()
            node.preflip_bias_rad = math.radians(1)
            source = AttachmentControlCandidate.SOURCE_NONE
            if case == "stale_status": node.wall_status_at = rospy.Time.from_sec(9)
            if case == "stale_fcu": node.flight_state_at = rospy.Time.from_sec(8)
            if case == "ALTCTL": node.flight_state.mode = "ALTCTL"
            if case == "ceiling": source = AttachmentControlCandidate.SOURCE_CEILING
            if case == "released": node.operator.enable_control = False
            if case == "disarmed": node.flight_state.armed = False
            q = node._preflip_bias_attitude(now, Quaternion(w=1), source)
            self.assertEqual(q.x, 0)
            self.assertEqual(node.preflip_bias_rad, 0)


if __name__ == "__main__":
    unittest.main()
