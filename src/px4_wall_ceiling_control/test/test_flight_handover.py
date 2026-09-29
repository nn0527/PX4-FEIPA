#!/usr/bin/env python3

import importlib.util
import math
import os
import time
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import EstimatorStatus, State
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
            attitude_status_flag=True, velocity_vert_status_flag=True
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
            velocity_vert_status_flag=True,
        )
        node.estimator_at = now
        node.sensor = SensorHealth(ceiling_valid=True, front_valid=False)
        node.sensor_at = now
        node.state_timeout_s = node.odom_timeout_s = 0.5
        node.estimator_timeout_s = node.sensor_timeout_s = 0.5

        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_CEILING))
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.sensor.front_valid = True
        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.sensor_at = rospy.Time.from_sec(9.0)
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))

        node.sensor_at = now
        node.estimator.velocity_horiz_status_flag = False
        self.assertTrue(node._preflight_ready(now, OperatorCommand.MODE_WALL))
        node.estimator.velocity_vert_status_flag = False
        self.assertFalse(node._preflight_ready(now, OperatorCommand.MODE_WALL))
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
        node.flight = State(mode="ALTCTL")
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

    def test_altctl_is_requested_only_after_completion_and_owner_release(self):
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
            attitude_status_flag=True, velocity_vert_status_flag=True
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
        node._request_mode.assert_called_once_with("ALTCTL")

    def test_terminal_detach_keeps_mode_until_altctl(self):
        node = Terminal.__new__(Terminal)
        node.mode = OperatorCommand.MODE_STANDBY
        node.detaching = False
        node.was_offboard = False
        node.flight_mode = "ALTCTL"
        node.command_pub = mock.Mock()
        node.ceiling_detach_pub = mock.Mock()
        node.wall_detach_pub = mock.Mock()

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
        node._on_state(State(mode="OFFBOARD"))
        node._on_state(State(mode="ALTCTL"))
        self.assertEqual(node.mode, OperatorCommand.MODE_STANDBY)


if __name__ == "__main__":
    unittest.main()
