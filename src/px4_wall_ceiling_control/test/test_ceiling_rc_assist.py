#!/usr/bin/env python3
"""Ceiling roll/pitch assistance and wall compatibility at the output boundary."""
import math
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as Candidate
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as Ceiling
from px4_wall_ceiling_control.msg import OperatorCommand
import test_flight_handover as handover


class CeilingRcAssistTest(unittest.TestCase):
    def fixture(self):
        node, now = handover.FlightHandoverTest()._roll_yaw_fixture()
        node.rc_ceiling_roll_pitch_enabled = True
        node.rc_calibration['pitch'] = (1, 1000, 1500, 2000, False)
        node.ceiling_status = Ceiling(state=Ceiling.STATE_APPROACH)
        node.ceiling_status_at = now
        node.operator = OperatorCommand(enable_control=True, mode=OperatorCommand.MODE_CEILING)
        node.operator_at = now
        node.rc_channels = (2000, 1000, 9999, 9999)
        return node, now

    def tick(self, node, seconds):
        now = rospy.Time.from_sec(seconds)
        node.rc_at = node.flight_state_at = node.ceiling_status_at = node.wall_status_at = now
        node.operator_at = now
        return now

    def test_ceiling_phases_bound_both_axes_and_ignore_yaw(self):
        for state, bound in ((Ceiling.STATE_APPROACH, 5), (Ceiling.STATE_DETACH, 5),
                             (Ceiling.STATE_RECOVERY_HOVER, 5),
                             (Ceiling.STATE_ATTACH_CONTROL, 2), (Ceiling.STATE_SURFACE_HOLD, 2),
                             (Ceiling.STATE_NORMAL_FLIGHT, 0), (Ceiling.STATE_CEILING_ARMED, 0),
                             (Ceiling.STATE_ABORT, 0)):
            with self.subTest(state=state):
                node, _ = self.fixture()
                node.ceiling_status.state = state
                for i in range(1, 11):
                    now = self.tick(node, 10 + i * .1)
                    node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_CEILING)
                self.assertAlmostEqual(math.degrees(node.rc_roll_rad), bound)
                self.assertAlmostEqual(math.degrees(node.rc_pitch_rad), -bound)
                self.assertEqual(node.rc_yaw_rad, 0)
                preview = node.rc_preview_publisher.publish.call_args.args[0]
                self.assertAlmostEqual(preview.vector.y, -bound)
                self.assertEqual(preview.vector.z, 0)

    def test_phase_tightening_and_stale_status_clamp_immediately(self):
        node, now = self.fixture()
        node.rc_roll_rad = math.radians(5)
        node.rc_pitch_rad = math.radians(-5)
        node.ceiling_status.state = Ceiling.STATE_SURFACE_HOLD
        node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_CEILING)
        self.assertAlmostEqual(math.degrees(node.rc_roll_rad), 2)
        self.assertAlmostEqual(math.degrees(node.rc_pitch_rad), -2)
        node.ceiling_status_at = rospy.Time.from_sec(9)
        node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_CEILING)
        self.assertEqual(node.rc_roll_rad, 0)
        self.assertEqual(node.rc_pitch_rad, 0)

    def test_slew_stale_rc_and_offboard_exit_reset(self):
        node, _ = self.fixture()
        now = self.tick(node, 10.1)
        node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_CEILING)
        self.assertAlmostEqual(math.degrees(node.rc_roll_rad), 1)
        self.assertAlmostEqual(math.degrees(node.rc_pitch_rad), -1)
        now = self.tick(node, 10.2)
        node.rc_at = rospy.Time.from_sec(9)
        with mock.patch.object(rospy, 'logwarn_throttle'):
            node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_CEILING)
        self.assertAlmostEqual(node.rc_roll_rad, 0)
        self.assertAlmostEqual(node.rc_pitch_rad, 0)
        for mode in ('ALTCTL', 'POSCTL'):
            node.rc_roll_rad = node.rc_pitch_rad = node.rc_yaw_rad = .1
            node.flight_state.mode = mode
            base = Quaternion(z=.5, w=math.sqrt(.75))
            self.assertEqual(node._assisted_attitude(now, base, Candidate.SOURCE_CEILING), base)
            self.assertEqual((node.rc_roll_rad, node.rc_pitch_rad, node.rc_yaw_rad), (0, 0, 0))

    def test_state_callback_clears_trim_even_without_altctl_prestream(self):
        node, now = self.fixture()
        for mode in ('ALTCTL', 'OFFBOARD'):
            node.rc_roll_rad = node.rc_pitch_rad = node.rc_yaw_rad = .1
            with mock.patch.object(rospy.Time, 'now', return_value=now):
                node._on_state(handover.State(mode=mode))
            self.assertEqual((node.rc_roll_rad, node.rc_pitch_rad, node.rc_yaw_rad), (0, 0, 0))

    def test_ready_uses_selected_axes_including_prestream(self):
        node, now = self.fixture()
        self.assertTrue(node._rc_ready(now))  # Ceiling selected, invalid yaw ignored.
        self.assertTrue(node._rc_ready(now, Candidate.SOURCE_CEILING))
        self.assertFalse(node._rc_ready(now, Candidate.SOURCE_WALL))
        node.rc_channels = (2000, 9999, 9999, 2000)
        self.assertFalse(node._rc_ready(now))
        self.assertTrue(node._rc_ready(now, Candidate.SOURCE_WALL))
        node.operator.mode = OperatorCommand.MODE_WALL
        self.assertTrue(node._rc_ready(now))

    def wall_control(self, node, now):
        # Match the atomic phase metadata required by an owned wall command.
        candidate = Candidate(source=Candidate.SOURCE_WALL, phase_valid=True,
                              control_phase=node.wall_status.state,
                              maneuver_id="ceiling-axis-switch-test",
                              control_sequence=max(1, int(now.to_nsec())))
        candidate.header.stamp = now
        return candidate

    def test_axis_switch_does_not_transfer_old_trim(self):
        node, _ = self.fixture()
        node.rc_channels = (2000, 2000, 1500, 2000)
        for i in range(1, 11):
            now = self.tick(node, 10 + i * .1)
            node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_WALL,
                                    self.wall_control(node, now))
        self.assertNotEqual(node.rc_yaw_rad, 0)
        now = self.tick(node, 11.1)
        node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_CEILING)
        self.assertEqual((node.rc_roll_rad, node.rc_pitch_rad, node.rc_yaw_rad), (0, 0, 0))
        now = self.tick(node, 11.2)
        node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_CEILING)
        self.assertGreater(node.rc_pitch_rad, 0)
        now = self.tick(node, 11.3)
        node._assisted_attitude(now, Quaternion(w=1), Candidate.SOURCE_WALL,
                                    self.wall_control(node, now))
        self.assertEqual((node.rc_roll_rad, node.rc_pitch_rad, node.rc_yaw_rad), (0, 0, 0))

    def test_px4_pitch_mapping_calibration_and_direction(self):
        node, _ = self.fixture()
        values = {'RC_MAP_ROLL': 2, 'RC_MAP_PITCH': 5, 'RC_MAP_YAW': 7}
        for channel in (2, 5, 7):
            for suffix, value in (('MIN', 1100), ('TRIM', 1510), ('MAX', 1900), ('REV', 1)):
                values['RC%d_%s' % (channel, suffix)] = value
        def get(param_id):
            return mock.Mock(success=True, value=mock.Mock(integer=values[param_id], real=values[param_id]))
        with mock.patch.object(rospy, 'wait_for_service'), mock.patch.object(rospy, 'ServiceProxy', return_value=get):
            node._load_rc_calibration()
            self.assertEqual(node.rc_calibration['pitch'], (4, 1100, 1510, 1900, False))
            node.rc_channels = (1500, 1510, 1500, 1500, 1900, 1500, 1510)
            self.assertEqual(node._calibrated_rc_fraction('pitch'), 1)
            values['RC5_REV'] = -1
            node._load_rc_calibration()
            self.assertEqual(node._calibrated_rc_fraction('pitch'), -1)
            values['RC_MAP_PITCH'] = 2
            with self.assertRaises(ValueError):
                node._load_rc_calibration()
            values['RC_MAP_PITCH'] = 0
            with self.assertRaises(ValueError):
                node._load_rc_calibration()

    def test_ceiling_correction_reaches_both_output_kinds_without_changing_thrust(self):
        for kind in (Candidate.KIND_VERTICAL_VELOCITY, Candidate.KIND_ATTITUDE_THRUST):
            with self.subTest(kind=kind):
                node, _ = self.fixture()
                node.odom = Odometry()
                node.odom.pose.pose.orientation.w = 1
                node.odom.twist.twist.linear.z = -.1
                node.odom_timeout_s = .5
                node.odom_twist_in_child_frame = True
                node.hover_thrust = .49
                node.vertical_speed_kp = .15
                node.max_velocity_z_mps = .3
                node.min_thrust, node.max_thrust = .1, .85
                node.local_frame_id = 'map'
                node.attitude_publisher = mock.Mock()
                candidate = Candidate(source=Candidate.SOURCE_CEILING, kind=kind,
                                      velocity_z_enu_mps=.1, attitude_setpoint=Quaternion(w=1), thrust_normalized=.52)
                for i in range(1, 11):
                    now = self.tick(node, 10 + i * .1)
                    node.odom_at = now
                    result = node._publish_candidate(now, candidate)
                self.assertEqual(result, node.OUTPUT_ATTITUDE_THRUST)
                target = node.attitude_publisher.publish.call_args.args[0]
                self.assertAlmostEqual(target.thrust, .52)  # .49 + .15 * (.1 - (-.1))
                self.assertGreater(abs(target.orientation.x), .01)
                self.assertGreater(abs(target.orientation.y), .01)
                self.assertEqual(node.rc_yaw_rad, 0)


if __name__ == '__main__':
    unittest.main()
