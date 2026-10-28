"""Full HOLD cycles must not wait for the ROS parameter server after startup."""
from dataclasses import FrozenInstanceError, fields
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from mavros_msgs.msg import OpticalFlowRad
from std_msgs.msg import Bool

from px4_wall_ceiling_control.domain.range_height_feedback import RangeHeightConfig
from px4_wall_ceiling_control.msg import WallPerchStatus as W
from test_attachment_safety import WallDecision
import test_detach_state_machine as fixtures


class HoldConfigLatencyTests(unittest.TestCase):
    def setUp(self):
        self.seconds = 10.
        self.params = {'~detach_range_height_enabled': True,
                       '~detach_range_min_distance_m': .02,
                       '~detach_range_max_distance_m': 6.,
                       '~detach_range_geometry_confirmed': True}
        self.fx = fixtures.DetachStateMachineTests()
        for name in ('Service', 'loginfo', 'logwarn'):
            patch = mock.patch.object(rospy, name)
            patch.start()
            self.addCleanup(patch.stop)
        patch = mock.patch.object(rospy.Time, 'now', side_effect=lambda: fixtures.at(self.seconds))
        patch.start()
        self.addCleanup(patch.stop)
        self.n = self.new_node()

    def new_node(self, **extra):
        params = dict(self.params)
        params.update(extra)
        node = self.fx.node(**params)
        # The shared fixture's Publisher mock otherwise returns one object for
        # both topics, making the candidate overwrite the captured status.
        node.status_publisher = mock.Mock()
        node.control_candidate_publisher = mock.Mock()
        return node

    def refresh(self, seconds, rate=0., tilt=90.):
        self.seconds = seconds
        self.fx.feedback(self.n, seconds, tilt=tilt, rate=rate)
        self.fx.output(self.n, seconds, tilt=tilt)
        self.n.flight_state.header.stamp = fixtures.at(seconds)

    def cycle(self, seconds, **feedback):
        self.refresh(seconds, **feedback)
        self.n._on_timer(None)
        return self.n.status_publisher.publish.call_args[0][0]

    def deny_parameters(self):
        return mock.patch.object(rospy, 'get_param', side_effect=AssertionError(
            'control path accessed the parameter server'))

    def test_hold_without_request_publishes_when_ready_or_blocked_without_parameter_io(self):
        for installation_ready, rate, ready in ((True, 0., True), (True, 30., True),
                                                (False, 0., False)):
            with self.subTest(installation_ready=installation_ready, rate=rate):
                self.n = self.new_node(**{'~detach_range_geometry_confirmed': installation_ready})
                with self.deny_parameters() as reads:
                    for i in range(40):
                        status = self.cycle(10.02+i*.02, rate=rate)
                        candidate = self.n.control_candidate_publisher.publish.call_args[0][0]
                        self.assertEqual(self.n.state, W.STATE_WALL_HOLD)
                        self.assertEqual(status.detach_ready, ready)
                        self.assertFalse(status.detach_pending)
                        self.assertEqual(bool(status.detach_block_reason), not ready)
                        self.assertTrue(candidate.valid)
                        self.assertEqual(candidate.header.stamp, fixtures.at(self.seconds))
                    reads.assert_not_called()
                self.assertEqual(self.n.control_candidate_publisher.publish.call_count, 40)
                self.assertFalse(self.n.detach_vertical_active)
                self.assertEqual(self.n.detach_range_episode, 0)

    def test_status_reuses_readiness_only_within_one_cycle(self):
        with self.deny_parameters(), mock.patch.object(
                self.n, '_detach_start_ready', wraps=self.n._detach_start_ready) as ready:
            self.assertTrue(self.cycle(10.02).detach_ready)
            self.assertEqual(ready.call_count, 1)
            self.assertTrue(self.cycle(10.04, rate=30.).detach_ready)
            self.assertEqual(ready.call_count, 2)
            self.assertTrue(self.cycle(10.06).detach_ready)
            for i in range(1, 31):
                status = self.cycle(10.06+i*.02)
                self.assertEqual(ready.call_count, 3+i)
            self.assertTrue(status.detach_ready)
            self.assertEqual(self.n.state, W.STATE_WALL_HOLD)

    def test_valid_request_and_first_range_need_no_parameter_io(self):
        config = self.n._range_config()
        with self.deny_parameters() as reads:
            self.refresh(10.)
            self.n._on_detach(Bool(data=True))
            self.n._on_timer(None)
            self.assertEqual(self.n.state, W.STATE_DETACH_ROTATE)
            self.assertIs(self.n.detach_range.config, config)
            for seconds in (10.25, 10.5):
                self.refresh(seconds, tilt=0.)
                frame = OpticalFlowRad(distance=1.24)
                frame.header.stamp = fixtures.at(seconds)
                self.n._on_detach_range(frame)
            self.assertTrue(self.n.detach_range.locked)
            self.assertAlmostEqual(self.n.detach_range.lock_height, 1.24)
            self.assertTrue(self.n.detach_range.velocity_valid)
            self.fx.enter(self.n, W.STATE_RECOVER, self.seconds)
            self.n._detach_progress = lambda: 1.
            self.n._wall_fusion_samples_ready = lambda now: True
            reads.assert_not_called()

    def test_parameter_changes_require_restart_even_for_later_departures(self):
        config = self.n._range_config()
        changed = dict(self.params, **{'~detach_range_entry_tilt_deg': 8.})
        with mock.patch.object(rospy, 'get_param', side_effect=lambda key, default=None: changed.get(key, default)) as reads:
            for seconds in (10., 11.):
                self.refresh(seconds)
                self.assertTrue(self.n._begin_detach_vertical(
                    fixtures.at(seconds), self.n.horizontal_output))
                self.assertIs(self.n.detach_range.config, config)
                self.assertEqual(self.n.detach_range.config.entry_tilt_deg, 10.)
            reads.assert_not_called()
        restarted = self.new_node(**changed)
        self.assertEqual(restarted._range_config().entry_tilt_deg, 8.)

    def test_startup_normalizes_and_freezes_config_and_rejects_invalid_values(self):
        node = self.new_node(**{'~detach_range_sensor_axis_body': [0., 0., -1.],
                                '~detach_range_reference_offset_body_m': [0., 0., .02]})
        config = node._range_config()
        self.assertIsInstance(config.sensor_axis_body, tuple)
        self.assertIsInstance(config.reference_offset_body_m, tuple)
        self.assertIs(node.detach_range.config, config)
        with self.assertRaises(FrozenInstanceError):
            config.entry_tilt_deg = 2.
        with self.assertRaises(ValueError):
            self.new_node(**{'~detach_range_timeout_s': -1.})

    def test_each_range_parameter_is_read_once_during_startup(self):
        with mock.patch.object(rospy, 'get_param', side_effect=lambda key, default=None: self.params.get(key, default)) as reads, \
             mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), \
             mock.patch.object(rospy, 'Timer'):
            WallDecision()
        for field in fields(RangeHeightConfig):
            key = '~detach_range_'+field.name
            self.assertEqual(sum(call.args[0] == key for call in reads.call_args_list), 1, key)


if __name__ == '__main__':
    unittest.main()
