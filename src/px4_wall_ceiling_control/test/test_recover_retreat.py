"""RECOVER behavior with mocked ROS: no flight nodes or control publishing."""
import math
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import OpticalFlowRad
from sensor_msgs.msg import Range
from px4_wall_ceiling_control.domain.geometry import rotate_vector
from px4_wall_ceiling_control.msg import WallPerchStatus as W
import test_positive_recovery as positive_fixtures
from test_detach_state_machine import at


class RetreatTests(unittest.TestCase):
    def setUp(self):
        self.fx = positive_fixtures.PositiveRecoveryTests()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n
        for name in ('_begin_departure_clearance', '_reset_recovery_horizontal',
                     '_departure_clearance_ready', '_horizontal_attitude'):
            delattr(self.n, name)
        self.fx.fx.frame(12.8, distance=1., angle=5.)
        self.fx.fx.frame(12.9, distance=.95, angle=5.)
        self.fx.finish(5.)
        self.assertEqual(self.n.state, W.STATE_RECOVER)
        self.n._wall_fusion_samples_ready = lambda now: True
        self.n.horizontal_estimator.velocity_horiz_status_flag = True

    def frame(self, t, distance=.95, angle=0., speed=0.):
        n = self.n
        self.fx.fx.frame(t, distance=distance, angle=angle, height=math.nan, vz=math.nan)
        n.odom.twist.twist.linear.x = speed
        n.horizontal_estimator.header.stamp = at(t)
        flow = OpticalFlowRad(distance=distance, quality=100, integration_time_us=20000)
        flow.header.stamp = at(t)
        n.horizontal_flow_snapshot = (flow, at(t), '')

    def front(self, t, distance, source=None):
        self.fx.fx.seconds = t
        message = Range(range=distance, min_range=.03, max_range=6.)
        message.header.stamp = at(t if source is None else source)
        self.n._on_front_range(message)

    def test_entry_reanchors_once_to_actual_projected_height_not_old_target(self):
        n = self.n
        expected = .95*math.cos(math.radians(5.))
        self.assertTrue(n.recover_height_captured)
        self.assertAlmostEqual(n.detach_range.target, expected)
        self.assertAlmostEqual(n.detach_vertical_controller.reference, expected)
        self.assertEqual(n.detach_range.target_source, 'recover_entry_range_height')
        revision = n.detach_range.target_revision
        self.frame(13.1, distance=.90)
        n._simple_recovery_attitude(at(13.1))
        self.assertAlmostEqual(n.detach_range.target, expected)
        self.assertEqual(n.detach_range.target_revision, revision)

    def test_missing_front_feedback_keeps_zero_target_without_retreat(self):
        n = self.n
        n.front_range = n.front_received_at = None
        n.sensor_health = None
        for i in range(1, 12):
            t = 13.+i*.05
            self.frame(t)
            n._simple_recovery_attitude(at(t))
        self.assertTrue(n.horizontal_velocity_active)
        self.assertFalse(n.departure_feedback_valid)
        self.assertEqual(n.recovery_horizontal_controller.target, (0., 0.))

    def test_delayed_height_is_captured_before_first_range_thrust_update(self):
        n = self.n
        n.recover_height_captured = False
        n.detach_range_synced = False
        n.recovery_horizontal_controller.target = (0., 0.)
        self.frame(13.1)
        # Temporarily unavailable output synchronization must inhibit motion.
        n.detach_range_synced = False
        n._simple_recovery_attitude(at(13.1))
        self.assertFalse(n.horizontal_velocity_active)
        self.assertFalse(n.recover_height_captured)
        n.detach_range_synced = True
        self.frame(13.2, distance=.9)
        expected = n.detach_range.read_snapshot(13.2).projected_height
        original = n.detach_vertical_controller.update_range
        def checked(*args, **kwargs):
            self.assertTrue(n.recover_height_captured)
            self.assertAlmostEqual(n.detach_vertical_controller.reference, expected)
            return original(*args, **kwargs)
        with mock.patch.object(n.detach_vertical_controller, 'update_range', side_effect=checked):
            n._range_vertical_thrust(at(13.2))
        self.assertTrue(n.recover_height_captured)


    def test_front_frames_no_longer_latch_or_drive_retreat(self):
        n = self.n
        self.frame(13.1)
        n.recovery_horizontal_controller.target = (-.05, 0.)
        self.front(13.1, .5001)
        self.assertFalse(n.departure_clear_confirmed)
        self.assertEqual(n.departure_clear_samples, 0)
        self.front(13.11, .2)
        self.front(13.12, math.nan)
        self.frame(14.)
        n._simple_recovery_attitude(at(14.))
        self.assertFalse(n.departure_clear_confirmed)
        self.assertEqual(n.recovery_horizontal_controller.target, (0., 0.))
        n._reset_horizontal_feedback('temporary fusion pause')
        self.assertFalse(n.departure_clear_confirmed)
        n._begin_departure_clearance(at(15.))
        self.assertFalse(n.departure_clear_confirmed)

    def test_stale_pre_recover_future_nan_inf_out_of_range_and_equal_do_not_latch(self):
        for distance, source in ((.5, 13.1), (.8, 12.99), (.8, 10.), (.8, 14.),
                                 (math.nan, 13.1), (math.inf, 13.1), (7., 13.1)):
            with self.subTest(distance=distance, source=source):
                self.front(13.1, distance, source=source)
                self.assertFalse(self.n.departure_clear_confirmed)

    def test_large_front_frame_does_not_bypass_attitude_stability(self):
        n = self.n
        self.frame(13.1)
        self.front(13.1, .6)
        self.assertFalse(n.recover_stable)
        n._update_state_machine(at(13.1))
        self.assertEqual(n.state, W.STATE_RECOVER)
        self.assertFalse(n.recover_stable)

    def test_baseline_levels_smoothly_even_before_fusion_restored(self):
        n = self.n
        n._wall_fusion_samples_ready = lambda now: False
        n.wall_fusion_reason = 'restoring'
        previous = n.recover_baseline_axis
        for i in range(1, 61):
            t = 13.+i*.02
            self.frame(t)
            q = n._simple_recovery_attitude(at(t))
            axis = rotate_vector(q, (0., 0., 1.))
            step = math.acos(max(-1., min(1., sum(a*b for a,b in zip(axis, previous)))))
            self.assertLessEqual(step, math.radians(.201))
            previous = axis
        self.assertAlmostEqual(previous[2], 1., places=8)
        self.assertFalse(n.horizontal_velocity_active)

    def test_after_blend_has_zero_target_and_preserves_hover_correction_limits(self):
        n = self.n
        for i in range(1, 61):
            t = 13.+i*.02
            self.frame(t)
            q = n._simple_recovery_attitude(at(t))
        axis = rotate_vector(q, (0., 0., 1.))
        self.assertAlmostEqual(axis[0], 0.)
        self.assertLessEqual(math.acos(axis[2]), math.radians(5.))
        self.assertEqual(n.recovery_horizontal_controller.target, (0., 0.))

    def test_missing_trusted_baseline_or_high_rate_cannot_command_retreat(self):
        n = self.n
        self.frame(13.1)
        n.horizontal_equilibrium.frozen = False
        n._simple_recovery_attitude(at(13.1))
        self.assertFalse(n.horizontal_velocity_active)
        self.assertEqual(n.recovery_horizontal_controller.target, (0., 0.))
        n.horizontal_equilibrium.frozen = True
        self.frame(13.2)
        n.odom.twist.twist.angular.y = math.radians(25.)
        n._simple_recovery_attitude(at(13.2))
        self.assertFalse(n.departure_feedback_valid)
        self.assertEqual(n.recovery_horizontal_controller.target, (0., 0.))

    def test_recovery_reanchor_does_not_inherit_manual_fifteen_second_budget(self):
        self.frame(30., distance=.95)
        self.n._prepare_range_feedback(at(30.))
        self.assertFalse(self.n.detach_range_faulted)
        self.assertFalse(self.n.detach_vertical_fault)


if __name__ == '__main__':
    unittest.main()
