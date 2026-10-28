"""Source-time, geometry and low-rate range contracts; no ROS required."""
import math
import unittest
from dataclasses import replace

from px4_wall_ceiling_control.domain.range_height_feedback import RangeHeightConfig, RangeHeightFeedback
from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController


def pitch(degrees):
    a = math.radians(degrees)/2.
    return (0., math.sin(a), 0., math.cos(a))


class RangeHeightTests(unittest.TestCase):
    def test_read_snapshot_cannot_lock_pending_frame_or_advance_clock(self):
        r = self.tracker()
        r.observe_pose(10.24, 10.25, pitch(7.))
        r.observe_range(10.2501, 10.25, 1.13)
        self.assertFalse(r.locked)
        last_tick, pending = r.last_tick, tuple(r.pending)
        r.read_snapshot(10.280711)
        self.assertFalse(r.locked)
        self.assertEqual(r.last_tick, last_tick)
        self.assertEqual(tuple(r.pending), pending)
        r.observe_pose(10.26, 10.29, pitch(6.))
        self.assertTrue(r.locked)
        self.assertEqual(r.action_started, 10.29)

    def test_read_snapshot_does_not_invalidate_history_but_still_reports_expiry(self):
        r = self.tracker()
        self.frame(r, 10.1)
        self.frame(r, 10.35)
        last_tick, samples = r.last_tick, tuple(r.samples)
        self.assertFalse(r.read_snapshot(11.).valid)
        self.assertFalse(r.read_snapshot(10.2).valid)
        self.assertEqual(r.last_tick, last_tick)
        self.assertEqual(tuple(r.samples), samples)
        self.assertIsNone(r.invalid_since)
        r.process(11.)
        self.assertIsNotNone(r.invalid_since)



    def tracker(self, **kwargs):
        # Synthetic device limits for tests, never installation defaults.
        c = replace(RangeHeightConfig(), min_distance_m=.1, max_distance_m=5.,
                    geometry_confirmed=True, **kwargs)
        r = RangeHeightFeedback(c)
        r.begin(7, 10.)
        return r

    def frame(self, r, source, height=1.24, tilt=0., received=None):
        received = source if received is None else received
        r.observe_pose(source, received, pitch(tilt))
        r.observe_range(source, received, height)
        return r.snapshot(received)

    def test_historical_values_reject_high_angle_and_invalid_before_124(self):
        r = self.tracker()
        self.frame(r, 10.1, 4.48, 70.)
        self.frame(r, 10.2, -1., 8.)
        self.assertFalse(r.locked)
        s = self.frame(r, 10.3, 1.24, 2.66)
        self.assertAlmostEqual(s.target, 1.24*math.cos(math.radians(2.66)), places=9)
        self.assertFalse(s.velocity_valid)
        self.assertTrue(math.isnan(s.velocity))
        self.assertEqual(r.lock_raw, 1.24)

    def test_delayed_pose_keeps_first_candidate_ahead_of_later_range(self):
        r = self.tracker()
        r.observe_range(10.1, 10.1, 1.24)
        r.observe_range(10.12, 10.12, 1.5)
        self.assertFalse(r.locked)
        r.observe_pose(10.09, 10.125, pitch(8.))
        r.observe_pose(10.11, 10.126, pitch(4.))
        self.assertEqual(r.lock_source, 10.1)
        self.assertAlmostEqual(r.target, 1.24*math.cos(math.radians(6.)), places=8)

    def test_receipt_pose_cannot_qualify_old_high_tilt_range(self):
        r = self.tracker()
        r.observe_pose(10.1, 10.1, pitch(40.))
        r.observe_pose(10.3, 10.3, pitch(1.))
        r.observe_range(10.1, 10.31, 4.48)
        self.assertFalse(r.locked)
        self.frame(r, 10.32)
        self.assertEqual(r.lock_source, 10.32)

    def test_interpolated_tilt_controls_8_10_12_degree_entry(self):
        for limit in (8., 10., 12.):
            r = self.tracker(entry_tilt_deg=limit)
            self.frame(r, 10.1, 2., limit+.1)
            self.assertFalse(r.locked)
            self.frame(r, 10.2, 1.2, limit)
            self.assertEqual(r.lock_source, 10.2)

    def test_target_immutable_across_rebound_loss_recovery_and_new_samples(self):
        r = self.tracker()
        self.frame(r, 10.1)
        target = r.target
        self.frame(r, 10.35, 1.3)
        self.frame(r, 10.6, 1.3, 16.)
        self.assertTrue(r.snapshot(10.6).valid)
        self.assertTrue(r.snapshot(10.6).velocity_valid)
        self.assertEqual(r.target, target)
        self.assertTrue(self.frame(r, 10.8, 1.25, 12.).velocity_valid)
        self.assertTrue(self.frame(r, 11.05, 1.25, 5.).velocity_valid)
        self.assertEqual(r.target, target)

    def test_duplicate_backward_expired_and_future_do_not_refresh_source(self):
        r = self.tracker()
        self.frame(r, 10.1)
        for stamp, receipt in ((10.1, 10.3), (10.05, 10.3), (10.2, 11.), (11.4, 11.)):
            r.observe_range(stamp, receipt, 2.)
        self.assertEqual(r.count, 1)
        self.assertEqual(r.source, 10.1)
        self.assertFalse(r.snapshot(11.).valid)

    def test_message_before_rotation_is_never_first_frame(self):
        r = self.tracker()
        self.frame(r, 10., 1.5)
        self.assertFalse(r.locked)

    def test_velocity_uses_source_intervals_and_duplicate_timer_adds_no_samples(self):
        r = self.tracker()
        self.frame(r, 10.1, 1.)
        s = self.frame(r, 10.35, 1.05, received=10.4)
        self.assertTrue(s.velocity_valid)
        self.assertAlmostEqual(s.velocity, .2)
        for t in (10.42, 10.44, 10.46):
            r.snapshot(t)
        self.assertEqual(r.count, 2)
        self.assertEqual(len(r.samples), 2)

    def test_gap_requires_new_velocity_window_and_keeps_target(self):
        r = self.tracker()
        self.frame(r, 10.1)
        self.frame(r, 10.35)
        self.assertTrue(r.velocity_valid)
        self.assertFalse(r.snapshot(11.).valid)
        self.assertFalse(self.frame(r, 11.1, 1.3).velocity_valid)
        self.assertTrue(self.frame(r, 11.35, 1.3).velocity_valid)
        self.assertEqual(r.target, 1.24)

    def test_jump_cannot_generate_large_velocity(self):
        r = self.tracker()
        self.frame(r, 10.1)
        s = self.frame(r, 10.35, 4.5)
        self.assertTrue(s.valid)
        self.assertFalse(s.velocity_valid)
        self.assertAlmostEqual(s.height, 1.24)
        self.assertEqual(r.target, 1.24)

    def test_invalid_numbers_and_device_limits(self):
        for value in (-1., 0., math.nan, math.inf, 5.01):
            r = self.tracker()
            self.frame(r, 10.1, value)
            self.assertFalse(r.locked)
        r = RangeHeightFeedback()
        r.begin(1, 10.)
        self.frame(r, 10.1)
        self.assertFalse(r.locked)

    def test_geometry_is_applied_once_to_both_target_and_feedback(self):
        r = self.tracker(reference_offset_body_m=(0., 0., .1))
        s = self.frame(r, 10.1, 1.24, 5.)
        expected = 1.34*math.cos(math.radians(5.))
        self.assertAlmostEqual(s.target, expected)
        self.assertEqual(s.height, s.target)

    def test_pending_pose_timeout_does_not_fabricate_height(self):
        r = self.tracker()
        r.observe_range(10.1, 10.1, 1.24)
        r.snapshot(10.31)
        self.assertFalse(r.locked)
        self.assertIn('采样时刻姿态缺失或匹配超时', r.rejected)

    def test_low_rate_pose_requires_bounded_bracketing_interpolation(self):
        r = self.tracker()
        r.observe_pose(10.09, 10.09, pitch(7.))
        r.observe_range(10.1, 10.11, 1.24)
        self.assertFalse(r.locked)
        r.observe_pose(10.17, 10.18, pitch(5.))
        self.assertTrue(r.locked)
        self.assertAlmostEqual(r.pose_match_dt_s, .08)
        self.assertAlmostEqual(r.pose_before_dt_s, .01)
        self.assertAlmostEqual(r.pose_after_dt_s, .07)
        self.assertAlmostEqual(r.target, 1.24*math.cos(math.radians(6.75)), places=7)

    def test_nearby_source_pose_is_used_after_wait_budget(self):
        r = self.tracker()
        r.observe_pose(10.09, 10.09, pitch(7.))
        r.observe_range(10.1, 10.11, 1.24)
        self.assertFalse(r.locked)
        r.snapshot(10.32)
        self.assertTrue(r.locked)
        self.assertAlmostEqual(r.target, 1.24*math.cos(math.radians(7.)))
        self.assertAlmostEqual(r.pose_match_dt_s, .01)

    def test_full_eighty_ms_pose_span_is_accepted(self):
        r = self.tracker()
        r.observe_pose(10.08, 10.09, pitch(8.))
        r.observe_range(10.10, 10.11, 1.24)
        r.observe_pose(10.16, 10.17, pitch(4.))
        s = r.snapshot(10.17)
        self.assertTrue(s.valid)
        self.assertAlmostEqual(s.pose_span_s, .08)
        self.assertAlmostEqual(s.pose_before_dt_s, .02)
        self.assertAlmostEqual(s.pose_after_dt_s, .06)

    def test_speed_confidence_failure_keeps_fresh_height_valid(self):
        r = self.tracker(jump_m=5.)
        self.frame(r, 10.1, 1.)
        s = self.frame(r, 10.35, 2.)
        self.assertTrue(s.valid)
        self.assertFalse(s.velocity_valid)
        self.assertIsNone(r.height_invalid_since)
        self.assertIsNotNone(r.velocity_invalid_since)

    def test_height_expiry_starts_at_earlier_source_or_receipt_deadline(self):
        r = self.tracker()
        self.frame(r, 10.1, received=10.2)
        r.process(10.71)
        self.assertFalse(r.read_snapshot(10.71).valid)
        self.assertAlmostEqual(r.height_invalid_since, 10.7)

    def test_fast_velocity_is_parallel_observer(self):
        r = self.tracker()
        self.frame(r, 10.1, 1.)
        s = self.frame(r, 10.35, 1.05)
        self.assertTrue(s.fast_velocity_valid)
        self.assertAlmostEqual(s.fast_velocity, .2)
        self.assertTrue(s.velocity_valid)

    def test_nearest_pose_outside_tolerance_never_qualifies(self):
        r = self.tracker()
        r.observe_pose(9.98, 9.98, pitch(0.))
        r.observe_range(10.1, 10.11, 1.24)
        r.snapshot(10.32)
        self.assertFalse(r.locked)

    def test_parameters_are_frozen_and_inconsistent_limits_rejected(self):
        r = self.tracker()
        with self.assertRaises(AttributeError):
            r.config.entry_tilt_deg = 12.
        for kwargs in (dict(entry_tilt_deg=61.), dict(timeout_s=.01), dict(stable_min_samples=1),
                       dict(sensor_axis_body=(0., 0., 0.)), dict(max_distance_m=-1.)):
            with self.assertRaises(ValueError):
                replace(RangeHeightConfig(), **kwargs)

    def test_begin_discards_previous_ranges_but_keeps_pose_buffer(self):
        r = self.tracker()
        self.frame(r, 10.1)
        r.begin(8, 10.2)
        self.assertFalse(r.locked)
        self.assertEqual(len(r.poses), 1)
        self.frame(r, 10.3, 1.5)
        self.assertEqual(r.episode, 8)
        self.assertEqual(r.target, 1.5)

    def test_unknown_velocity_uses_explicit_height_gain_with_no_integral(self):
        c = DepartureHeightController()
        c.begin(1.24, .53, .68, 10.)
        c.integral = .05
        value = c.update(1.24, None, 1., 10.02, 10., .68,
                         height_only_kp=.15, velocity_weight=0.)
        self.assertTrue(math.isnan(c.target_speed))
        self.assertEqual(c.integral, 0.)
        self.assertEqual(c.correction, 0.)
        self.assertLessEqual(abs(value-.68), .01000001)
        c.update(1.14, None, 1., 10.04, 10.04, value,
                 height_only_kp=.15, velocity_weight=0.)
        self.assertAlmostEqual(c.correction, .015)

    def test_four_hz_feedback_integrates_only_new_frames_and_retains_full_time_budget(self):
        c = DepartureHeightController()
        c.begin(1.24, .53, .545, 10.)
        for index in range(1, 251):
            now = 10.+index*.02
            source = 10.+math.floor((index*.02+1.e-8)/.25)*.25
            c.update(1.24, -.05, 1., now, source, .545+c.integral, height_only_kp=.15)
        self.assertAlmostEqual(c.integral, .12*.05*5., delta=.002)


if __name__ == '__main__':
    unittest.main()
