"""Bounded flip timing and source-clock/atomic-candidate regressions."""
import math
import unittest
from unittest import mock

import rospy
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate as C, WallPerchStatus as W,
)
from px4_wall_ceiling_control.domain.geometry import quaternion_angle
from test_attachment_safety import WallDecision
import test_flip_vertical_support as support_fixtures


def at(seconds):
    return rospy.Time.from_sec(seconds)


class FlipLatencyTests(unittest.TestCase):
    def node(self, entry_pitch=4.):
        n = support_fixtures.FlipVerticalSupportTests().node()
        angle = math.radians(entry_pitch)
        n.horizontal_output.orientation.y = math.sin(angle / 2.)
        n.horizontal_output.orientation.w = math.cos(angle / 2.)
        n.flip_duration_timeout_s = 6.
        n.flip_max_tracking_error_rad = math.radians(90.)
        n.flip_top_start_fraction = .9
        n._prepare_vertical_reference = lambda now: None
        with mock.patch.object(rospy, 'loginfo'):
            n._enter_state(W.STATE_FLIP_TO_WALL, at(10.), 'test')
        self.assertEqual(n.state, W.STATE_FLIP_TO_WALL)
        return n

    def candidate(self, n, seconds):
        return n._candidate(at(seconds))

    def test_entry_starts_at_last_published_nonzero_attitude(self):
        n = self.node(entry_pitch=4.)
        result = self.candidate(n, 10.)
        self.assertAlmostEqual(quaternion_angle(result[1], n.q_hover), math.radians(4.))
        self.assertAlmostEqual(quaternion_angle(result[1], n.q_flip_start), 0.)
        self.assertAlmostEqual(result[2], .518)
        self.assertEqual(result[3], 0.)

    def test_100_and_200ms_stalls_share_50ms_attitude_and_thrust_budget(self):
        for stall in (.10, .20):
            n = self.node(entry_pitch=0.)
            self.candidate(n, 10.)
            for i in range(1, 51):
                previous = self.candidate(n, 10. + i*.02)
            n.z = 2.; n.vz = -.5
            result = self.candidate(n, 11. + stall)
            self.assertAlmostEqual(n.flip_step_dt, .05)
            self.assertAlmostEqual(result[3] - previous[3], .05 / n.flip_time_s)
            self.assertLessEqual(abs(result[2] - previous[2]), .5*.05 + 1e-9)
            # Mid-trajectory 50ms is about 4.2 degrees, never the old 8/16 degree catchup.
            self.assertLess(quaternion_angle(result[1], previous[1]), math.radians(4.3))
            next_result = self.candidate(n, 11. + stall + .02)
            self.assertAlmostEqual(next_result[3] - result[3], .02 / n.flip_time_s)

    def test_repeated_same_timestamp_does_not_advance_or_recalculate_thrust(self):
        n = self.node()
        self.candidate(n, 10.)
        n.z = 2.; n.vz = -.5
        first = self.candidate(n, 10.1)
        n.tilt = 90.  # A second read during the same tick must not spend another slew budget.
        second = self.candidate(n, 10.1)
        self.assertEqual(first, second)
        self.assertEqual(n.flip_elapsed_s, .05)

    def test_long_gap_does_not_open_top_window_or_complete_trajectory(self):
        n = self.node()
        self.candidate(n, 10.)
        self.candidate(n, 12.)
        self.assertFalse(n._late_top_window_open(at(12.)))
        self.assertIsNone(n.top_hold_accept_after)
        self.assertAlmostEqual(n._flip_progress(), .025)

    def test_top_window_opens_at_actual_bounded_progress_timestamp(self):
        n = self.node()
        self.candidate(n, 10.)
        self.candidate(n, 10.2)  # Discard 150ms of stalled time.
        for i in range(1, 88):
            self.candidate(n, 10.2 + i*.02)
        self.assertFalse(n._late_top_window_open(at(11.94)))
        self.candidate(n, 11.96)
        self.assertTrue(n._late_top_window_open(at(11.96)))
        self.assertEqual(n.top_hold_accept_after, at(11.96))
        # A range generated at nominal 11.8 would precede this opening.
        self.assertGreater(n.top_hold_accept_after, at(11.8))

    def state_machine(self, n):
        n._mode_requested = lambda now: True
        n.cancel_requested = n.rearm_required = False
        n._safety_ok = lambda now: True
        n._late_top_frame_ready = lambda now: False
        n._pin_triggered = lambda: False
        def enter(state, now, reason):
            n.state = state
            n.last_reason = reason
        n._enter_state = mock.Mock(side_effect=enter)

    def test_absolute_timeout_aborts_even_if_feedback_is_fresh(self):
        n = self.node()
        self.state_machine(n)
        n._update_state_machine(at(15.99))
        self.assertEqual(n.state, W.STATE_FLIP_TO_WALL)
        n._update_state_machine(at(16.))
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertEqual(n.fault_count, 1)
        self.assertIn('wall-clock duration timeout', n.last_reason)
        self.assertTrue(n.abort_land_requested)

    def test_backward_clock_aborts_instead_of_resetting_or_rewinding(self):
        n = self.node()
        self.candidate(n, 10.2)
        self.state_machine(n)
        n._update_state_machine(at(10.1))
        self.assertEqual(n.state, W.STATE_ABORT)
        self.assertIn('clock moved backwards', n.last_reason)
        self.assertFalse(getattr(n, 'abort_land_requested', False))
        self.assertAlmostEqual(n.flip_elapsed_s, .05)

    def test_capture_waits_for_accumulated_progress(self):
        n = self.node()
        self.state_machine(n)
        n._update_state_machine(at(12.))
        self.assertEqual(n.state, W.STATE_FLIP_TO_WALL)
        for i in range(1, 100):
            n._advance_flip_progress(at(12. + .02*i))
        n._update_state_machine(at(13.99))
        self.assertEqual(n.state, W.STATE_WALL_CAPTURE)

    def test_tracking_lag_limits_target_but_preserves_vertical_thrust_response(self):
        n = self.node(entry_pitch=0.)
        n.flip_max_tracking_error_rad = math.radians(15.)
        for i in range(101):
            result = self.candidate(n, 10. + i*.02)
        self.assertTrue(n.flip_tracking_limited)
        self.assertLessEqual(quaternion_angle(result[1], n._current_quaternion()), math.radians(15.) + 1e-8)
        progress, thrust = result[3], result[2]
        n.z = 2.; n.vz = -.5
        result = self.candidate(n, 12.02)
        self.assertAlmostEqual(result[3], progress, places=7)
        self.assertGreater(result[2], thrust)
        self.assertLessEqual(result[2] - thrust, .5*.02 + 1e-9)
        n.tilt = 15.
        result = self.candidate(n, 12.04)
        self.assertGreater(result[3], progress)

    def test_measured_rotation_ahead_of_target_can_reduce_tracking_error(self):
        n = self.node(entry_pitch=0.)
        n.flip_max_tracking_error_rad = math.radians(15.)
        n.tilt = 30.
        result = self.candidate(n, 10.05)
        self.assertGreater(result[3], 0.)
        self.assertLess(quaternion_angle(result[1], n._current_quaternion()), math.radians(30.))

    def test_automatic_entry_rejects_missing_stale_or_invalid_final_attitude(self):
        for fault in ('missing', 'stale', 'zero', 'ignored', 'foreign'):
            n = support_fixtures.FlipVerticalSupportTests().node()
            n.horizontal_output.orientation.w = 1.
            if fault == 'missing': n.horizontal_output = None
            elif fault == 'stale': n.horizontal_output.header.stamp = at(9.)
            elif fault == 'zero': n.horizontal_output.orientation.w = 0.
            elif fault == 'ignored': n.horizontal_output.type_mask |= n.horizontal_output.IGNORE_ATTITUDE
            else: n.horizontal_output_owner = C.SOURCE_CEILING
            self.assertIsNone(n._flip_entry_quaternion(at(10.)), fault)

    def test_manual_acquisition_uses_fresh_measured_attitude_without_owner(self):
        n = self.node()
        n.auto_approach = False; n.horizontal_output = None; n.tilt = 7.
        q = n._flip_entry_quaternion(at(10.))
        self.assertAlmostEqual(quaternion_angle(q, n.q_hover), math.radians(7.))
        n.odom_valid = False
        self.assertIsNone(n._flip_entry_quaternion(at(10.)))


class DecisionSourceTimeTests(unittest.TestCase):
    def node(self):
        n = WallDecision.__new__(WallDecision)
        n.odom = n.odom_received_at = n.odom_source_stamp = None
        n.odom_timeout_s = .5
        n.state = W.STATE_WALL_HOLD
        n.detach_stable_since = None; n.detach_stable_samples = 0
        n._detach_pose_valid = lambda: True
        return n

    def feed(self, n, receipt, source):
        message = Odometry()
        message.header.stamp = at(source)
        message.pose.pose.orientation.w = 1.
        with mock.patch.object(rospy.Time, 'now', return_value=at(receipt)):
            n._on_odom(message)
        return message

    def test_old_duplicate_and_out_of_order_samples_do_not_refresh_or_accumulate(self):
        n = self.node()
        first = self.feed(n, 10., 9.9)
        for receipt, source in [(10.1, 9.9), (10.2, 9.8), (10.3, 9.7), (10.4, 0.), (10.4, 12.)]:
            self.feed(n, receipt, source)
            self.assertIs(n.odom, first)
            self.assertEqual(n.odom_received_at, at(10.))
            self.assertEqual(n.detach_stable_samples, 1)
        self.feed(n, 10.4, 10.35)
        self.assertEqual(n.detach_stable_samples, 2)
        self.assertEqual(n.odom_received_at, at(10.4))

    def test_small_mapped_future_is_accepted_but_source_expiry_still_applies(self):
        n = self.node()
        self.feed(n, 10., 10.05)
        self.assertEqual(n.detach_stable_samples, 1)
        self.assertTrue(n._odom_fresh(at(10.)))
        self.assertFalse(n._odom_fresh(at(10.6)))

    def test_recent_receipt_cannot_hide_expired_source(self):
        n = self.node()
        self.feed(n, 10., 9.55)
        self.assertTrue(n._odom_fresh(at(10.)))
        self.assertFalse(n._odom_fresh(at(10.1)))

    def test_detach_stability_requires_source_samples_spanning_hold_duration(self):
        n = self.node()
        n.state_entered_at = at(9.)
        n.hold_entry_blend_s = n.detach_stable_hold_s = .5
        self.feed(n, 10., 10.)
        self.feed(n, 10.1, 10.1)
        self.assertFalse(n._detach_pose_ready(at(10.5)))
        self.feed(n, 10.5, 10.5)
        self.assertTrue(n._detach_pose_ready(at(10.5)))
        self.feed(n, 11.1, 11.1)  # A feedback gap must start a new stability interval.
        self.assertEqual(n.detach_stable_samples, 1)
        self.assertFalse(n._detach_pose_ready(at(11.1)))

    def test_candidate_embeds_same_status_phase_flags_and_action_identity(self):
        status = W()
        status.header.stamp = at(10.)
        status.state = W.STATE_FLIP_TO_WALL
        status.horizontal_velocity_enabled = True
        status.horizontal_velocity_active = False
        status.approach_paused = True
        status.attitude_setpoint_valid = status.candidate_valid = True
        status.attitude_setpoint.w = 1.; status.thrust_normalized = .518
        candidate = WallDecision._build_control_candidate(status, 'attempt-a', 17)
        self.assertTrue(candidate.phase_valid)
        self.assertEqual(candidate.control_phase, status.state)
        self.assertEqual(candidate.maneuver_id, 'attempt-a')
        self.assertEqual(candidate.control_sequence, 17)
        self.assertEqual(candidate.header.stamp, status.header.stamp)
        self.assertTrue(candidate.horizontal_velocity_enabled)
        self.assertFalse(candidate.horizontal_velocity_active)
        self.assertTrue(candidate.approach_paused)


class RangeSourceTimeTests(unittest.TestCase):
    def node(self):
        from test_front_only_late_top import FrontOnlyLateTopTests
        return FrontOnlyLateTopTests().node()

    def feed(self, n, receipt, source, top=False):
        from sensor_msgs.msg import Range
        message = Range(range=.04, min_range=.03, max_range=40.)
        message.header.stamp = at(source)
        with mock.patch.object(rospy.Time, 'now', return_value=at(receipt)):
            (n._on_top_range if top else n._on_front_range)(message)
        return message

    def test_front_replay_cannot_refresh_or_complete_flip_gate(self):
        n = self.node()
        self.feed(n, 10., 9.9)
        self.feed(n, 10.4, 9.9)
        self.assertEqual(n.front_received_at, at(10.))
        self.assertFalse(n._flip_ready(at(10.4)))
        self.assertFalse(n._front_fresh(at(10.55)))

    def test_receipt_delay_does_not_inflate_front_source_hold_duration(self):
        n = self.node()
        self.feed(n, 10., 9.9)
        self.feed(n, 10.4, 10.)
        self.assertFalse(n._flip_ready(at(10.4)))
        self.assertAlmostEqual((n.flip_ready_last_sample_at - n.flip_ready_since).to_sec(), .1)
        self.feed(n, 10.5, 10.3)
        self.assertTrue(n._flip_ready(at(10.5)))

    def test_top_replay_and_zero_or_stale_headers_do_not_refresh_contact(self):
        n = self.node(); n.state = W.STATE_WALL_HOLD
        self.feed(n, 10., 9.9, top=True)
        for receipt, source in [(10.1, 9.9), (10.2, 9.8), (10.3, 0.), (10.4, 8.)]:
            self.feed(n, receipt, source, top=True)
            self.assertEqual(n.top_received_at, at(10.))
            self.assertEqual(n.top_contact_last_sample_at, at(9.9))


if __name__ == '__main__':
    unittest.main()
