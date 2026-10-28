"""Downward-range reference ascent; no hardware or flight commands."""
import math
import sys
from pathlib import Path
import unittest
import rospy
from mavros_msgs.msg import OpticalFlowRad
from sensor_msgs.msg import Range
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, SensorHealth
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_ceiling_motion as lifecycle


class DownwardFlowAscentTests(unittest.TestCase):
    def setUp(self):
        self.f = lifecycle.CeilingMotionLifecycleTests(methodName='runTest')
        self.f.extra_params = {'~wall_contact_distance_m': .067,
            '~abort_land_on_fault': True}
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.n = self.f.n

    def take_over(self, distance=.6):
        for _ in range(14):
            self.f.feed(distance=distance, flow_distance=2.)
        self.assertTrue(self.n._activation_ready(rospy.Time.now()))
        self.f.mode = 'OFFBOARD'
        for _ in range(2):
            self.f.feed(distance=distance, flow_distance=2.)
        self.assertEqual(self.n.state, S.STATE_APPROACH)
        self.assertEqual(self.n.approach_flow_reference, (distance, 2.))

    def test_top_loss_longer_than_two_seconds_does_not_stop_healthy_flow_ascent(self):
        self.take_over()
        for _ in range(35):
            status = self.f.feed(distance=255., flow_distance=2.)
            self.n._on_sensor_health(SensorHealth(ceiling_valid=False))
        self.assertEqual(self.n.state, S.STATE_APPROACH)
        self.assertTrue(status.range_fresh)
        self.assertGreater(self.n.motion_last_target_vz, 0.)
        self.assertIn('朝下光流', status.reason)

    def test_remaining_distance_uses_downward_height_change_not_top_or_odom_height(self):
        self.take_over()
        self.f.feed(distance=.04, flow_distance=2.1)
        self.assertAlmostEqual(self.n.range_message.range, .5)
        self.assertAlmostEqual(self.n._closed_loop_approach_velocity(rospy.Time.now()), .15)
        self.assertEqual(self.n.state, S.STATE_APPROACH)
        self.f.feed(distance=10., flow_distance=2.2)
        self.assertAlmostEqual(self.n.range_message.range, .4)
        self.assertAlmostEqual(self.n.approach_flow_reference[0], .6)

    def test_attitude_projection_does_not_turn_tilt_into_false_climb(self):
        self.take_over()
        tilt = math.radians(8.)
        self.f.feed(distance=255., flow_distance=2./math.cos(tilt), tilt=tilt)
        self.assertAlmostEqual(self.n._approach_vertical_distance(rospy.Time.now()), .6, places=6)

    def test_approach_slows_as_downward_height_reaches_reference(self):
        self.take_over()
        first = self.n._closed_loop_approach_velocity(rospy.Time.now())
        for i in range(1, 11):
            self.f.feed(distance=255., flow_distance=2.+i*.05)
        self.assertEqual(self.n.state, S.STATE_APPROACH)
        self.assertLess(self.n._closed_loop_approach_velocity(rospy.Time.now()), first)
        self.assertAlmostEqual(self.n.range_message.range, .1)

    def test_new_flow_samples_can_complete_contact_after_top_loss(self):
        self.take_over()
        for i in range(1, 12):
            self.f.feed(distance=255., flow_distance=2.+min(i*.05, .545))
        self.assertEqual(self.n.state, S.STATE_ATTACH_CONTROL)
        for _ in range(200):
            self.f.feed(dt=.02, distance=255., flow_distance=2.545)
        self.assertEqual(self.n.state, S.STATE_SURFACE_HOLD)

    def test_repeated_timer_never_refreshes_flow_measurement(self):
        self.take_over()
        self.f.feed(distance=255., flow_distance=2.1)
        receipt = self.n.range_received_at
        for _ in range(5):
            self.f.clock += .02
            self.n._on_timer(None)
        self.assertEqual(self.n.range_received_at, receipt)

    def test_absent_top_baseline_cannot_start(self):
        for _ in range(14):
            self.f.feed(distance=255., flow_distance=2.)
        self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.assertIsNone(self.n.approach_flow_reference)

    def test_bad_or_stale_downward_data_stops_and_eventually_aborts(self):
        self.take_over()
        self.f.feed(distance=255., flow_distance=0.)
        self.assertFalse(self.n._range_fresh(rospy.Time.now()))
        self.assertEqual(self.n._closed_loop_approach_velocity(rospy.Time.now()), 0.)
        for _ in range(30):
            self.f.feed(distance=255., flow=False)
        self.assertEqual(self.n.state, S.STATE_ABORT)

    def test_fresh_flow_packet_with_old_distance_is_rejected(self):
        self.take_over()
        f = self.n.motion_flow
        f.time_delta_distance_us = 700000
        self.n._update_approach_flow_range(rospy.Time.now())
        self.assertFalse(self.n._range_fresh(rospy.Time.now()))

    def test_replayed_flow_source_cannot_become_a_new_height_sample(self):
        self.take_over()
        f = OpticalFlowRad(quality=100, integration_time_us=100000, distance=2.4)
        f.header.stamp = self.n.motion_flow.header.stamp
        self.n._on_ceiling_flow(f)
        self.n._update_approach_flow_range(rospy.Time.now())
        self.assertFalse(self.n._range_fresh(rospy.Time.now()))

    def test_flow_range_outside_qualified_range_is_rejected(self):
        self.take_over()
        self.f.feed(distance=255., flow_distance=7.)
        self.assertFalse(self.n._range_fresh(rospy.Time.now()))
        self.assertEqual(self.n._closed_loop_approach_velocity(rospy.Time.now()), 0.)

    def test_floor_step_latches_fault_instead_of_moving_ceiling_reference(self):
        self.take_over(distance=3.)
        self.f.feed(distance=255., flow_distance=3.)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertIn('跳变', self.n.last_reason)
        self.assertEqual(self.n.approach_flow_reference, (3., 2.))

    def test_total_ascent_budget_still_applies(self):
        self.take_over()
        self.n.approach_budget_s = .3
        for _ in range(5):
            self.f.feed(distance=255., flow_distance=2.)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertIn('time budget', self.n.last_reason)

    def test_cycle_reset_discards_reference_and_requires_a_new_top_sample(self):
        self.take_over()
        self.n._enter_state(S.STATE_NORMAL_FLIGHT, rospy.Time.now(), 'test reset')
        self.assertIsNone(self.n.approach_flow_reference)
        self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.n._on_range(Range(range=.8, min_range=.03, max_range=40.))
        self.assertTrue(self.n._activation_ready(rospy.Time.now()))

    def test_hold_does_not_inherit_ascent_deadline_but_keeps_flow_loss_watchdog(self):
        self.take_over()
        self.n._enter_state(S.STATE_SURFACE_HOLD, rospy.Time.now(), 'test hold')
        self.n.approach_budget_s = .01
        self.f.clock += .02
        self.assertEqual(self.n._closed_loop_approach_fault(rospy.Time.now()), '')
        self.n.motion_flow.time_delta_distance_us = 700000
        self.n._closed_loop_approach_fault(rospy.Time.now())
        self.f.clock += 2.1
        self.assertIn('downward flow height lost', self.n._closed_loop_approach_fault(rospy.Time.now()))

    def test_no_motion_from_timer_ticks_can_confirm_estimated_contact(self):
        self.take_over(distance=.055)
        self.n.attach_thrust_command = self.n.surface_hold_thrust
        self.n._closed_loop_contact_ready(rospy.Time.now())
        for _ in range(30):
            self.f.clock += .02
            self.n._closed_loop_contact_ready(rospy.Time.now())
        self.assertFalse(self.n._closed_loop_contact_ready(rospy.Time.now()))

    def test_unsynchronized_baseline_is_rejected(self):
        for _ in range(14):
            self.f.feed(distance=.6, flow_distance=2.)
        self.n.range_received_at -= rospy.Duration(.3)
        self.assertFalse(self.n._activation_ready(rospy.Time.now()))

    def test_estimate_beyond_ceiling_latches_abort(self):
        self.take_over(distance=.1)
        self.f.feed(distance=255., flow_distance=2.11)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertIn('超过锁定天花板基准', self.n.last_reason)


if __name__ == '__main__':
    unittest.main()
