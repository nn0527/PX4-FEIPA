"""Estimated ascent and handoff to physical upward lidar at contact."""
import sys
from pathlib import Path
import unittest

import rospy
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as Status

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_ceiling_motion as lifecycle


class TopLidarAscentTests(unittest.TestCase):
    def setUp(self):
        self.flight = lifecycle.CeilingMotionLifecycleTests(methodName='runTest')
        self.flight.extra_params = {'~wall_contact_distance_m': .067,
                                    '~abort_land_on_fault': True}
        self.flight.setUp()
        self.addCleanup(self.flight.doCleanups)
        self.node = self.flight.n

    def take_over(self):
        for _ in range(14):
            self.flight.feed(distance=.6, flow_distance=2.)
        self.flight.mode = 'OFFBOARD'
        for _ in range(2):
            self.flight.feed(distance=.6, flow_distance=2.)
        self.assertEqual(self.node.state, Status.STATE_APPROACH)

    def test_measured_contact_passes_when_downward_estimate_would_miss(self):
        self.take_over()
        distance = self.node._compressed_target_distance()
        status = self.flight.feed(distance=distance, flow_distance=2.1)
        self.assertAlmostEqual(status.ceiling_distance_raw_m, distance)
        self.assertEqual(self.node.state, Status.STATE_SURFACE_HOLD)
        self.assertFalse(self.node._flow_approach_active())
        status = self.flight.feed(distance=distance, flow_distance=4.)
        self.assertAlmostEqual(status.ceiling_distance_raw_m, distance)
        self.assertEqual(self.node.state, Status.STATE_SURFACE_HOLD)

    def test_downward_range_cannot_trip_contact_gate(self):
        self.take_over()
        for step in range(1, 12):
            status = self.flight.feed(distance=.1, flow_distance=2. + .05 * step)
        self.assertAlmostEqual(status.ceiling_distance_raw_m, .1)
        self.assertEqual(self.node.state, Status.STATE_APPROACH)

    def test_missing_top_sample_keeps_approach_using_downward_range(self):
        self.take_over()
        status = self.flight.feed(distance=255., flow_distance=2.1)
        self.assertFalse(status.range_fresh)
        self.assertGreater(self.node._closed_loop_approach_velocity(rospy.Time.now()), 0.)
        for _ in range(30):
            status = self.flight.feed(distance=255., flow_distance=2.1)
        self.assertFalse(status.range_fresh)
        self.assertEqual(self.node.state, Status.STATE_APPROACH)

    def test_hold_uses_top_range_loss_watchdog_without_flow_fallback(self):
        self.take_over()
        self.flight.feed(distance=.054, flow_distance=2.1)
        self.assertEqual(self.node.state, Status.STATE_SURFACE_HOLD)
        for _ in range(25):
            self.flight.feed(distance=255., flow_distance=2.1)
        self.assertEqual(self.node.state, Status.STATE_ABORT)
        self.assertIn('ceiling lidar range lost', self.node.last_reason)

    def test_total_ascent_deadline_still_applies(self):
        self.take_over()
        self.node.approach_budget_s = .3
        for _ in range(5):
            self.flight.feed(distance=.6, flow_distance=2.)
        self.assertEqual(self.node.state, Status.STATE_ABORT)
        self.assertIn('time budget', self.node.last_reason)


if __name__ == '__main__':
    unittest.main()
