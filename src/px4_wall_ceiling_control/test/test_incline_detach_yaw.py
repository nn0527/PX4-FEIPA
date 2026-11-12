"""Receipt-time measured yaw guard; no flight commands or live ROS nodes."""
from dataclasses import replace
import math
import unittest

import test_incline_attachment as fixtures
from px4_wall_ceiling_control.domain.geometry import hover_and_pitched_quaternions


def attitude(yaw, pitch=30):
    return hover_and_pitched_quaternions(math.radians(yaw), math.radians(pitch))[1]


class DetachYawLogicTests(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.InclineLogicTests()
        fixture.setUp()
        self.c, self.f = fixture.c, fixture.f
        self.c.phase = 'HOLD'
        self.c.q = self.c.contact_q = attitude(0)
        self.c.thrust = .49
        self.f = replace(self.f, q=attitude(0), applied_q=attitude(0),
                         top_valid=False)

    def begin(self, yaw=0, target_yaw=None):
        self.f = replace(self.f, q=attitude(yaw),
            applied_q=attitude(yaw if target_yaw is None else target_yaw))
        self.c._return(self.f)
        self.assertEqual(self.c.phase, 'RETURN')

    def step(self, yaw, phase=None):
        if phase:
            self.c.phase = phase
        now = self.f.now+.02
        self.f = replace(self.f, now=now, pose_stamp=now,
                         output_stamp=now, q=attitude(yaw))
        self.c.step(self.f)

    def test_baseline_is_measured_yaw_not_target_yaw(self):
        self.begin(yaw=15, target_yaw=70)
        self.assertAlmostEqual(self.c.detach_yaw_reference, math.radians(15))
        self.assertEqual(self.c.detach_yaw_reference_stamp, self.f.pose_stamp)
        self.assertEqual(self.c.detach_yaw_request_at, self.f.now)

    def test_strict_positive_and_negative_30_degree_boundary(self):
        for yaw, abort in ((29.99, False), (30., False), (30.001, True),
                           (-30., False), (-30.001, True), (50.528, True)):
            with self.subTest(yaw=yaw):
                self.setUp()
                self.begin()
                self.step(yaw)
                self.assertEqual(self.c.phase == 'LAND', abort)
                if abort:
                    self.assertIn('超过30度', self.c.fault)
                    self.assertIn('基准0.000', self.c.fault)

    def test_wrapped_yaw_uses_shortest_angle(self):
        for start, end, abort in ((179, -179, False), (-179, 179, False),
                                  (179, -151, False), (179, -150.99, True)):
            with self.subTest(start=start, end=end):
                self.setUp()
                self.begin(start)
                self.step(end)
                self.assertEqual(self.c.phase == 'LAND', abort)

    def test_reference_does_not_follow_subthreshold_drift(self):
        self.begin()
        self.step(16)
        self.assertEqual(self.c.phase, 'RETURN')
        self.step(32)
        self.assertEqual(self.c.phase, 'LAND')
        self.assertEqual(self.c.detach_yaw_reference, 0.)

    def test_duplicate_latch_does_not_replace_receipt_sample(self):
        self.assertTrue(self.c.latch_detach_yaw(self.f))
        later = replace(self.f, q=attitude(8), now=10.1, pose_stamp=10.1)
        self.assertTrue(self.c.latch_detach_yaw(later))
        self.assertEqual(self.c.detach_yaw_reference, 0.)
        self.assertEqual(self.c.detach_yaw_request_at, 10.)

    def test_invalid_receipt_pose_cannot_seed_a_baseline(self):
        for f in (replace(self.f, pose_valid=False), replace(self.f, pose_stamp=0),
                  replace(self.f, q=(math.nan, 0., 0., 1.))):
            self.setUp()
            self.assertFalse(self.c.latch_detach_yaw(f))
            self.assertEqual(self.c.phase, 'LAND')
            self.assertIsNone(self.c.detach_yaw_reference)

    def test_guard_already_active_between_receipt_and_return_entry(self):
        self.assertTrue(self.c.latch_detach_yaw(self.f))
        self.step(31)
        self.assertEqual(self.c.phase, 'LAND')
        self.assertIn('偏航', self.c.fault)

    def test_new_task_starts_without_old_yaw_baseline(self):
        self.begin(15)
        new = type(self.c)(self.c.s, .85)
        self.assertIsNone(new.detach_yaw_reference)

    def test_guard_does_not_extend_into_recovery(self):
        self.begin()
        self.c.recovery_started = self.f.now
        self.step(32, phase='RECOVER')
        self.assertEqual(self.c.phase, 'RECOVER')


class DetachYawReceiptTests(unittest.TestCase):
    def setUp(self):
        import test_incline_integration as boundary
        self.b = boundary
        self.fixture = boundary.InclineBoundaryTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.n = self.fixture.node()
        self.fixture.feedback(self.n)
        f = self.n._incline_feedback_at(boundary.rospy.Time.now())
        self.assertTrue(self.n.incline.start(f, fixtures.pose(0), .49))
        self.n.incline.phase = 'HOLD'
        self.n.incline.q = self.n.incline.contact_q = attitude(15)
        self.n.state = boundary.W.STATE_WALL_HOLD
        self.fixture.feedback(self.n)

    def test_subscriber_captures_receipt_sample_before_next_control_tick(self):
        self.n._on_detach(self.b.Bool(data=True))
        self.assertTrue(self.n.detach_requested)
        self.assertEqual(self.n.incline.phase, 'HOLD')
        self.assertAlmostEqual(self.n.incline.detach_yaw_reference, math.radians(15))
        request = self.n.incline.detach_yaw_request_at
        self.fixture.clock += .02
        self.fixture.feedback(self.n)
        self.n.odom.pose.pose.orientation = self.b.Quaternion(*attitude(18))
        self.n._on_detach(self.b.Bool(data=True))  # repeated request must not reanchor
        self.n._incline_cycle(self.b.rospy.Time.now())
        self.assertEqual(self.n.incline.phase, 'RETURN')
        self.assertAlmostEqual(self.n.incline.detach_yaw_reference, math.radians(15))
        self.assertEqual(self.n.incline.detach_yaw_request_at, request)

    def test_stale_receipt_pose_requests_land_instead_of_later_reanchoring(self):
        self.n.odom.header.stamp = self.b.rospy.Time.from_sec(
            self.fixture.clock-self.n.odom_timeout_s-.01)
        self.n._on_detach(self.b.Bool(data=True))
        self.assertFalse(self.n.detach_requested)
        self.assertEqual(self.n.incline.phase, 'LAND')
        self.fixture.clock += .02
        self.fixture.feedback(self.n)
        self.n._incline_cycle(self.b.rospy.Time.now())
        self.assertTrue(self.n.abort_land_requested)
        status = self.n._incline_status(self.b.rospy.Time.now(), self.n.incline_feedback, True)
        self.assertTrue(status.abort_land_requested)
        self.assertIsNone(self.n.incline.detach_yaw_reference)


if __name__ == '__main__':
    unittest.main()
