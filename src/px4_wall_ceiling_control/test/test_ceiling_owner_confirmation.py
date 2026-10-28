"""Output arbitration must acknowledge this attempt before ascent starts."""
import unittest
import rospy
from std_msgs.msg import UInt8
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S
import test_ceiling_console_flow as fixtures


class CeilingOwnerConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.CeilingConsoleFlowTests(methodName='runTest')
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.n = self.f.n
        self.f.sample(10., .5)
        self.n.motion_owner = 1
        self.n.motion_owner_at = rospy.Time.now()
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state, S.STATE_CEILING_ARMED)

    def tick(self, t, owner=None, distance=.5):
        self.f.sample(t, distance)
        self.f.odom(2.)
        if owner is not None:
            self.n._on_ceiling_owner(UInt8(data=owner))
        self.n._update_state_machine(rospy.Time.now())
        return self.n._build_control_candidate(self.n._build_status(rospy.Time.now(), .02))

    def test_wait_continues_request_and_never_climbs_without_owner(self):
        for t, owner in ((10.1, None), (10.2, 0), (10.3, 2), (15.9, None)):
            c = self.tick(t, owner)
            self.assertEqual(self.n.state, S.STATE_CEILING_ARMED)
            self.assertTrue(c.acquire_request)
            self.assertTrue(c.keep_ownership)
            self.assertTrue(c.valid)
            self.assertEqual(c.velocity_z_enu_mps, 0.)

    def test_only_fresh_receipt_after_this_attempt_allows_approach(self):
        for receipt in (9.9, 10., 10.1, 10.6):
            self.n.motion_owner = 1
            self.n.motion_owner_at = rospy.Time.from_sec(receipt)
            self.tick(10.5)
            self.assertEqual(self.n.state, S.STATE_CEILING_ARMED)
        self.tick(10.6, 1)
        self.assertEqual(self.n.state, S.STATE_APPROACH)

    def test_timeout_withdraws_request_and_latches_until_release(self):
        c = self.tick(16., 1)
        self.assertEqual(self.n.state, S.STATE_NORMAL_FLIGHT)
        self.assertTrue(self.n.rearm_required)
        self.assertFalse(c.acquire_request)
        self.assertFalse(c.keep_ownership)
        self.assertFalse(c.valid)
        self.tick(16.1, 1)
        self.assertEqual(self.n.state, S.STATE_NORMAL_FLIGHT)
        self.n._mode_requested = self.n._arm_requested = lambda *a: False
        self.tick(16.2)
        self.assertFalse(self.n.rearm_required)
        self.n._mode_requested = self.n._arm_requested = lambda *a: True
        self.tick(16.3)
        self.assertEqual(self.n.state, S.STATE_CEILING_ARMED)
        self.tick(16.4)
        self.assertEqual(self.n.state, S.STATE_CEILING_ARMED)
        self.tick(16.5, 1)
        self.assertEqual(self.n.state, S.STATE_APPROACH)

    def test_invalid_range_cannot_extend_owner_deadline(self):
        self.tick(15., distance=255.)
        c = self.tick(16., distance=255.)
        self.assertEqual(self.n.state, S.STATE_NORMAL_FLIGHT)
        self.assertFalse(c.acquire_request)
        self.assertFalse(c.keep_ownership)

    def test_release_cancels_before_acknowledgment(self):
        self.n._mode_requested = self.n._arm_requested = lambda *a: False
        c = self.tick(10.1)
        self.assertEqual(self.n.state, S.STATE_NORMAL_FLIGHT)
        self.assertFalse(c.acquire_request)
        self.assertFalse(c.keep_ownership)


class CeilingMotionOwnerConfirmationTests(unittest.TestCase):
    def setUp(self):
        import test_ceiling_motion as lifecycle
        self.f = lifecycle.CeilingMotionLifecycleTests(methodName='runTest')
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        for _ in range(14):
            self.f.feed()
        self.f.mode = 'OFFBOARD'
        self.f.feed(owner=None)
        self.assertEqual(self.f.n.state, S.STATE_CEILING_ARMED)

    def test_motion_candidate_keeps_requesting_without_climbing_until_ack(self):
        for _ in range(20):
            status = self.f.feed(owner=0)
            candidate = self.f.n.control_candidate_publisher.publish.call_args.args[0]
            self.assertEqual(status.state, S.STATE_CEILING_ARMED)
            self.assertTrue(candidate.acquire_request)
            self.assertTrue(candidate.valid)
            self.assertEqual(self.f.n.motion_last_target_vz, 0.)
        self.f.feed(owner=1)
        self.assertEqual(self.f.n.state, S.STATE_APPROACH)

    def test_missing_owner_cancels_motion_without_automatic_landing(self):
        for _ in range(65):
            status = self.f.feed(owner=None)
        self.assertEqual(status.state, S.STATE_NORMAL_FLIGHT)
        self.assertFalse(status.abort_land_requested)
        self.assertFalse(status.acquire_request)
        self.assertFalse(status.keep_ownership)
        self.assertTrue(self.f.n.rearm_required)


if __name__ == '__main__':
    unittest.main()
