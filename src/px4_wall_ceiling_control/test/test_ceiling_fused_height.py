"""Fused ENU height owns normal ceiling descent; pressure is diagnostic only."""
import unittest
import rospy
from mavros_msgs.msg import EstimatorStatus
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, CeilingRcAssist
import test_ceiling_motion as motion
import test_ceiling_ch12_handover as consent


class FusedHeightTests(unittest.TestCase):
    def setUp(self):
        self.fx=motion.CeilingMotionLifecycleTests()
        self.fx.extra_params={'~detach_fused_height_enabled':True}
        self.fx.setUp();self.addCleanup(self.fx.doCleanups)
        self.n=self.fx.n;self.fx.hold()
        self.assertEqual(self.n.hold_fused_height_m,2.)
        self.n.detach_requested=True
        self.fx.feed(distance=.6)
        self.assertEqual(self.n.state,S.STATE_DETACH)

    def descend(self, **kw):
        for drop in (.1,.2,.3,.4,.51):status=self.fx.feed(distance=.6,drop=drop,vz=-.15,**kw)
        return status

    def test_pressure_descent_cannot_complete_unchanged_fused_height(self):
        for _ in range(5):status=self.fx.feed(distance=.6,pressure_drop=.8)
        self.assertEqual(self.n.state,S.STATE_DETACH)
        self.assertFalse(status.detach_height_ready)
        self.assertAlmostEqual(status.descended_height_m,0.)
        self.assertAlmostEqual(status.baro_descended_height_m,.8,places=3)
        self.assertEqual(status.height_source,'px4_fused_hold_relative')

    def test_fused_descent_and_recovery_do_not_require_pressure(self):
        self.n.baro_received_at=None
        status=self.descend(baro=False)
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
        self.assertTrue(status.detach_height_ready)
        self.assertAlmostEqual(status.descended_height_m,.51,places=4)
        for _ in range(9):status=self.fx.feed(distance=.6,drop=.51,baro=False)
        self.assertTrue(status.recovery_ready)
        self.assertTrue(status.recovery_attitude_locked)
        self.assertTrue(status.height_feedback_valid)
        self.assertFalse(status.baro_fresh)
        self.assertAlmostEqual(status.fused_vertical_speed_mps,0.)
        self.assertAlmostEqual(status.hold_fused_height_m,2.)
        self.assertAlmostEqual(status.fused_height_m,1.49,places=4)

    def test_fused_recovery_keeps_manual_coordination_gate_without_pressure(self):
        self.descend(baro=False)
        self.n.rc_manual_coordination_enabled=True
        for _ in range(9):status=self.fx.feed(distance=.6,drop=.51,baro=False)
        self.assertFalse(status.recovery_ready)
        for _ in range(12):
            candidate=self.n.control_candidate_publisher.publish.call_args.args[0]
            feedback=CeilingRcAssist(maneuver_id=self.n.motion_id,
                control_sequence=candidate.control_sequence,control_phase=self.n.state,input_valid=True)
            feedback.header.stamp=rospy.Time.now();feedback.candidate_stamp=candidate.header.stamp
            self.n._on_ceiling_rc_assist(feedback)
            status=self.fx.feed(distance=.6,drop=.51,baro=False)
        self.assertTrue(status.rc_manual_feedback_ready)
        self.assertTrue(status.recovery_ready)
        self.assertTrue(status.recovery_attitude_locked)
        self.assertFalse(status.baro_fresh)

    def test_vertical_estimate_and_range_validity_are_required(self):
        for kwargs in ({'vertical_position':False},{'flow_distance':0.},{'flow_distance':float('nan')},{'frame':'local_ned'}):
            # Each invalid sample must prevent threshold confirmation even with good pressure.
            status=self.fx.feed(distance=.6,drop=.1,pressure_drop=.8,**kwargs)
            self.assertFalse(status.height_feedback_valid)
            self.assertFalse(status.detach_height_ready)
            self.assertFalse(status.recovery_ready)

    def test_vertical_reset_cannot_be_counted_as_half_metre_descent(self):
        status=self.fx.feed(distance=.6,drop=.6,vz=0.,pressure_drop=.6)
        self.assertEqual(self.n.state,S.STATE_ABORT)
        self.assertIn('高度跳变',self.n.motion_fatal)
        self.assertFalse(status.detach_height_ready)
        self.assertFalse(status.height_feedback_valid)

    def test_stale_fused_height_never_falls_back_to_pressure(self):
        self.fx.feed(distance=.6,pressure_drop=.8)
        self.assertIsNone(self.n._height_feedback(rospy.Time.now()+rospy.Duration(1.)))
        self.assertFalse(self.n._detach_height_ready(rospy.Time.now()+rospy.Duration(1.)))

    def test_hold_reference_is_not_recaptured_during_descent(self):
        baseline=self.n.hold_fused_height_m
        self.descend(pressure_drop=.0)
        for _ in range(9):status=self.fx.feed(distance=.6,drop=.51,pressure_drop=0.)
        self.assertEqual(self.n.hold_fused_height_m,baseline)
        self.assertTrue(status.recovery_attitude_locked)
        self.fx.mode='POSCTL';self.fx.feed(distance=.6,drop=.51)
        self.assertIsNone(self.n.hold_fused_height_m)


class FusedConsentTests(unittest.TestCase):
    def test_ceiling_ch12_uses_selected_height_validity_instead_of_pressure(self):
        fx=consent.CeilingConsentTests();fx.setUp();self.addCleanup(fx.doCleanups)
        fx.status(height_source='px4_fused_hold_relative',height_feedback_valid=True,height_reference_ready=True,
                  baro_fresh=False,baro_reference_ready=False)
        fx.fx.press()
        self.assertGreater(fx.n.pilot_handover_request_time.to_sec(),0.)

    def test_manager_checks_selected_height_and_its_own_vertical_estimator(self):
        fx=consent.ManagerConsentTests();fx.setUp();m=fx.m
        m.ceiling.height_source='px4_fused_hold_relative'
        m.ceiling.height_feedback_valid=m.ceiling.height_reference_ready=True
        m.ceiling.baro_fresh=m.ceiling.baro_reference_ready=False
        m.estimator=EstimatorStatus(pos_vert_abs_status_flag=True)
        m.operator.pilot_handover_request_time=consent.at(9.5)
        self.assertTrue(m._ceiling_baro_handover_ready(fx.now))
        m.estimator.pos_vert_abs_status_flag=False
        self.assertFalse(m._ceiling_baro_handover_ready(fx.now))
        m.estimator.pos_vert_abs_status_flag=True;m.ceiling.height_feedback_valid=False
        self.assertFalse(m._ceiling_baro_handover_ready(fx.now))


if __name__=='__main__':unittest.main()
