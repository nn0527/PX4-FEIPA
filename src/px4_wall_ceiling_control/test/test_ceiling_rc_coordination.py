"""Manual trim/position control coordination, with mocked ROS transports."""
import copy
import math
import unittest
from dataclasses import replace
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget
from std_msgs.msg import UInt8
from px4_wall_ceiling_control.msg import CeilingRcAssist, CeilingAttachmentStatus as S, AttachmentControlCandidate as C
from px4_wall_ceiling_control.domain.ceiling_motion import CeilingMotionController, CeilingMotionSettings
import test_ceiling_motion as fixtures
import test_ceiling_rc_assist as rc_fixtures


def at(seconds):
    return rospy.Time.from_sec(seconds)


class ManualCoordinationTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.CeilingMotionLifecycleTests()
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.fx.takeover()
        self.n = self.fx.n
        self.n.rc_manual_coordination_enabled = True

    def feedback(self, active=False, trim=0., valid=True):
        n = self.n
        message = CeilingRcAssist(maneuver_id=n.motion_id, control_sequence=n.motion_sequence,
            control_phase=n.state, input_valid=valid, stick_active=active,
            roll_trim_rad=trim, pitch_trim_rad=-trim)
        message.header.stamp = rospy.Time.now()
        message.candidate_stamp = n.control_candidate_publisher.publish.call_args.args[0].header.stamp
        n._on_ceiling_rc_assist(message)
        return message

    def tick(self, active=False, trim=0., **kw):
        self.feedback(active, trim)
        return self.fx.feed(dt=.05, **kw)

    def test_manual_input_clears_integral_blends_pull_and_keeps_original_reference(self):
        self.n.motion_controller.horizontal.integral = (.10, -.10)
        for _ in range(10):
            status = self.tick(active=True, trim=.01, x=.20)
        self.assertTrue(status.rc_manual_active)
        self.assertTrue(status.rc_manual_feedback_ready)
        self.assertAlmostEqual(status.rc_position_weight, .25)
        self.assertEqual(self.n.motion_controller.horizontal.integral, (0., 0.))
        self.assertEqual(self.n.motion_controller.xy, (0., 0.))
        self.assertLess(self.n.motion_controller.horizontal.acceleration[0], 0.)
        self.assertFalse(self.n._ceiling_manual_settled())

    def test_centering_blends_back_before_integrator_can_resume(self):
        for _ in range(10): self.tick(active=True)
        weight = self.n.motion_rc_weight
        status = self.tick()
        self.assertGreater(status.rc_position_weight, weight)
        self.assertLess(status.rc_position_weight, 1.)
        self.assertFalse(self.n._ceiling_manual_settled())
        for _ in range(10): status = self.tick()
        self.assertAlmostEqual(status.rc_position_weight, 1.)
        self.assertTrue(self.n._ceiling_manual_settled())
        self.assertEqual(self.n.motion_controller.xy, (0., 0.))

    def test_integral_stays_frozen_until_manual_blend_finishes(self):
        for active, count in ((True, 12), (False, 5), (False, 20)):
            for _ in range(count):
                output = AttitudeTarget(thrust=.53, orientation=Quaternion(w=1.))
                output.header.stamp = rospy.Time.now()
                self.n._on_ceiling_owner(UInt8(data=1))
                self.n._on_ceiling_output(output)
                self.tick(active=active, x=.20)
                if not self.n._ceiling_manual_settled():
                    self.assertEqual(self.n.motion_controller.horizontal.integral, (0., 0.))
        self.assertTrue(self.n._ceiling_manual_settled())
        self.assertLess(self.n.motion_controller.horizontal.integral[0], 0.)

    def test_residual_trim_counts_as_manual_after_stick_centering(self):
        status = self.tick(active=False, trim=math.radians(.10))
        self.assertTrue(status.rc_manual_active)
        self.assertFalse(self.n._ceiling_manual_settled())

    def test_drift_warning_restores_full_position_pull_without_rebasing(self):
        for _ in range(10): self.tick(active=True, x=.20)
        for _ in range(10): status = self.tick(active=True, x=.35)
        self.assertAlmostEqual(status.rc_position_weight, 1.)
        self.assertEqual(self.n.motion_controller.xy, (0., 0.))
        self.assertEqual(self.n.motion_controller.horizontal.integral, (0., 0.))

    def test_stale_feedback_restores_pull_and_cannot_qualify_as_settled(self):
        for _ in range(10): self.tick(active=True)
        for _ in range(15): status = self.fx.feed(dt=.05)
        self.assertFalse(status.rc_manual_feedback_ready)
        self.assertAlmostEqual(status.rc_position_weight, 1.)
        self.assertFalse(self.n._ceiling_manual_settled())

    def test_invalid_rc_cannot_qualify_as_settled(self):
        self.feedback(valid=False)
        status = self.fx.feed(dt=.05)
        self.assertFalse(status.rc_manual_feedback_ready)
        self.assertFalse(self.n._ceiling_manual_settled())

    def test_feedback_rejects_other_attempt_phase_future_sequence_and_stale_source(self):
        original = self.feedback(active=True)
        for field, value in [('maneuver_id', 'previous'), ('control_phase', S.STATE_SURFACE_HOLD),
                             ('control_sequence', self.n.motion_sequence+1), ('roll_trim_rad', float('nan'))]:
            message = copy.deepcopy(original); setattr(message, field, value)
            self.n._on_ceiling_rc_assist(message)
            self.assertIs(self.n.motion_rc_feedback, original)
        for future in (False, True):
            message = copy.deepcopy(original)
            message.header.stamp = at(self.fx.clock+(.01 if future else -1.))
            self.n._on_ceiling_rc_assist(message)
            self.assertIs(self.n.motion_rc_feedback, original)
        message = copy.deepcopy(original); message.candidate_stamp = at(self.fx.clock-1.)
        self.n._on_ceiling_rc_assist(message)
        self.assertIs(self.n.motion_rc_feedback, original)

    def test_feedback_replay_cannot_refresh_receipt_age(self):
        original = self.feedback()
        receipt = self.n.motion_rc_feedback_at
        self.fx.clock += .10
        self.n._on_ceiling_rc_assist(original)
        self.assertEqual(self.n.motion_rc_feedback_at, receipt)

    def test_recovery_needs_centered_feedback_and_fresh_stability_samples(self):
        self.n.rc_manual_coordination_enabled = False
        self.fx.hold()
        self.n.detach_requested = True
        self.fx.feed(distance=.055)
        self.fx.feed(distance=.6, drop=.51)
        self.assertEqual(self.n.state, S.STATE_RECOVERY_HOVER)
        self.n.rc_manual_coordination_enabled = True
        for _ in range(20): status = self.tick(active=True, distance=.6, drop=.51)
        self.assertFalse(status.recovery_ready)
        for _ in range(5): status = self.tick(distance=.6, drop=.51)
        self.assertFalse(status.recovery_ready)
        for _ in range(22): status = self.tick(distance=.6, drop=.51)
        self.assertTrue(status.recovery_ready)

    def test_detach_blocks_trim_until_clearance_and_release_confirmed(self):
        self.n.rc_manual_coordination_enabled = False
        self.fx.hold(); self.n.detach_requested = True
        status = self.fx.feed(distance=.055)
        self.assertTrue(self.n._build_control_candidate(status).ceiling_rc_assist_blocked)
        for _ in range(5): status = self.fx.feed(distance=.15, drop=.1)
        self.assertTrue(self.n.motion_contact_released)
        self.assertFalse(self.n._build_control_candidate(status).ceiling_rc_assist_blocked)
        self.n._mechanism_release_confirmed = lambda now: False
        self.assertTrue(self.n._build_control_candidate(status).ceiling_rc_assist_blocked)

    def test_abort_blocks_trim_and_retains_existing_fault_support(self):
        self.n._enter_state(S.STATE_ABORT, rospy.Time.now(), 'test abort')
        status = self.fx.feed()
        candidate = self.n._build_control_candidate(status)
        self.assertTrue(candidate.ceiling_rc_assist_blocked)
        self.assertFalse(candidate.ceiling_abort_land_requested)
        self.assertTrue(status.keep_ownership)


class ManualControllerTests(unittest.TestCase):
    def test_reduced_position_pull_keeps_velocity_damping(self):
        s = CeilingMotionSettings()
        automatic, manual = CeilingMotionController(s), CeilingMotionController(s)
        for c in (automatic, manual): c.capture((0., 0.), 0., (0., 0., 1.), .53)
        for _ in range(30):
            automatic.attitude((.2, 0.), (0., 0.), .05)
            manual.attitude((.2, 0.), (0., 0.), .05, position_weight=.25)
        self.assertAlmostEqual(manual.horizontal.target[0], automatic.horizontal.target[0]*.25)
        manual.attitude((.05, 0.), (.15, 0.), .05, position_weight=.25)
        self.assertLess(manual.horizontal.acceleration[0], 0.)

    def test_invalid_manual_parameters_are_rejected(self):
        for values in ({'rc_manual_position_weight': 1.1}, {'rc_manual_blend_s': 0.},
                       {'rc_manual_feedback_timeout_s': float('nan')}):
            with self.assertRaises(ValueError): replace(CeilingMotionSettings(), **values)


class ManualOutputTests(unittest.TestCase):
    def test_atomic_block_gate_clears_calibrated_trim(self):
        n, now = rc_fixtures.CeilingRcAssistTest().fixture()
        n.rc_roll_rad = n.rc_pitch_rad = .02
        candidate = C(source=C.SOURCE_CEILING, phase_valid=True, control_phase=S.STATE_DETACH,
                      ceiling_rc_assist_blocked=True)
        base = Quaternion(w=1.)
        self.assertEqual(n._assisted_attitude(now, base, C.SOURCE_CEILING, candidate), base)
        self.assertEqual((n.rc_roll_rad, n.rc_pitch_rad), (0., 0.))

    def test_feedback_matches_actual_published_candidate_and_rejection_publishes_none(self):
        fx = fixtures.CeilingFinalOutputTests(); fx.setUp(); self.addCleanup(fx.doCleanups)
        n, candidate = fx.n, fx.c
        n.ceiling_rc_feedback_publisher = mock.Mock()
        n._on_ceiling_candidate(candidate)
        n._publish_candidate(fx.now, candidate)
        feedback = n.ceiling_rc_feedback_publisher.publish.call_args.args[0]
        self.assertEqual(feedback.maneuver_id, candidate.maneuver_id)
        self.assertEqual(feedback.control_sequence, candidate.control_sequence)
        self.assertEqual(feedback.candidate_stamp, candidate.header.stamp)
        self.assertTrue(feedback.input_valid)  # Assist disabled is known zero input.
        self.assertFalse(feedback.stick_active)
        n.ceiling_rc_feedback_publisher.reset_mock()
        n.ceiling_candidate_snapshot = (candidate, at(9.))
        n._publish_candidate(fx.now, candidate)
        n.ceiling_rc_feedback_publisher.publish.assert_not_called()

    def test_feedback_reports_both_stick_and_decaying_trim(self):
        n, now = rc_fixtures.CeilingRcAssistTest().fixture()
        n.ceiling_rc_feedback_publisher = mock.Mock()
        candidate = C(source=C.SOURCE_CEILING, phase_valid=True, control_phase=S.STATE_APPROACH,
                      maneuver_id='ceiling-test', control_sequence=1)
        candidate.header.stamp = now
        n._assisted_attitude(now, Quaternion(w=1.), C.SOURCE_CEILING, candidate)
        target = mock.Mock(); target.header.stamp = now
        n._publish_ceiling_rc_feedback(candidate, target)
        feedback = n.ceiling_rc_feedback_publisher.publish.call_args.args[0]
        self.assertTrue(feedback.input_valid)
        self.assertTrue(feedback.stick_active)
        n.rc_channels = (1500, 1500, 9999, 9999)
        n.rc_roll_rad = .01
        n._publish_ceiling_rc_feedback(candidate, target)
        feedback = n.ceiling_rc_feedback_publisher.publish.call_args.args[0]
        self.assertFalse(feedback.stick_active)
        self.assertEqual(feedback.roll_trim_rad, .01)


if __name__ == '__main__':
    unittest.main()
