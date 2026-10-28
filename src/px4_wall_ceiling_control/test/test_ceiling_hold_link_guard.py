"""Indefinite operator HOLD and observed detach-command failure boundaries."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import json
import unittest
from unittest import mock
import rospy
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, OperatorCommand as Op
import test_ceiling_motion as motion
import test_ceiling_hold_abort as landing
import test_ceiling_fault_land as guard_fixtures


class ExplicitHoldTests(unittest.TestCase):
    def setUp(self):
        self.fx = motion.CeilingMotionLifecycleTests(methodName='runTest')
        self.fx.extra_params = {'~hold_timeout_s': 0., '~hold_require_explicit_detach': True, '~abort_land_on_fault': True}
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.fx.hold(); self.n = self.fx.n

    def test_hold_has_no_five_second_deadline(self):
        for _ in range(1200):
            s = self.fx.feed(distance=.055)
        self.assertEqual(self.n.state, S.STATE_SURFACE_HOLD)
        self.assertGreater(s.hold_elapsed_s, 100.)
        self.assertEqual(s.hold_timeout_s, 0.)
        self.assertFalse(s.hold_timed_out)
        self.assertFalse(s.abort_land_requested)

    def test_explicit_detach_after_long_hold_uses_normal_path(self):
        for _ in range(80): self.fx.feed(distance=.055)
        self.n._on_detach(Bool(data=True))
        s = self.fx.feed(distance=.055)
        self.assertEqual(s.state, S.STATE_DETACH)
        self.assertFalse(s.abort_land_requested)
        self.fx.feed(distance=.6, drop=.51)
        self.assertEqual(self.n.state, S.STATE_RECOVERY_HOVER)

    def test_recovery_uses_fused_feedback_without_requiring_fresh_top_range(self):
        self.n._on_detach(Bool(data=True))
        self.fx.feed(distance=.055)
        self.fx.feed(distance=.6, drop=.51)
        for _ in range(10):
            status = self.fx.feed(distance=255., drop=.51)
        self.assertEqual(self.n.state, S.STATE_RECOVERY_HOVER)
        self.assertFalse(status.range_fresh)
        self.assertTrue(status.recovery_ready)
        self.assertFalse(status.abort_land_requested)

    def test_command_feedback_loss_aborts_instead_of_starting_normal_detach(self):
        self.n.operator_received_at = rospy.Time.from_sec(self.fx.clock-1.)
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertTrue(self.n._landing_abort())
        self.assertIn('指令反馈失效', self.n.last_reason)

    def test_cancel_during_hold_is_an_abort_not_normal_detach(self):
        self.n._on_operator(Op(mode=Op.MODE_STANDBY, enable_control=False))
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertTrue(self.n._landing_abort())

    def test_delayed_or_duplicate_detach_cannot_arm_a_future_hold(self):
        self.n.state = S.STATE_DETACH
        self.n._on_detach(Bool(data=True))
        self.assertFalse(self.n.detach_requested)

    def test_safety_fault_still_outranks_detach(self):
        self.n._on_detach(Bool(data=True))
        self.n._tilt_fault = lambda now: True
        self.fx.feed(distance=.055)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertTrue(self.n._landing_abort())


class HoldManagerWatchdogTests(unittest.TestCase):
    def setUp(self):
        self.fx = landing.CeilingLandingManagerTests(methodName='runTest')
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.n = n = self.fx.n
        self.clock = 10.
        patch = mock.patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.clock))
        patch.start(); self.addCleanup(patch.stop)
        n.ceiling_explicit_detach = True
        n.operator_timeout_s = n.status_timeout_s = .5
        n.ceiling = S(state=S.STATE_SURFACE_HOLD)
        n.cancel_pub = mock.Mock()
        n.operator = Op(mode=Op.MODE_CEILING, enable_control=True, source='terminal', reason='terminal request')
        self.refresh()

    def refresh(self):
        now=rospy.Time.now()
        self.n.ceiling.header.stamp=self.n.ceiling_at=now
        self.n.operator.header.stamp=self.n.operator_at=now
        self.n.flight.header.stamp=self.n.flight_at=now
        return now

    def test_healthy_hold_does_not_create_landing_request(self):
        for _ in range(30):
            self.clock += 1.
            self.assertFalse(self.n._ceiling_hold_command_fault(self.refresh()))
        self.n._request_mode.assert_not_called()

    def test_missing_detach_pulse_detected_from_persistent_operator_intent(self):
        n=self.n;n.operator.reason='detach pending'
        self.assertFalse(n._ceiling_hold_command_fault(self.refresh()))
        self.clock += .9; self.assertFalse(n._ceiling_hold_command_fault(self.refresh()))
        self.clock += .11; now=self.refresh()
        reason=n._ceiling_hold_command_fault(now)
        self.assertIn('未进入DETACH',reason)
        self.assertTrue(n._tick_ceiling_abort_land(now,reason))
        self.assertEqual(n.phase,'CEILING_LANDING')
        n._request_mode.assert_called_once_with('AUTO.LAND')
        n._send_mode_request('AUTO.LAND',attempt=3)
        self.assertEqual(n.set_mode.call_args.args[0].custom_mode,'AUTO.LAND')
        n.cancel_pub.publish.assert_called_once()
        self.assertFalse(n.ceiling_land_confirmed)

    def test_detach_feedback_clears_the_ack_watchdog(self):
        self.n.operator.reason='detach pending'
        self.n._ceiling_hold_command_fault(self.refresh())
        self.clock += 2.;self.n.ceiling.state=S.STATE_DETACH
        self.assertFalse(self.n._ceiling_hold_command_fault(self.refresh()))
        self.assertIsNone(self.n.ceiling_detach_pending_at)

    def test_stale_behavior_feedback_can_land_without_faking_a_behavior_status(self):
        n=self.n; self.clock += .6
        n.operator.header.stamp=n.operator_at=n.flight.header.stamp=n.flight_at=rospy.Time.now()
        reason=n._ceiling_hold_command_fault(rospy.Time.now())
        self.assertIn('状态反馈失效',reason)
        n._tick_ceiling_abort_land(rospy.Time.now(),reason)
        self.assertEqual(n.ceiling.state,S.STATE_SURFACE_HOLD)
        self.assertEqual(n.phase,'CEILING_LANDING')
        n._request_mode.assert_called_once_with('AUTO.LAND')
        self.assertFalse(n._ceiling_abort_land_live(rospy.Time.now(),attempt=2))

    def test_stale_or_released_operator_intent_is_a_fault(self):
        n=self.n;n.operator_at=rospy.Time.from_sec(8.)
        self.assertIn('指令反馈失效',n._ceiling_hold_command_fault(rospy.Time.now()))
        self.refresh();n.operator.enable_control=False
        self.assertIn('指令反馈失效',n._ceiling_hold_command_fault(rospy.Time.now()))

    def test_pilot_mode_change_and_kill_keep_their_priority(self):
        n=self.n;n.flight.mode='POSCTL'
        n._tick_ceiling_abort_land(rospy.Time.now(),'test command failure')
        n._request_mode.assert_not_called()
        n._reset.assert_called_once()


class GuardReadbackStatusTests(unittest.TestCase):
    def setUp(self):
        self.fx=guard_fixtures.GroundFailsafeGuardTests(methodName='runTest')
        self.fx.setUp();self.addCleanup(self.fx.doCleanups)
        self.n=self.fx.n
        patch=mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.))
        patch.start();self.addCleanup(patch.stop)
        self.n.ready=self.n.detail=self.n.status=mock.Mock()
        self.n.reason='pending'

    def test_ready_and_ui_report_require_actual_two_second_readback(self):
        self.fx.run_once()
        self.assertTrue(self.n.verified)
        self.assertEqual(self.n.readbacks['COM_OF_LOSS_T']['actual'],2.0)
        self.n.publish(None)
        payload=json.loads(self.n.status.publish.call_args.args[0].data)
        self.assertTrue(payload['verified'])
        self.assertEqual(payload['parameters']['COM_OF_LOSS_T']['expected'],2.0)
        self.assertEqual(payload['parameters']['COM_OBL_RC_ACT']['actual'],4)

    def test_failed_readback_is_visible_and_never_verified(self):
        self.n.transport._exchange.side_effect=lambda name: 4 if name=='COM_OBL_RC_ACT' else .2
        self.fx.run_once();self.n.publish(None)
        payload=json.loads(self.n.status.publish.call_args.args[0].data)
        self.assertFalse(payload['verified'])
        self.assertEqual(payload['parameters']['COM_OF_LOSS_T']['actual'],.2)
        self.assertIn('mismatch',payload['reason'])

    def test_two_second_link_window_accepts_one_hz_heartbeat(self):
        self.n.state=State(connected=True, armed=False)
        self.n.state.header.stamp=self.n.received=rospy.Time.from_sec(8.)
        self.assertTrue(type(self.n).connected(self.n))
        self.n.state.header.stamp=self.n.received=rospy.Time.from_sec(7.99)
        self.assertFalse(type(self.n).connected(self.n))

    def test_old_readbacks_cannot_remain_verified_after_disconnect(self):
        self.fx.run_once();self.n.connected.return_value=False;self.n.publish(None)
        payload=json.loads(self.n.status.publish.call_args.args[0].data)
        self.assertFalse(payload['verified'])
        self.assertFalse(payload['connected'])


if __name__ == '__main__': unittest.main()
