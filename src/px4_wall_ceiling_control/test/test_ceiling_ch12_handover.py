"""Ceiling recovery holds attitude until a new CH12 edge; no live transports."""
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import Quaternion
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, OperatorCommand as Op, AttachmentControlCandidate as C
import test_ceiling_motion as motion
import test_rc_wall_toggle as terminal
import test_flight_handover as manager


def at(t): return rospy.Time.from_sec(t)


class RecoveryHoldTests(unittest.TestCase):
    def setUp(self):
        self.fx = motion.CeilingMotionLifecycleTests()
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n
        self.fx.hold(); self.n.detach_requested = True
        self.fx.feed(distance=.055)
        status = self.fx.feed(distance=.6, drop=.51, vz=-.4)
        self.assertFalse(status.recovery_attitude_locked)

    def stable(self):
        for _ in range(8): status = self.fx.feed(distance=.6, drop=.51, tilt=.03)
        self.assertTrue(status.recovery_ready)
        self.assertTrue(status.recovery_attitude_locked)
        return status

    def test_hold_freezes_attitude_but_keeps_height_feedback_and_ownership(self):
        status = self.stable()
        locked, stamp = self.n.motion_recovery_attitude, status.recovery_lock_time
        for _ in range(10):
            status = self.fx.feed(distance=.6, drop=.48, tilt=.05, x=.10, vx=.05)
            self.assertEqual(tuple(getattr(status.attitude_setpoint,k) for k in 'xyzw'), locked)
            self.assertTrue(status.keep_ownership)
            self.assertEqual(status.recovery_lock_time, stamp)
            self.assertEqual(self.n.motion_controller.horizontal.integral, (0.,0.))
        above = status.target_vertical_speed_mps
        for _ in range(10): status = self.fx.feed(distance=.6, drop=.53)
        self.assertGreater(status.target_vertical_speed_mps, above)
        candidate = self.n.control_candidate_publisher.publish.call_args.args[0]
        self.assertTrue(candidate.ceiling_rc_assist_blocked)

    def test_healthy_wait_does_not_timeout_or_auto_release_after_fifteen_seconds(self):
        self.stable()
        for _ in range(170): status = self.fx.feed(distance=.6,drop=.51,tilt=.03)
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
        self.assertTrue(status.recovery_attitude_locked)
        self.assertTrue(status.keep_ownership)
        self.assertFalse(status.fault_detected)

    def test_unstable_feedback_cannot_reanchor_or_authorize_handover(self):
        status = self.stable()
        locked, stamp = self.n.motion_recovery_attitude, status.recovery_lock_time
        status = self.fx.feed(distance=.6, drop=.60, tilt=.05)
        self.assertFalse(status.recovery_ready)
        self.assertTrue(status.recovery_attitude_locked)
        self.assertEqual(self.n.motion_recovery_attitude, locked)
        for _ in range(9): status = self.fx.feed(distance=.6, drop=.51, tilt=.02)
        self.assertTrue(status.recovery_ready)
        self.assertEqual(status.recovery_lock_time, stamp)
        self.assertEqual(self.n.motion_recovery_attitude, locked)

    def test_fault_and_new_cycle_clear_the_lock(self):
        self.stable()
        self.n._enter_state(S.STATE_ABORT,rospy.Time.now(),'test fault')
        self.assertIsNone(self.n.motion_recovery_attitude)
        self.assertEqual(self.n.motion_recovery_lock_time,at(0))


class CeilingConsentTests(unittest.TestCase):
    def setUp(self):
        self.fx = terminal.MixedOperatorTest()
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.n = n = self.fx.node
        n.mode=Op.MODE_CEILING; n.detaching=True; n.flight_mode='OFFBOARD'
        n._on_state(State(connected=True,armed=True,mode="OFFBOARD"))
        n.pilot_handover_request_time=at(0)
        self.status()

    def status(self, **values):
        c=S(state=S.STATE_RECOVERY_HOVER,recovery_ready=True,recovery_attitude_locked=True,
            keep_ownership=True,odom_fresh=True,baro_fresh=True,baro_reference_ready=True,detach_height_ready=True)
        c.header.stamp=at(self.fx.now);c.recovery_lock_time=at(self.fx.now-.1)
        for k,v in values.items():setattr(c,k,v)
        self.n._on_ceiling_status(c);self.n.flight_at=at(self.fx.now)
        return c

    def test_early_edge_not_queued_and_third_edge_independent_of_ch8_selection(self):
        self.status(recovery_attitude_locked=False,recovery_ready=False)
        self.fx.press();self.assertEqual(self.n.pilot_handover_request_time,at(0))
        self.status()
        self.n.rc_mode_selector_enabled=True
        self.n.rc_mode_channel_index=7;self.n.mode_selector=terminal.AttachmentModeSelector()
        self.n.rc_selected_mode=None;self.n.selected_mode_pub=mock.Mock()
        self.fx.press();event=self.n.pilot_handover_request_time
        self.assertGreater(event.to_sec(),0)
        self.n.ceiling_detach_pub.publish.assert_not_called()
        self.status();self.fx.press()
        self.assertEqual(self.n.pilot_handover_request_time,event)
        self.n._on_state(State(connected=True,armed=True,mode='POSCTL'))
        self.assertEqual(self.n.pilot_handover_request_time,at(0))

    def test_stale_unstable_kill_and_future_lock_cannot_grant_consent(self):
        for values in ({'recovery_ready':False},{'fault_detected':True},{'detach_failed':True},
                       {'recovery_lock_time':at(20)},{'header':type(self.status().header)(stamp=at(1))}):
            self.status(**values);self.n._on_wall_toggle()
            self.assertEqual(self.n.pilot_handover_request_time,at(0))
        self.status();self.n.kill_guard_enabled=True;self.n._kill_report=lambda:{'valid':True,'active':True}
        self.n._on_wall_toggle();self.assertEqual(self.n.pilot_handover_request_time,at(0))


class ManagerConsentTests(unittest.TestCase):
    def setUp(self):
        self.now=at(10)
        self.m=m=manager.Manager.__new__(manager.Manager)
        m.ceiling=S(state=S.STATE_RECOVERY_HOVER,recovery_ready=True,recovery_attitude_locked=True,
            detach_height_ready=True,baro_fresh=True,baro_reference_ready=True)
        m.ceiling.header.stamp=self.now;m.ceiling.recovery_lock_time=at(9)
        m.ceiling_at=self.now;m.status_timeout_s=.5;m.operator_timeout_s=.5
        m.ceiling_baro_handover_enabled=True;m.selected_mode=Op.MODE_CEILING
        m._ceiling_position_ready=lambda now:True
        m.operator=Op(mode=Op.MODE_CEILING,enable_control=True,source='terminal')
        m.operator.header.stamp=self.now;m.operator_at=self.now

    def test_manager_needs_later_consent_and_all_live_recovery_conditions(self):
        m=self.m
        for event in (0,8.9,9,11):
            m.operator.pilot_handover_request_time=at(event)
            self.assertFalse(m._ceiling_baro_handover_ready(self.now))
        m.operator.pilot_handover_request_time=at(9.5)
        self.assertTrue(m._ceiling_baro_handover_ready(self.now))
        for field in ('recovery_ready','recovery_attitude_locked','detach_height_ready','baro_fresh','baro_reference_ready'):
            setattr(m.ceiling,field,False);self.assertFalse(m._ceiling_baro_handover_ready(self.now));setattr(m.ceiling,field,True)
        m.operator_at=at(1);self.assertFalse(m._ceiling_baro_handover_ready(self.now))
        m.operator_at=self.now;m.operator.mode=Op.MODE_WALL
        self.assertFalse(m._ceiling_baro_handover_ready(self.now))


if __name__=='__main__':unittest.main()
