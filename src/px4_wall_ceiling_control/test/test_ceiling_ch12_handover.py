"""Ceiling recovery requires fresh pilot confirmation; no live transports."""
import json
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

    def test_wait_keeps_xy_feedback_and_ownership_after_readiness_lock(self):
        status = self.stable()
        locked, stamp = self.n.motion_recovery_attitude, status.recovery_lock_time
        for _ in range(10):
            status = self.fx.feed(distance=.6, drop=.48, tilt=.05, x=.30, vx=.15)
            self.assertTrue(status.keep_ownership)
            self.assertEqual(status.recovery_lock_time, stamp)
        from px4_wall_ceiling_control.domain.geometry import rotate_vector
        self.assertLess(rotate_vector(tuple(getattr(status.attitude_setpoint,k) for k in 'xyzw'), (0.,0.,1.))[0], 0.)
        self.assertNotEqual(tuple(getattr(status.attitude_setpoint,k) for k in 'xyzw'), locked)
        self.assertEqual(self.n.motion_controller.xy, (0., 0.))
        above = status.target_vertical_speed_mps
        for _ in range(10): status = self.fx.feed(distance=.6, drop=.58)
        self.assertGreater(status.target_vertical_speed_mps, above)
        candidate = self.n.control_candidate_publisher.publish.call_args.args[0]
        self.assertFalse(candidate.ceiling_rc_assist_blocked)

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


class RecoveryBandTests(unittest.TestCase):
    def setUp(self):
        self.fx = motion.CeilingMotionLifecycleTests()
        self.fx.extra_params = {'~detach_fused_height_enabled': True,
            '~detach_height_velocity_enabled': True, '~detach_drop_m': .4,
            '~detach_height_tolerance_m': .1, '~recovery_height_tolerance_m': .1}
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n
        self.fx.hold(); self.n.detach_requested = True
        self.fx.feed(distance=.6)

    def test_thirty_to_fifty_cm_can_enter_recovery_and_wait_without_centre_pull(self):
        for drop in (.1, .2): self.fx.feed(distance=.6, drop=drop, vz=-.1)
        for _ in range(15): status = self.fx.feed(distance=.6, drop=.31)
        self.assertEqual(self.n.state, S.STATE_RECOVERY_HOVER)
        self.assertTrue(status.recovery_ready)
        stamp = status.recovery_lock_time
        for drop in (.30, .304, .4, .496, .50):
            for _ in range(12): status = self.fx.feed(distance=.6, drop=drop)
            self.assertTrue(status.recovery_ready)
            self.assertAlmostEqual(status.target_vertical_speed_mps, 0.)
            self.assertEqual(status.recovery_lock_time, stamp)
            self.assertTrue(status.keep_ownership)
        for drop in (.29, .51):
            for _ in range(12): status = self.fx.feed(distance=.6, drop=drop)
            self.assertFalse(status.recovery_ready)
            self.assertTrue(status.keep_ownership)
            if drop < .3: self.assertLess(status.target_vertical_speed_mps, 0.)
            else: self.assertGreater(status.target_vertical_speed_mps, 0.)

    def test_height_band_does_not_authorize_drifting_or_moving_handover(self):
        for drop in (.1, .2, .3): self.fx.feed(distance=.6, drop=drop, vz=-.1)
        for _ in range(15): self.fx.feed(distance=.6, drop=.4)
        for _ in range(12): status = self.fx.feed(distance=.6, drop=.4, vx=.3)
        self.assertFalse(status.recovery_ready)
        self.assertTrue(status.keep_ownership)


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

    def test_early_edge_not_queued_and_third_edge_independent_of_start_target(self):
        self.status(recovery_attitude_locked=False,recovery_ready=False)
        self.fx.press();self.assertEqual(self.n.pilot_handover_request_time,at(0))
        self.status()
        self.fx.press();event=self.n.pilot_handover_request_time
        self.assertGreater(event.to_sec(),0)
        self.n.ceiling_detach_pub.publish.assert_not_called()
        self.status();self.fx.press()
        self.assertEqual(self.n.pilot_handover_request_time,event)
        self.n._on_state(State(connected=True,armed=True,mode='POSCTL'))
        self.assertEqual(self.n.pilot_handover_request_time,at(0))

    def test_console_confirmation_checks_recovery_stamp_and_live_readiness(self):
        status = self.status(recovery_ready=False)
        lock_time = status.recovery_lock_time.to_sec()
        for request_id, time_value in (("unstable", lock_time), ("stale", lock_time - 1)):
            request = {"id": request_id, "command": "ceiling_handover", "recovery_lock_time": time_value}
            self.n._on_command(String(data=json.dumps(request)))
            reply = json.loads(self.n.command_ack_pub.publish.call_args.args[0].data)
            self.assertFalse(reply["accepted"])
            self.assertEqual(self.n.pilot_handover_request_time, at(0))
            self.status()
        request = {"id": "ready", "command": "ceiling_handover", "recovery_lock_time": lock_time}
        self.n._on_command(String(data=json.dumps(request)))
        reply = json.loads(self.n.command_ack_pub.publish.call_args.args[0].data)
        self.assertTrue(reply["accepted"])
        self.assertEqual(reply["pilot_handover_request_time"], self.fx.now)
        intent = self.n.command_pub.publish.call_args.args[0]
        self.assertEqual(intent.pilot_handover_request_time, at(self.fx.now))

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
