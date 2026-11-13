"""Ceiling HOLD watchdog and PX4 landing handoff; no live flight commands."""
import copy
from types import SimpleNamespace
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from px4_wall_ceiling_control.msg import (
    CeilingAttachmentStatus as S, OperatorCommand as Op, AttachmentMechanismCommand as M,
)
import test_ceiling_motion as motion_fixtures
from test_flight_handover import Manager


def at(value):
    return rospy.Time.from_sec(value)


class HoldWatchdogTests(unittest.TestCase):
    def setUp(self):
        self.fx = motion_fixtures.CeilingMotionLifecycleTests()
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.fx.hold()
        self.n = self.fx.n
        self.fx.clock = self.n.state_entered_at.to_sec()

    def expire(self):
        return self.fx.feed(dt=5.001, distance=.055)

    def test_strict_five_second_boundary_and_release_request(self):
        status = self.fx.feed(dt=5., distance=.055)
        self.assertEqual(status.state, S.STATE_SURFACE_HOLD)
        seq = self.n.mechanism_sequence
        status = self.fx.feed(dt=.001, distance=.055)
        self.assertEqual(status.state, S.STATE_ABORT)
        self.assertTrue(status.hold_timed_out)
        self.assertTrue(status.abort_land_requested)
        self.assertTrue(status.abort_maneuver_id)
        self.assertTrue(status.keep_ownership)
        self.assertTrue(status.candidate_valid)
        self.assertTrue(status.mechanism_release_requested)
        self.assertAlmostEqual(-status.thrust_body_z_normalized, .55)
        self.assertEqual(self.n.mechanism_sequence, seq+1)
        self.assertEqual(self.n.mechanism_candidate_publisher.publish.call_args.args[0].action, M.ACTION_RELEASE)
        self.assertEqual(self.n.motion_controller.horizontal.integral, (0., 0.))

    def test_normal_detach_accepted_on_deadline_tick_has_priority(self):
        self.n.detach_requested = True
        status = self.expire()
        self.assertEqual(status.state, S.STATE_DETACH)
        self.assertFalse(status.abort_land_requested)
        status = self.fx.feed(dt=5.1, distance=.6)
        self.assertEqual(status.state, S.STATE_DETACH)
        self.assertFalse(status.hold_timed_out)

    def test_missing_range_cannot_pause_hold_watchdog(self):
        status = self.fx.feed(dt=5.001, distance=255.)
        self.assertFalse(status.range_fresh)
        self.assertEqual(status.state, S.STATE_ABORT)
        self.assertTrue(status.abort_land_requested)

    def test_abort_latches_despite_cancel_detach_or_motion_fault(self):
        status = self.expire(); identity = status.abort_maneuver_id
        self.n.detach_requested = True
        self.n.motion_fatal = 'flow fault'
        self.n._on_operator(Op(mode=Op.MODE_STANDBY, enable_control=False))
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertEqual(self.n.abort_maneuver_id, identity)
        self.assertTrue(self.n.rearm_required)

    def test_only_actual_mode_feedback_releases_offboard_output(self):
        self.expire()
        self.assertTrue(self.n._build_status(rospy.Time.now(), .02).keep_ownership)
        self.fx.mode = 'AUTO.LAND'
        status = self.fx.feed(distance=.6)
        self.assertEqual(status.state, S.STATE_ABORT)
        self.assertFalse(status.keep_ownership)
        self.assertFalse(status.candidate_valid)
        self.assertTrue(status.abort_land_requested)

    def test_fresh_disarm_resets_abort_before_neutral_intent(self):
        self.expire()
        self.n._on_operator(Op(mode=Op.MODE_CEILING, enable_control=True))
        abort_time = self.n.state_entered_at
        fault = self.n.last_reason
        st = State(connected=True, armed=False, mode='AUTO.LAND')
        st.header.stamp = abort_time
        self.fx.clock += .01
        self.n._on_state(st)
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state, S.STATE_ABORT)

        self.fx.clock += .01
        st = State(connected=True, armed=False, mode='AUTO.LAND')
        st.header.stamp = rospy.Time.now()
        self.n._on_state(st)
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state, S.STATE_NORMAL_FLIGHT)
        self.assertIn(fault, self.n.last_reason)
        self.assertFalse(self.n.hold_timed_out)
        self.n._update_state_machine(rospy.Time.now())
        self.assertTrue(self.n.rearm_required)
        self.assertFalse(self.n._build_status(rospy.Time.now(), .02).activation_ready)

        self.n._on_operator(Op(mode=Op.MODE_STANDBY, enable_control=False))
        self.n._update_state_machine(rospy.Time.now())
        self.assertFalse(self.n.rearm_required)

    def test_ordinary_abort_keeps_original_support_without_release_or_landing(self):
        # A former FAULT must retain the captured support (0.53), bounded XY
        # correction and POSCTL acknowledgment, rather than the HOLD landing path.
        self.fx.clock = self.n.previous_update_at.to_sec()
        self.n._enter_state(S.STATE_ABORT, rospy.Time.now(), 'ordinary feedback fault')
        sequence = self.n.mechanism_sequence
        for _ in range(5):
            status = self.fx.feed(x=.20, distance=.6)
        self.assertEqual(status.state_name, 'ABORT')
        self.assertFalse(status.abort_land_requested)
        self.assertFalse(status.mechanism_release_requested)
        self.assertEqual(status.abort_maneuver_id, '')
        self.assertEqual(self.n.mechanism_sequence, sequence)
        self.assertEqual(self.n.mechanism_candidate_publisher.publish.call_args.args[0].action, M.ACTION_NONE)
        self.assertTrue(status.keep_ownership)
        self.assertGreater(-status.thrust_body_z_normalized, .52)
        self.assertNotEqual(self.n.motion_controller.horizontal.acceleration, (0., 0.), (self.n.motion_ready, self.n.motion_fatal, self.n.motion_controller.xy, self.n.motion_controller.horizontal.target, self.n.motion_reason))
        self.assertEqual(self.n.motion_controller.horizontal.integral, (0., 0.))
        candidate = self.n.control_candidate_publisher.publish.call_args.args[0]
        self.assertFalse(candidate.ceiling_abort_land_requested)
        self.fx.mode = 'POSCTL'
        self.assertEqual(self.fx.feed(distance=.6).state, S.STATE_NORMAL_FLIGHT)

    def test_landing_subtype_travels_with_control_candidate(self):
        self.expire()
        candidate = self.n.control_candidate_publisher.publish.call_args.args[0]
        self.assertEqual(candidate.control_phase, S.STATE_ABORT)
        self.assertTrue(candidate.ceiling_abort_land_requested)


class CeilingLandingManagerTests(unittest.TestCase):
    def setUp(self):
        self.now = at(10.)
        patch = mock.patch.object(rospy.Time, 'now', return_value=self.now)
        patch.start(); self.addCleanup(patch.stop)
        patch = mock.patch.dict(Manager._ceiling_abort_land_live.__globals__, kill_clear=lambda *a: True)
        patch.start(); self.addCleanup(patch.stop)
        patch = mock.patch.object(rospy, 'wait_for_service')
        patch.start(); self.addCleanup(patch.stop)
        n = self.n = Manager.__new__(Manager)
        n.phase = n.ACTIVE; n.phase_reason = ''
        n.selected_mode = Op.MODE_CEILING; n.takeover_attempt = 3
        n.status_timeout_s = n.state_timeout_s = 1.
        n.ceiling = S(state=S.STATE_ABORT, abort_land_requested=True, hold_timed_out=True,
                      abort_maneuver_id='ceiling-abort-test', reason='HOLD timeout')
        n.ceiling.header.stamp = n.ceiling_at = self.now
        n.flight = State(connected=True, armed=True, mode='OFFBOARD')
        n.flight.header.stamp = n.flight_at = self.now
        n.rc_takeover_requested = False
        n.prestream_pub = mock.Mock(); n.phase_pub = mock.Mock(); n.phase_reason_pub = mock.Mock()
        n._request_mode = mock.Mock()
        n.request_in_flight = False
        n.set_mode = mock.Mock(return_value=SimpleNamespace(mode_sent=True))
        n._reset = mock.Mock(side_effect=lambda **kw: n._set_phase(n.LOCKED))

    def test_abort_immediately_requests_land_without_baro_flow_or_recovery_gate(self):
        n = self.n
        n._handover_ready = mock.Mock(side_effect=AssertionError('must not wait for POSCTL'))
        n._ceiling_baro_handover_ready = mock.Mock(side_effect=AssertionError('must not wait for baro'))
        self.assertTrue(n._tick_ceiling_abort_land(self.now))
        self.assertEqual(n.phase, 'CEILING_LANDING')
        n._request_mode.assert_called_once_with('AUTO.LAND')
        n._send_mode_request('AUTO.LAND', attempt=3)
        self.assertEqual(n.set_mode.call_args.args[0].custom_mode, 'AUTO.LAND')
        self.assertFalse(n.ceiling_land_confirmed)

    def test_landing_has_priority_over_normal_completion(self):
        n = self.n
        n._publish_range_handover_readiness = mock.Mock()
        n._tick_rc_takeover_manager = mock.Mock(return_value=False)
        n._observe_completion = mock.Mock(side_effect=AssertionError('must not run normal handback'))
        n._on_timer(None)
        n._request_mode.assert_called_once_with('AUTO.LAND')

    def test_service_success_needs_new_actual_auto_land_feedback(self):
        n = self.n; n._tick_ceiling_abort_land(self.now)
        n._send_mode_request('AUTO.LAND', attempt=3)
        n.flight.mode = 'AUTO.LAND'
        n._tick_ceiling_abort_land(self.now)
        self.assertFalse(n.ceiling_land_confirmed)
        n.flight.header.stamp = n.flight_at = at(10.1)
        n._tick_ceiling_abort_land(at(10.1))
        self.assertTrue(n.ceiling_land_confirmed)

    def test_rejection_can_retry(self):
        n = self.n; n._tick_ceiling_abort_land(self.now)
        n.set_mode.return_value.mode_sent = False
        n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.return_value.mode_sent = True
        n._send_mode_request('AUTO.LAND', attempt=3)
        self.assertEqual(n.set_mode.call_count, 2)
        self.assertFalse(n.ceiling_land_confirmed)

    def test_worker_rechecks_attempt_and_identity_after_service_wait(self):
        n = self.n; n._tick_ceiling_abort_land(self.now)
        n._send_mode_request('AUTO.LAND', attempt=2)
        n.set_mode.assert_not_called()
        with mock.patch.object(rospy, 'wait_for_service', side_effect=lambda *a, **kw: setattr(n.ceiling, 'abort_maneuver_id', 'another-flight')):
            n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.assert_not_called()

    def test_manual_mode_change_stops_requests_and_retires_abort(self):
        n = self.n; n._tick_ceiling_abort_land(self.now)
        n._request_mode.reset_mock(); n.flight.mode = 'POSCTL'
        n._tick_ceiling_abort_land(self.now)
        n._request_mode.assert_not_called(); n._reset.assert_called_once()
        n.phase = n.ACTIVE
        self.assertFalse(n._tick_ceiling_abort_land(self.now))

    def test_stale_or_missing_identity_cannot_start_landing(self):
        n = self.n
        for field, value in [('stamp', at(8.)), ('identity', '')]:
            n.ceiling.header.stamp = self.now; n.ceiling.abort_maneuver_id = 'ceiling-abort-test'
            if field == 'stamp': n.ceiling.header.stamp = value
            else: n.ceiling.abort_maneuver_id = value
            self.assertFalse(n._tick_ceiling_abort_land(self.now))
        n._request_mode.assert_not_called()

    def test_disarm_rc_override_or_manual_mode_cannot_send_land(self):
        n = self.n; n._tick_ceiling_abort_land(self.now)
        for field, value in [('armed', False), ('mode', 'POSCTL')]:
            before = getattr(n.flight, field); setattr(n.flight, field, value)
            n._send_mode_request('AUTO.LAND', attempt=3)
            setattr(n.flight, field, before)
        n.rc_takeover_requested = True
        n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.assert_not_called()


    def test_ordinary_abort_cannot_start_land_even_with_identity(self):
        n = self.n
        n.ceiling.abort_land_requested = False
        n.ceiling.hold_timed_out = False
        self.assertFalse(n._tick_ceiling_abort_land(self.now))
        n._request_mode.assert_not_called()
        n.ceiling.motion_control_enabled = True
        self.assertTrue(n._ceiling_approach_timeout_requested(self.now))
        n.ceiling.abort_land_requested = True
        self.assertFalse(n._ceiling_approach_timeout_requested(self.now))


class AbortOutputReleaseTests(unittest.TestCase):
    def test_abort_release_retires_only_current_attempt(self):
        fixture = motion_fixtures.CeilingFinalOutputTests(); fixture.setUp(); self.addCleanup(fixture.doCleanups)
        n, candidate = fixture.n, fixture.c
        n._on_ceiling_candidate(candidate)
        release = copy.deepcopy(candidate)
        release.header.stamp = at(10.01)
        release.control_sequence += 1; release.control_phase = S.STATE_ABORT
        release.ceiling_abort_land_requested = True
        release.keep_ownership = False; release.valid = False
        with mock.patch.object(rospy.Time, 'now', return_value=at(10.01)):
            n._on_ceiling_candidate(release)
        self.assertIs(n.ceiling_candidate, release)
        self.assertIn(candidate.maneuver_id, n.ceiling_retired_maneuvers)


    def test_ordinary_abort_cannot_release_current_output_attempt(self):
        fixture = motion_fixtures.CeilingFinalOutputTests(); fixture.setUp(); self.addCleanup(fixture.doCleanups)
        n, candidate = fixture.n, fixture.c
        n._on_ceiling_candidate(candidate)
        release = copy.deepcopy(candidate)
        release.header.stamp = at(10.01)
        release.control_sequence += 1; release.control_phase = S.STATE_ABORT
        release.keep_ownership = False; release.valid = False
        release.ceiling_abort_land_requested = False
        with mock.patch.object(rospy.Time, 'now', return_value=at(10.01)):
            n._on_ceiling_candidate(release)
        self.assertIs(n.ceiling_candidate, candidate)
        self.assertNotIn(candidate.maneuver_id, n.ceiling_retired_maneuvers)

    def test_final_envelope_uses_atomic_abort_subtype(self):
        for landing in (False, True):
            fixture = motion_fixtures.CeilingFinalOutputTests(); fixture.setUp(); self.addCleanup(fixture.doCleanups)
            n, candidate = fixture.n, fixture.c
            candidate.control_phase = S.STATE_ABORT
            candidate.ceiling_abort_land_requested = landing
            n._on_ceiling_candidate(candidate)
            # Conflicting status cannot change the envelope of this candidate.
            n.ceiling_status = S(state=S.STATE_ABORT, abort_land_requested=not landing)
            with mock.patch.dict(n._publish_candidate.__globals__,
                    limit_ceiling_attitude=mock.Mock(return_value=(0., 0., 0., 1.))) as namespace:
                n._publish_candidate(fixture.now, candidate)
                self.assertEqual(namespace['limit_ceiling_attitude'].call_args.args[-1], landing)


if __name__ == '__main__':
    unittest.main()
