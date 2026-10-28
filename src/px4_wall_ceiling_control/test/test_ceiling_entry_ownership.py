"""Ceiling ownership handoff must not masquerade as a prestream dropout."""
import unittest
from unittest import mock

import rospy
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus, OperatorCommand

import test_manual_wall_approach as fixtures
from test_flight_handover import Manager


class CeilingEntryOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.ManualApproachTests()
        self.f.setUp()
        self.n = self.f.node
        n = self.n
        n.phase = Manager.ENTERING
        n.selected_mode = OperatorCommand.MODE_CEILING
        n._requested_mode.return_value = OperatorCommand.MODE_CEILING
        n.takeover_attempt = 4
        n.offboard_request = (4, OperatorCommand.MODE_CEILING, self.f.clock)
        n.flight.mode = 'OFFBOARD'
        n.flight.header.stamp = self.f.now
        n.owner = 1
        n.ceiling = CeilingAttachmentStatus(
            state=CeilingAttachmentStatus.STATE_CEILING_ARMED,
            enabled=True, candidate_valid=True, keep_ownership=True,
            acquire_request=True)
        n.ceiling.header.stamp = self.f.now
        n.ceiling_at = self.f.now

    def tick(self, dt=0):
        self.f.clock += dt
        self.f.tick()

    def assert_cancels(self):
        self.tick()
        self.assertEqual(self.n.phase, Manager.CANCELING_ENTRY)
        self.n.cancel_pub.publish.assert_called_once()

    def test_current_ceiling_owner_finishes_entry_without_cancel(self):
        self.tick()
        self.assertEqual(self.n.phase, Manager.ACTIVE)
        self.n.cancel_pub.publish.assert_not_called()

    def test_owner_wait_timeout_cancels_even_if_manager_saw_owner(self):
        for state in (CeilingAttachmentStatus.STATE_CEILING_ARMED,
                      CeilingAttachmentStatus.STATE_NORMAL_FLIGHT):
            with self.subTest(state=state):
                self.n.phase = Manager.ACTIVE
                self.n.acquire_timeout_s = 6.
                self.n.completed = self.n.detach_seen = self.n.recovery_seen = self.n.fault_seen = False
                self.n.phase_since = self.f.clock - self.n.acquire_timeout_s - .1
                self.n.owner_seen = True
                self.n.ceiling.state = state
                self.n.cancel_pub.publish.reset_mock()
                self.tick()
                self.assertEqual(self.n.phase, Manager.CANCELING_ENTRY)
                self.n.cancel_pub.publish.assert_called_once()
                self.assertIsNone(self.n.offboard_request)

    def test_preentry_stream_still_requires_unowned_output(self):
        self.assertFalse(self.n._stream_ready(self.f.now))
        for phase in (Manager.PRESTREAM, Manager.IDLE, Manager.CANCELING_ENTRY):
            self.n.phase = phase
            self.assertFalse(self.n._entry_stream_ready(self.f.now, self.f.clock))

    def test_actual_logged_owner_before_manager_tick_order(self):
        # 2026-10-25: owner=1 arrived at 04.127, manager tick at 04.166.
        self.n.owner = 0
        self.n.flight.mode = 'POSCTL'
        self.n._request_mode = mock.Mock()
        self.tick()
        self.assertEqual(self.n.phase, Manager.ENTERING)
        self.n.flight.mode = 'OFFBOARD'
        self.n.owner = 1
        self.n.ceiling.state = CeilingAttachmentStatus.STATE_APPROACH
        self.tick(.051)
        self.assertEqual(self.n.phase, Manager.ACTIVE)
        self.n.cancel_pub.publish.assert_not_called()

    def test_owner_callback_before_status_has_only_bounded_grace(self):
        self.n.ceiling.state = CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
        self.n.ceiling.candidate_valid = False
        self.tick()
        self.assertEqual(self.n.phase, Manager.ENTERING)
        self.n.ceiling.state = CeilingAttachmentStatus.STATE_APPROACH
        self.n.ceiling.candidate_valid = True
        self.tick(.02)
        self.assertEqual(self.n.phase, Manager.ACTIVE)
        self.n.cancel_pub.publish.assert_not_called()

    def test_owner_callback_before_offboard_has_only_bounded_grace(self):
        self.n.flight.mode = 'POSCTL'
        self.tick()
        self.assertEqual(self.n.phase, Manager.ENTERING)
        self.n.flight.mode = 'OFFBOARD'
        self.tick(.02)
        self.assertEqual(self.n.phase, Manager.ACTIVE)

    def test_unconfirmed_handoff_times_out(self):
        self.n.ceiling.candidate_valid = False
        self.tick()
        self.assertEqual(self.n.phase, Manager.ENTERING)
        self.tick(.13)
        self.assertEqual(self.n.phase, Manager.CANCELING_ENTRY)

    def test_other_owner_is_immediate_failure(self):
        self.n.owner = 2
        self.assert_cancels()

    def test_ceiling_owner_cannot_authorize_wall_request(self):
        self.n.selected_mode = OperatorCommand.MODE_WALL
        self.n._requested_mode.return_value = OperatorCommand.MODE_WALL
        self.n.offboard_request = (4, OperatorCommand.MODE_WALL, self.f.clock)
        self.assert_cancels()

    def test_absent_canceled_mismatched_or_expired_attempt_is_rejected(self):
        for request in (None, (3, OperatorCommand.MODE_CEILING, 100.),
                        (4, OperatorCommand.MODE_WALL, 100.),
                        (4, OperatorCommand.MODE_CEILING, 93.),
                        (4, OperatorCommand.MODE_CEILING, 101.)):
            with self.subTest(request=request):
                self.setUp()
                self.n.offboard_request = request
                self.assert_cancels()

    def test_stale_or_missing_output_and_owner_have_no_grace(self):
        for name in ('output_at', 'owner_at'):
            for value in (None, rospy.Time.from_sec(8.)):
                with self.subTest(name=name, value=value):
                    self.setUp()
                    setattr(self.n, name, value)
                    self.assert_cancels()

    def test_invalid_source_status_flight_or_candidate_never_authorizes(self):
        mutations = [
            lambda n: setattr(n, 'ceiling', None),
            lambda n: setattr(n, 'ceiling_at', rospy.Time.from_sec(8.)),
            lambda n: setattr(n.ceiling.header, 'stamp', rospy.Time.from_sec(8.)),
            lambda n: setattr(n.ceiling.header, 'stamp', rospy.Time.from_sec(11.)),
            lambda n: setattr(n.ceiling, 'enabled', False),
            lambda n: setattr(n.ceiling, 'candidate_valid', False),
            lambda n: setattr(n.ceiling, 'keep_ownership', False),
            lambda n: setattr(n.ceiling, 'fault_detected', True),
            lambda n: setattr(n.ceiling, 'detach_failed', True),
            lambda n: setattr(n.ceiling, 'approach_timed_out', True),
            lambda n: setattr(n.ceiling, 'abort_land_requested', True),
            lambda n: setattr(n.ceiling, 'state', CeilingAttachmentStatus.STATE_ABORT),
            lambda n: setattr(n.ceiling, 'state', CeilingAttachmentStatus.STATE_DETACH),
            lambda n: setattr(n.flight, 'armed', False),
            lambda n: setattr(n.flight, 'connected', False),
            lambda n: setattr(n.flight, 'mode', 'POSCTL'),
            lambda n: setattr(n, 'flight_at', rospy.Time.from_sec(8.)),
            lambda n: setattr(n.flight.header, 'stamp', rospy.Time.from_sec(8.)),
            lambda n: setattr(n.flight.header, 'stamp', rospy.Time.from_sec(11.)),
        ]
        for index, change in enumerate(mutations):
            with self.subTest(index=index):
                self.setUp()
                change(self.n)
                with mock.patch.object(rospy.Time, 'now', return_value=self.f.now):
                    self.assertFalse(self.n._entry_stream_ready(self.f.now, self.f.clock))

    def test_sustained_output_mode_dropout_still_cancels(self):
        self.n.output_mode = 0
        self.tick()
        self.assertEqual(self.n.phase, Manager.ENTERING)
        self.tick(.13)
        self.assertEqual(self.n.phase, Manager.CANCELING_ENTRY)

    def test_operator_release_still_cancels(self):
        self.n._requested_mode.return_value = None
        self.assert_cancels()

    def test_hard_preflight_fault_still_cancels(self):
        self.n._preflight_ready.return_value = False
        self.assert_cancels()


if __name__ == '__main__':
    unittest.main()
