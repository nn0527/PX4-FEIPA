"""ROS boundary checks using the same fixtures as existing attachment tests."""
import json
import math
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String, Bool
from px4_wall_ceiling_control.msg import OperatorCommand as O, WallPerchStatus as W, AttachmentControlCandidate as C, SensorHealth
from px4_wall_ceiling_control.domain.geometry import quaternion_angle
from px4_wall_ceiling_control.domain.incline_attachment import InclineSettings
from test_attachment_safety import WallDecision, OutputMux
from test_flight_handover import Manager, Terminal
from test_incline_attachment import pose


class InclineBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.
        self.now = mock.patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.clock))
        self.now.start(); self.addCleanup(self.now.stop)
        self.params = {'/incline_attachment/enabled': True, '/incline_attachment/flip_thrust_max': .75,
            '~auto_approach': True, '~horizontal_velocity_enabled': True,
            '~decision_enabled': True, '~require_min_altitude': False, '~kill_guard_enabled': False}
        self.get = mock.patch.object(rospy, 'get_param', side_effect=lambda key, default=None: self.params.get(key, default))
        self.get.start(); self.addCleanup(self.get.stop)

    def node(self):
        with mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), mock.patch.object(rospy, 'Timer'):
            n = WallDecision()
        n._prepare_horizontal_velocity = mock.Mock()
        n._prepare_horizontal_equilibrium = mock.Mock()
        n._horizontal_attitude = lambda now, q: q
        n._fill_horizontal_status = mock.Mock()
        n.horizontal_equilibrium = mock.Mock(ready=True, frozen=True)
        n.horizontal_prestream_attitude = pose(0)
        n.approach_takeover_thrust = .49
        n._entry_height = mock.Mock(return_value=2.)
        n._on_operator(O(mode=O.MODE_INCLINE, enable_control=True))
        self.assertTrue(n.incline_selected)
        n.incline_approach_equilibrium.ready = n.incline_approach_equilibrium.frozen = True
        n.incline_approach_equilibrium.axis = (0., 0., 1.)
        n.incline_approach_yaw = 0.
        n.incline_approach_reference = 2.
        n.incline_approach_base = n.incline_approach_takeover = .49
        return n

    def feedback(self, n, front=.4, top=.5):
        now = rospy.Time.now()
        q = n.incline.q or pose(0)
        n.odom = Odometry(); n.odom.header.stamp = now
        n.odom.pose.pose.orientation = Quaternion(*q)
        n.odom.pose.pose.position.z = 2.
        n.odom_received_at = now
        n.front_range = Range(range=front, min_range=.03, max_range=40.); n.front_range.header.stamp = now
        n.front_received_at = now; n.front_filtered_m = front
        n.top_range = Range(range=top, min_range=.03, max_range=40.); n.top_range.header.stamp = now
        n.top_received_at = now; n.top_filtered_m = top
        n.sensor_health = SensorHealth(front_valid=True, ceiling_valid=True)
        n.sensor_health.header.stamp = now; n.sensor_health_received_at = now
        n.horizontal_velocity_ready = True
        n.horizontal_measured = (0., 0.)
        n.flight_state = State(connected=True, armed=True, mode='OFFBOARD'); n.flight_state.header.stamp = now
        n.flight_state_received_at = now
        n.manager_phase = 'ACTIVE'; n.manager_phase_received_at = now
        output = AttitudeTarget(); output.header.stamp = now
        output.orientation = Quaternion(*q)
        output.thrust = n.incline.thrust if math.isfinite(n.incline.thrust) else .49
        n._valid_wall_output = mock.Mock(return_value=output)
        n.incline_approach_source = (output, now)
        n.operator_received_at = now

    def warm_still_pitch(self, n):
        for _ in range(30):
            self.clock += .02
            self.feedback(n, front=.18, top=.257)
            n.odom.pose.pose.orientation = Quaternion(*pose(0))
            n._valid_wall_output.return_value.orientation = Quaternion(*pose(9))
            n._control_cycle_at(rospy.Time.now())
            if n.incline.phase != 'SLOW_FLIP':
                break
        self.assertEqual(n.incline.phase, 'CONTACT_SETTLE')

    def test_adapter_waits_for_front_gate_and_far_safety_then_emits_candidate(self):
        n = self.node()
        for _ in range(25):
            self.clock += .02
            self.feedback(n, front=2.0)
            n._control_cycle_at(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'WAIT')
        for _ in range(23):
            self.clock += .02
            self.feedback(n, front=1.799)
            n._control_cycle_at(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'APPROACH')
        candidate = n.control_candidate_publisher.publish.call_args[0][0]
        self.assertTrue(candidate.incline_active)
        self.assertEqual(candidate.source, C.SOURCE_WALL)
        self.assertTrue(candidate.acquire_request and candidate.phase_valid and candidate.valid)
        self.clock += .02; self.feedback(n, front=.1)
        n._control_cycle_at(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'SLOW_FLIP')
        self.warm_still_pitch(n)
        self.clock += .02; self.feedback(n, top=.055)
        n.odom.pose.pose.orientation = Quaternion(*pose(0))
        n._control_cycle_at(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'HOLD')

    def test_incline_flip_consumes_live_top_callback_without_wall_progress(self):
        n = self.node(); self.feedback(n, front=.18, top=.257)
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f, pose(0), .49))
        n._incline_cycle(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'SLOW_FLIP')
        self.assertEqual(n.state, W.STATE_FLIP_TO_WALL)
        self.assertEqual(n.flip_elapsed_s, 0.)
        self.assertIsNone(n.top_hold_accept_after)
        n._late_top_window_open = mock.Mock(side_effect=AssertionError('side-wall gate used'))
        self.warm_still_pitch(n)
        for distance, phase in ((.192, 'CONTACT_SETTLE'), (.061, 'CONTACT_SETTLE'),
                                (.059, 'HOLD')):
            self.clock += .1
            self.feedback(n, top=n.top_range.range)
            n.odom.pose.pose.orientation = Quaternion(*pose(0))
            msg = Range(range=distance, min_range=.03, max_range=40.)
            msg.header.stamp = rospy.Time.now()
            before_q, before_thrust = n.incline.q, n.incline.thrust
            n._on_top_range(msg)
            self.assertEqual(n.top_range, msg)
            self.assertTrue(n._top_fresh(rospy.Time.now()))
            n._incline_cycle(rospy.Time.now())
            self.assertEqual(n.incline.phase, phase)
            if phase == 'HOLD':
                self.assertEqual(n.incline.q, pose(0))
                self.assertEqual(n.incline.thrust, before_thrust)
        n._late_top_window_open.assert_not_called()

    def test_side_wall_flip_keeps_late_top_callback_gate(self):
        n = self.node(); self.feedback(n, top=.257)
        n.incline_selected = False
        n.state = W.STATE_FLIP_TO_WALL
        n._late_top_window_open = mock.Mock(return_value=False)
        self.clock += .1
        msg = Range(range=.064, min_range=.03, max_range=40.)
        msg.header.stamp = rospy.Time.now()
        old_range, old_received = n.top_range, n.top_received_at
        n._on_top_range(msg)
        self.assertIs(n.top_range, old_range)
        self.assertEqual(n.top_received_at, old_received)
        n._late_top_window_open.assert_called_once()
        n._late_top_window_open.return_value = True
        n._on_top_range(msg)
        self.assertEqual(n.top_range, msg)

    def test_incline_flip_still_rejects_invalid_and_replayed_top_frames(self):
        n = self.node(); self.feedback(n, top=.257)
        n.state = W.STATE_FLIP_TO_WALL
        self.clock += .1
        first = Range(range=.2, min_range=.03, max_range=40.)
        first.header.stamp = rospy.Time.now()
        n._on_top_range(first)
        for value in (math.nan, math.inf, .02):
            self.clock += .02
            bad = Range(range=value, min_range=.03, max_range=40.)
            bad.header.stamp = rospy.Time.now()
            n._on_top_range(bad)
            self.assertIs(n.top_range, first)
        replay = Range(range=.064, min_range=.03, max_range=40.)
        replay.header.stamp = first.header.stamp
        n._on_top_range(replay)
        self.assertIs(n.top_range, first)

    def test_active_profile_is_latched_until_exit(self):
        n = self.node()
        n.state = W.STATE_WALL_HOLD
        n._on_operator(O(mode=O.MODE_WALL, enable_control=True))
        self.assertTrue(n.incline_selected)
        self.assertFalse(n._incline_requested(rospy.Time.now()))

    def test_mavros_source_lead_matches_wall_feedback_gate(self):
        n = self.node()
        for lead in (0., .029155, .099, .101, .2):
            with self.subTest(lead=lead):
                self.feedback(n)
                n.odom.header.stamp = rospy.Time.from_sec(self.clock + lead)
                n.flight_state.header.stamp = rospy.Time.from_sec(self.clock + lead)
                f = n._incline_feedback_at(rospy.Time.now())
                self.assertEqual(f.pose_valid, n._odom_fresh(rospy.Time.now()))
                self.assertEqual(f.pose_valid, lead < .1)
                self.assertEqual(f.control_valid, lead < .1)

    def test_recorded_29ms_source_lead_does_not_abort_approach(self):
        n = self.node(); self.feedback(n)
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f, pose(0), .49))
        self.clock += .04
        self.feedback(n)
        n.odom.header.stamp = rospy.Time.from_sec(self.clock + .029155)
        n._incline_cycle(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'APPROACH')
        self.assertTrue(n.control_candidate_publisher.publish.call_args[0][0].valid)

    def test_source_tolerance_keeps_receipt_and_source_timeout_checks(self):
        n = self.node()
        for invalid in ('odom_receipt', 'odom_source', 'flight_receipt', 'flight_source', 'zero_source'):
            with self.subTest(invalid=invalid):
                self.feedback(n)
                if invalid == 'odom_receipt':
                    n.odom_received_at = rospy.Time.from_sec(self.clock - n.odom_timeout_s - .01)
                elif invalid == 'odom_source':
                    n.odom.header.stamp = rospy.Time.from_sec(self.clock - n.odom_timeout_s - .01)
                elif invalid == 'flight_receipt':
                    n.flight_state_received_at = rospy.Time.from_sec(self.clock - n.state_timeout_s - .01)
                elif invalid == 'flight_source':
                    n.flight_state.header.stamp = rospy.Time.from_sec(self.clock - n.state_timeout_s - .01)
                else:
                    n.odom.header.stamp = n.flight_state.header.stamp = rospy.Time()
                f = n._incline_feedback_at(rospy.Time.now())
                if invalid.startswith('odom') or invalid == 'zero_source':
                    self.assertFalse(f.pose_valid)
                if invalid.startswith('flight') or invalid == 'zero_source':
                    self.assertFalse(f.control_valid)

    def test_excessive_source_lead_still_requests_landing(self):
        n = self.node(); self.feedback(n)
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f, pose(0), .49))
        self.clock += .02; self.feedback(n)
        n.odom.header.stamp = rospy.Time.from_sec(self.clock + .101)
        n._incline_cycle(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'LAND')
        self.assertTrue(n.abort_land_requested)

    def test_external_mode_change_with_bounded_source_lead_releases_task(self):
        n = self.node(); self.feedback(n)
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f, pose(0), .49))
        self.clock += .02; self.feedback(n)
        n.flight_state.mode = 'POSCTL'
        n.flight_state.header.stamp = rospy.Time.from_sec(self.clock + .029155)
        n._incline_cycle(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'ABORT_DONE')

    def test_incline_height_is_strictly_above_10cm_and_wall_stays_30cm(self):
        n = self.node()
        self.assertEqual(n.min_altitude_m, .30)
        for height, expected in ((None, False), (math.nan, False), (.099, False), (.10, False), (.101, True)):
            n._entry_height = lambda: height
            self.assertEqual(n._incline_entry_height_ready(), expected)
        self.feedback(n)
        f = n._incline_feedback_at(rospy.Time.now())
        status = n._incline_status(rospy.Time.now(), f, True)
        self.assertAlmostEqual(status.min_entry_height_m, .10)

    def test_flip_failure_publishes_abort_land_request(self):
        n = self.node(); self.feedback(n)
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f, pose(0), .49))
        n.incline.phase = 'SLOW_FLIP'
        n.incline.abort(f, 'flip failed')
        n._incline_cycle(rospy.Time.now())
        status = n._incline_status(rospy.Time.now(), f, True)
        self.assertEqual(status.state, W.STATE_ABORT)
        self.assertTrue(status.abort_land_requested)
        self.assertTrue(status.incline_active)

    def test_manager_requests_auto_land_for_incline_abort(self):
        m = Manager.__new__(Manager)
        m.phase = 'ACTIVE'; m.selected_mode = O.MODE_WALL
        m.wall = W(state=W.STATE_ABORT, incline_active=True, abort_land_requested=True,
                   detach_range_maneuver_id='incline-failure', detach_range_episode=1)
        m.wall.header.stamp = rospy.Time.now(); m.wall_at = rospy.Time.now()
        m.status_timeout_s = .5; m.state_timeout_s = 1.5
        m.flight = State(connected=True, armed=True, mode='OFFBOARD')
        m.flight.header.stamp = rospy.Time.now(); m.flight_at = rospy.Time.now()
        m.kill_guard_enabled = False
        m._set_phase = lambda phase, reason: (setattr(m, 'phase', phase), setattr(m, 'phase_reason', reason))
        m._request_mode = mock.Mock()
        m.prestream_pub = mock.Mock(); m.phase_pub = mock.Mock(); m.phase_reason_pub = mock.Mock()
        self.assertTrue(m._tick_wall_abort_land(rospy.Time.now()))
        self.assertEqual(m.phase, 'WALL_LANDING')
        m._request_mode.assert_called_once_with('AUTO.LAND')

    def test_detach_does_not_start_side_wall_range_task(self):
        n = self.node()
        n.state = W.STATE_WALL_HOLD
        n.incline.phase = 'HOLD'
        n.detach_range_enabled = True
        old = n.detach_task_phase
        self.feedback(n)  # Detach now requires a fresh measured yaw at receipt.
        n._on_detach(Bool(data=True))
        self.assertTrue(n.detach_requested)
        self.assertEqual(n.detach_task_phase, old)

    def test_external_mode_change_releases_without_reentering(self):
        n = self.node(); self.feedback(n)
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f, pose(0), .49))
        n.flight_state.mode = 'POSCTL'
        n._incline_cycle(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'ABORT_DONE')
        n.flight_state.mode = 'OFFBOARD'
        n._incline_cycle(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'ABORT_DONE')
        self.assertFalse(n.control_candidate_publisher.publish.call_args[0][0].keep_ownership)

    def test_terminal_start_detach_and_posctl_release(self):
        with mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), mock.patch.object(rospy, 'Timer'):
            n = Terminal()
        n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        n._on_command(String(data=json.dumps({'id': 'incline-1', 'command': 'incline'})))
        self.assertEqual(n.mode, O.MODE_INCLINE)
        self.assertTrue(json.loads(n.command_ack_pub.publish.call_args[0][0].data)['accepted'])
        n._apply_command(String(data='detach')); n._on_timer(None)
        n.wall_detach_pub.publish.assert_called_with(Bool(data=True))
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertEqual(n.mode, O.MODE_STANDBY)

    def test_terminal_rejects_unconfigured_incline(self):
        self.params['/incline_attachment/flip_thrust_max'] = 0.
        with mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), mock.patch.object(rospy, 'Timer'):
            n = Terminal()
        n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        n._apply_command(String(data='incline'))
        self.assertEqual(n.mode, O.MODE_STANDBY)

    def test_plain_terminal_incline_rejects_stale_flight_state(self):
        with mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), mock.patch.object(rospy, 'Timer'):
            n = Terminal()
        n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.clock += 2.
        n._apply_command(String(data='incline'))
        self.assertEqual(n.mode, O.MODE_STANDBY)

    def test_manager_shares_owner_but_keeps_distance_and_profile_gate(self):
        m = Manager.__new__(Manager)
        m.operator = O(mode=O.MODE_INCLINE, enable_control=True)
        m.operator_at = rospy.Time.now(); m.operator_timeout_s = .5
        m.wall = W(incline_active=True, incline_ready=True, state=W.STATE_IDLE,
            front_ready=True, front_range_fresh=True, top_range_fresh=True,
            horizontal_equilibrium_valid=True, entry_height_ready=True)
        m.wall.header.stamp = rospy.Time.now(); m.wall_at = rospy.Time.now(); m.status_timeout_s = .5
        self.assertEqual(m._requested_mode(rospy.Time.now()), O.MODE_WALL)
        self.assertEqual(m._pilot_mode(), 'POSCTL')
        self.assertTrue(m._wall_takeover_ready(rospy.Time.now()))
        m.wall.front_ready = False
        self.assertFalse(m._wall_takeover_ready(rospy.Time.now()))
        m.wall.front_ready = True; m.wall.incline_active = False
        self.assertFalse(m._wall_takeover_ready(rospy.Time.now()))

    def test_output_envelope_has_no_instant_abort_level_bypass(self):
        m = OutputMux.__new__(OutputMux)
        m.max_thrust = .85; m.min_thrust = .1
        m.incline_output_settings = InclineSettings(enabled=True, flip_thrust_max=.75)
        m._wall_publication_seed = lambda now: (pose(60), .65, rospy.Time.from_sec(self.clock-.02), C.SOURCE_WALL)
        target = AttitudeTarget(orientation=Quaternion(*pose(0)), thrust=.49)
        candidate = C(incline_active=True, control_phase=W.STATE_ABORT)
        result = m._limit_wall_publication(rospy.Time.now(), target, candidate)
        q = tuple(getattr(result.orientation, key) for key in 'xyzw')
        self.assertLessEqual(quaternion_angle(pose(60), q), math.radians(.6)+1e-8)
        self.assertLessEqual(abs(result.thrust-.65), .004+1e-8)

    def test_candidate_cannot_masquerade_as_other_wall_profile(self):
        m = OutputMux.__new__(OutputMux); m.max_thrust=.85
        m.operator = O(mode=O.MODE_INCLINE, enable_control=True)
        candidate = C(acquire_request=True, incline_active=False)
        self.assertEqual(m._wall_candidate_problem(candidate, rospy.Time.now()), 'incline_profile_acquisition_mismatch')

    def test_flip_publication_allows_support_slew_but_remains_continuous(self):
        m = OutputMux.__new__(OutputMux)
        m.max_thrust = .85; m.min_thrust = .1
        m.incline_output_settings = InclineSettings(enabled=True, flip_thrust_max=.75)
        m._wall_publication_seed = lambda now: (pose(0), .49, rospy.Time.from_sec(self.clock-.02), C.SOURCE_WALL)
        candidate = C(incline_active=True, control_phase=W.STATE_FLIP_TO_WALL)
        result = m._limit_wall_publication(rospy.Time.now(),
            AttitudeTarget(orientation=Quaternion(*pose(10)), thrust=.57), candidate)
        self.assertAlmostEqual(result.thrust, .494)
        q = tuple(getattr(result.orientation,k) for k in 'xyzw')
        self.assertLessEqual(quaternion_angle(pose(0), q), math.radians(.2)+1e-8)
        # Repeating the call cannot accumulate against the same sent seed.
        again = m._limit_wall_publication(rospy.Time.now(),
            AttitudeTarget(orientation=Quaternion(*pose(10)), thrust=.57), candidate)
        self.assertAlmostEqual(again.thrust, .494)

    def test_flip_support_does_not_relax_hold_or_return_output_limits(self):
        m = OutputMux.__new__(OutputMux)
        m.max_thrust = .85; m.min_thrust = .1
        m.incline_output_settings = InclineSettings(enabled=True, flip_thrust_max=.75)
        m._wall_publication_seed = lambda now: (pose(0), .49, rospy.Time.from_sec(self.clock-.02), C.SOURCE_WALL)
        for phase, expected in ((W.STATE_WALL_HOLD, .4906),
                                (W.STATE_DETACH_ROTATE, .494)):
            result = m._limit_wall_publication(rospy.Time.now(),
                AttitudeTarget(orientation=Quaternion(*pose(0)), thrust=.57),
                C(incline_active=True, control_phase=phase))
            self.assertAlmostEqual(result.thrust, expected)

    def test_flip_status_records_support_reference_correction_and_targets(self):
        n = self.node(); self.feedback(n, front=.18)
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f, pose(0), .49))
        n._incline_cycle(rospy.Time.now())
        self.clock += .02; self.feedback(n, front=.18)
        n.odom.pose.pose.position.z = 1.95
        n.odom.twist.twist.linear.z = -.4
        n._incline_cycle(rospy.Time.now())
        status = n._incline_status(rospy.Time.now(), n.incline_feedback, True)
        self.assertTrue(status.flip_vertical_support_enabled)
        self.assertEqual(status.vertical_reference_m, 2.)
        self.assertAlmostEqual(status.vertical_thrust_correction, .08, places=6)
        self.assertAlmostEqual(status.flip_support_thrust, .57, places=6)
        self.assertAlmostEqual(status.thrust_normalized, .494, places=6)

    def test_front_trigger_boost_reaches_final_output_with_attitude_continuity(self):
        n = self.node(); self.feedback(n, front=.18)
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f, pose(0), .49))
        n._incline_cycle(rospy.Time.now())
        m = OutputMux.__new__(OutputMux)
        m.max_thrust = .85; m.min_thrust = .1
        m.incline_output_settings = InclineSettings(enabled=True, flip_thrust_max=.75)
        sent = [pose(0), .49, rospy.Time.now(), C.SOURCE_WALL]
        m._wall_publication_seed = lambda now: tuple(sent)
        for _ in range(20):
            self.clock += .02; self.feedback(n, front=.18)
            actual = AttitudeTarget(orientation=Quaternion(*sent[0]), thrust=sent[1])
            actual.header.stamp = sent[2]
            n._valid_wall_output = mock.Mock(return_value=actual)
            n._incline_cycle(rospy.Time.now())
            candidate = n.control_candidate_publisher.publish.call_args[0][0]
            self.assertTrue(candidate.valid and candidate.incline_active)
            target = AttitudeTarget(orientation=Quaternion(*n.incline.q), thrust=n.incline.thrust)
            result = m._limit_wall_publication(rospy.Time.now(), target, candidate)
            self.assertIsNotNone(result)
            q = tuple(getattr(result.orientation,k) for k in 'xyzw')
            self.assertLessEqual(quaternion_angle(sent[0],q), math.radians(.2)+1e-8)
            self.assertLessEqual(abs(result.thrust-sent[1]), .004+1e-8)
            sent[:] = [q, result.thrust, rospy.Time.now(), C.SOURCE_WALL]
        self.assertEqual(n.incline.phase, 'SLOW_FLIP')
        self.assertGreater(quaternion_angle(pose(0),sent[0]), 0.)
        # Actual pitch has begun, so tilt feedforward adds a small amount.
        self.assertGreaterEqual(sent[1], .53-1e-6)
        self.assertLess(sent[1], .531)


if __name__ == '__main__':
    unittest.main()
