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
from px4_wall_ceiling_control.msg import OperatorCommand as O, WallPerchStatus as W, AttachmentControlCandidate as C
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
        n._on_operator(O(mode=O.MODE_INCLINE, enable_control=True))
        self.assertTrue(n.incline_selected)
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
        n.flight_state = State(connected=True, armed=True, mode='OFFBOARD'); n.flight_state.header.stamp = now
        n.flight_state_received_at = now
        n.manager_phase = 'ACTIVE'; n.manager_phase_received_at = now
        output = AttitudeTarget(); output.header.stamp = now
        output.orientation = Quaternion(*q)
        output.thrust = n.incline.thrust if math.isfinite(n.incline.thrust) else .49
        n._valid_wall_output = mock.Mock(return_value=output)
        n.operator_received_at = now

    def test_adapter_waits_for_50cm_then_emits_atomic_incline_candidate(self):
        n = self.node()
        for _ in range(25):
            self.clock += .02
            self.feedback(n, front=.6)
            n._control_cycle_at(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'WAIT')
        for _ in range(23):
            self.clock += .02
            self.feedback(n)
            n._control_cycle_at(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'APPROACH')
        candidate = n.control_candidate_publisher.publish.call_args[0][0]
        self.assertTrue(candidate.incline_active)
        self.assertEqual(candidate.source, C.SOURCE_WALL)
        self.assertTrue(candidate.acquire_request and candidate.phase_valid and candidate.valid)
        self.clock += .02; self.feedback(n, front=.1)
        n._control_cycle_at(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'SLOW_FLIP')
        self.clock += .02; self.feedback(n, top=.065)
        n._control_cycle_at(rospy.Time.now())
        self.assertEqual(n.incline.phase, 'CONTACT_SETTLE')

    def test_active_profile_is_latched_until_exit(self):
        n = self.node()
        n.state = W.STATE_WALL_HOLD
        n._on_operator(O(mode=O.MODE_WALL, enable_control=True))
        self.assertTrue(n.incline_selected)
        self.assertFalse(n._incline_requested(rospy.Time.now()))

    def test_detach_does_not_start_side_wall_range_task(self):
        n = self.node()
        n.state = W.STATE_WALL_HOLD
        n.incline.phase = 'HOLD'
        n.detach_range_enabled = True
        old = n.detach_task_phase
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


if __name__ == '__main__':
    unittest.main()
