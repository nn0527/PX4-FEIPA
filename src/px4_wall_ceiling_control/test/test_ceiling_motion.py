"""POSCTL ceiling lifecycle and feedback-fault boundaries; no flight commands."""
import math
import copy
import unittest
from dataclasses import replace
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, OpticalFlowRad, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Range
from std_msgs.msg import String, UInt8
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, OperatorCommand, AttachmentControlCandidate as C
from px4_wall_ceiling_control.domain.ceiling_motion import CeilingMotionController, CeilingMotionSettings, limit_ceiling_attitude
from px4_wall_ceiling_control.domain.geometry import rotate_vector, euler_from_quaternion
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from test_attachment_safety import CeilingDecision, OutputMux
from test_flight_handover import Manager


class CeilingEnvelopeTests(unittest.TestCase):
    def setUp(self):
        self.s = CeilingMotionSettings()

    def test_bad_limits_and_hysteresis_are_rejected(self):
        for change in ({'actual_tilt_resume_deg': 16.}, {'horizontal_drift_pause_m': .2},
                       {'attitude_max_tilt_deg': float('nan')}, {'motion_flow_future_tolerance_s': .2}):
            with self.assertRaises(ValueError):
                replace(self.s, **change)

    def test_position_deadband_keeps_velocity_braking(self):
        c = CeilingMotionController(self.s)
        c.capture((0., 0.), 0., (0., 0., 1.), .53)
        q = c.attitude((.05, 0.), (.12, 0.), .05)
        self.assertEqual(c.horizontal.target, (0., 0.))
        self.assertLess(rotate_vector(q, (0., 0., 1.))[0], 0.)

    def test_large_drift_has_bounded_corrective_velocity_and_acceleration(self):
        c = CeilingMotionController(self.s)
        c.capture((0., 0.), .7, (0., 0., 1.), .53)
        for _ in range(100):
            q = c.attitude((2., -2.), (1., -1.), .05)
        self.assertLessEqual(math.hypot(*c.horizontal.target), .20+1e-9)
        self.assertLessEqual(math.hypot(*c.horizontal.acceleration), .40+1e-9)
        self.assertAlmostEqual(euler_from_quaternion(q)[2], .7)

    def test_final_envelope_limits_rc_trim_and_separate_axis_yaw_rates(self):
        desired = attitude_from_axis_yaw((.5, 0., math.sqrt(.75)), math.radians(60))
        previous = (0., 0., 0., 1.)
        q = limit_ceiling_attitude(desired, previous, .02, 0., self.s)
        self.assertLessEqual(math.degrees(math.acos(rotate_vector(q, (0., 0., 1.))[2])), .30+1e-6)
        self.assertAlmostEqual(math.degrees(euler_from_quaternion(q)[2]), .30, places=5)
        for _ in range(100):
            q = limit_ceiling_attitude(desired, q, .05, 0., self.s, contact=True)
        self.assertLessEqual(math.degrees(math.acos(rotate_vector(q, (0., 0., 1.))[2])), 5.+1e-6)
        self.assertLessEqual(abs(math.degrees(euler_from_quaternion(q)[2])), 10.+1e-6)

    def test_vertical_target_does_not_step_on_ascent_or_recovery(self):
        c = CeilingMotionController(self.s)
        self.assertAlmostEqual(c.vertical_target(.15, .02), .003)
        self.assertAlmostEqual(c.vertical_target(-.15, .02), 0.)


class CeilingMotionLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.
        patch = mock.patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.clock))
        patch.start(); self.addCleanup(patch.stop)
        params = {'~motion_control_enabled': True,
            '~detach_baro_height_enabled': True, '~require_mechanism_ack': False,
            '~approach_settle_time_s': 0.,
            '~contact_distance_m': .056, '~target_compression_m': .006,
            '~surface_hold_thrust': .55, '~detach_drop_m': .50,
            '~detach_timeout_s': 20.}
        params.update(getattr(self, "extra_params", {}))
        with mock.patch.object(rospy, 'get_param', side_effect=lambda k, d=None: params.get(k, d)), \
             mock.patch.object(rospy, 'Publisher'), mock.patch.object(rospy, 'Subscriber'), mock.patch.object(rospy, 'Timer'):
            self.n = CeilingDecision()
        self.n.status_publisher=mock.Mock()
        self.n.control_candidate_publisher=mock.Mock()
        self.n.mechanism_candidate_publisher=mock.Mock()
        self.reference_pressure = 101325.
        self.mode = 'POSCTL'
        self.n._on_operator(OperatorCommand(mode=OperatorCommand.MODE_CEILING, enable_control=True))

    def feed(self, dt=.10, distance=.5, x=0., y=0., vx=0., vz=0., drop=0., thrust=.53,
             flow=True, state=True, tilt=0., rate=0., estimator=True,
             pressure_drop=None, baro=True, vertical_position=True, flow_distance=None, frame="map", owner=1,
             range_min=.03):
        self.clock += dt
        now = rospy.Time.now()
        odom = Odometry(); odom.header.stamp = now; odom.header.frame_id = frame
        odom.pose.pose.orientation = Quaternion(*attitude_from_axis_yaw((math.sin(tilt), 0., math.cos(tilt)), 0.))
        odom.pose.pose.position.x = x; odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 2.-drop
        odom.twist.twist.linear.x = vx; odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.y = rate
        self.n._on_odom(odom)
        if estimator:
            e = EstimatorStatus(attitude_status_flag=True, velocity_horiz_status_flag=True,
                velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True, pos_vert_abs_status_flag=vertical_position)
            e.header.stamp = now; self.n._on_ceiling_estimator(e)
        if flow:
            if flow_distance is None:
                # The ceiling node locks a top-distance baseline with the first
                # downward-flow sample and afterwards only uses that baseline.
                # Derive the flow distance from the requested top distance so the
                # locked estimate reproduces "distance" exactly.
                reference = getattr(self.n, 'approach_flow_reference', None)
                top0 = reference[0] if reference is not None else distance
                flow_distance = 2.0 + (top0 - distance)
            if (getattr(self.n, 'approach_flow_last_height', None) is not None
                    and abs(flow_distance*math.cos(tilt) - self.n.approach_flow_last_height)
                    > .25 + 2.*dt):
                # A test may jump the simulated distance; start a fresh flow
                # history instead of tripping the real floor-step guard.
                self.n.approach_flow_last_height = None
            f = OpticalFlowRad(quality=100, integration_time_us=100000, distance=flow_distance)
            f.header.stamp = now; self.n._on_ceiling_flow(f)
        if self.mode == 'POSCTL':
            target = AttitudeTarget(thrust=thrust, orientation=Quaternion(w=1.))
            target.header.stamp = now; self.n._on_ceiling_fcu_target(target)
        if state:
            st = State(connected=True, armed=True, mode=self.mode); st.header.stamp=now
            self.n._on_state(st)
        self.n._on_range(Range(range=distance, min_range=range_min, max_range=40.))
        self.n._on_operator(OperatorCommand(mode=OperatorCommand.MODE_CEILING, enable_control=True))
        pressure = self.reference_pressure*(1.+(drop if pressure_drop is None else pressure_drop)/44330.)**(1./.190294957)
        if baro:
            message = FluidPressure(fluid_pressure=pressure); message.header.stamp=now
            self.n._on_pressure(message)
        if self.mode == "OFFBOARD" and owner is not None:
            self.n._on_ceiling_owner(UInt8(data=owner))
        self.n._on_timer(None)
        return self.n.status_publisher.publish.call_args.args[0]

    def takeover(self):
        for _ in range(14):
            status = self.feed()
        self.assertTrue(status.motion_ready)
        self.assertTrue(status.baseline_ready)
        self.mode = 'OFFBOARD'
        self.feed()
        self.feed()
        self.assertEqual(self.n.state, S.STATE_APPROACH)

    def hold(self):
        self.takeover()
        for _ in range(20):
            self.feed(distance=.055)
        self.assertEqual(self.n.state, S.STATE_SURFACE_HOLD)

    def test_capture_inherits_posctl_thrust_and_reference_and_no_echo_learning(self):
        self.takeover()
        self.assertAlmostEqual(self.n.motion_controller.vertical_support, .53)
        self.assertEqual(self.n.motion_controller.xy, (0., 0.))
        t = AttitudeTarget(thrust=.8, orientation=Quaternion(w=1.)); t.header.stamp=rospy.Time.now()
        self.n._on_ceiling_fcu_target(t)
        self.assertAlmostEqual(self.n.motion_controller.vertical_support, .53)
        candidate = self.n.control_candidate_publisher.publish.call_args.args[0]
        self.assertEqual(candidate.kind, C.KIND_ATTITUDE_THRUST)
        self.assertTrue(candidate.phase_valid)

    def test_entering_freezes_source_through_delayed_offboard_heartbeat(self):
        for _ in range(14):
            self.feed()
        self.n._on_ceiling_manager_phase(String(data='ENTERING'))
        self.assertTrue(self.n.motion_equilibrium.frozen)
        for _ in range(8):
            self.feed(thrust=.8)
        self.mode='OFFBOARD'
        self.feed()
        self.assertEqual(self.n.motion_fatal, '')
        self.assertAlmostEqual(self.n.motion_controller.vertical_support, .53)

    def test_expired_frozen_source_cannot_capture_offboard(self):
        for _ in range(14):
            self.feed()
        self.n._on_ceiling_manager_phase(String(data='ENTERING'))
        for _ in range(22):
            self.feed()
        self.mode='OFFBOARD'
        self.feed()
        self.assertIsNone(self.n.motion_controller.xy)
        self.assertTrue(self.n.motion_fatal)

    def test_wall_entering_does_not_freeze_ceiling_or_latch_fault(self):
        self.n._on_operator(OperatorCommand(mode=OperatorCommand.MODE_WALL, enable_control=True))
        self.n._on_ceiling_manager_phase(String(data='ENTERING'))
        self.assertFalse(self.n.motion_equilibrium.frozen)
        self.assertEqual(self.n.motion_fatal, '')

    def test_position_reset_fault_does_not_chase_original_reference(self):
        self.takeover()
        status=self.feed(x=2.)
        self.assertTrue(self.n.motion_fatal)
        self.assertEqual(self.n.motion_controller.horizontal.acceleration, (0., 0.))
        self.assertEqual(self.n.motion_controller.horizontal.integral, (0., 0.))

    def test_full_cycle_requires_real_recovery_and_confirmed_posctl(self):
        self.hold()
        self.n.detach_requested = True
        self.feed(distance=.055)
        self.assertEqual(self.n.state, S.STATE_DETACH)
        status = self.feed(distance=.6, drop=.51, vz=-.4)
        self.assertEqual(self.n.state, S.STATE_RECOVERY_HOVER)
        self.assertFalse(status.recovery_ready)
        for _ in range(8):
            status = self.feed(distance=.6, drop=.51, vz=0.)
        self.assertTrue(status.recovery_ready)
        self.assertTrue(status.keep_ownership)
        self.assertEqual(self.n.state, S.STATE_RECOVERY_HOVER)
        self.mode = 'POSCTL'
        status = self.feed(distance=.6, drop=.51)
        self.assertEqual(self.n.state, S.STATE_NORMAL_FLIGHT)
        self.assertFalse(status.keep_ownership)

    def test_single_near_frame_enters_contact_control_then_hold(self):
        self.takeover()
        status=self.feed(distance=.067)
        self.assertEqual(self.n.state,S.STATE_ATTACH_CONTROL)
        for _ in range(60):
            status=self.feed(distance=.055)
            if self.n.state == S.STATE_SURFACE_HOLD:
                break  # Test contact entry without consuming the separate HOLD timeout.
        self.assertEqual(self.n.state,S.STATE_SURFACE_HOLD)
        self.assertAlmostEqual(-status.thrust_body_z_normalized,.55)

    def test_near_frame_is_not_hidden_by_another_range_in_the_same_timer_period(self):
        self.takeover()
        self.n._on_range(Range(range=.067,min_range=.03,max_range=40.))
        status=self.feed(dt=.01,distance=.09)
        self.assertEqual(self.n.state,S.STATE_ATTACH_CONTROL)

    def test_recovery_accepts_five_cm_band_after_reaching_half_metre(self):
        self.hold();self.n.detach_requested=True;self.feed()
        self.feed(distance=.6,drop=.5)
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
        for drop in (.45,.55):
            for _ in range(8):
                status=self.feed(distance=.6,drop=drop)
            self.assertTrue(status.recovery_ready)
        for drop in (.449,.551):
            status=self.feed(distance=.6,drop=drop)
            self.assertFalse(status.recovery_ready)

    def test_stability_below_half_metre_cannot_skip_initial_descent_threshold(self):
        self.hold();self.n.detach_requested=True;self.feed()
        for _ in range(8):
            status=self.feed(distance=.6,drop=.49)
        self.assertEqual(self.n.state,S.STATE_DETACH)
        self.assertFalse(status.recovery_ready)

    def test_no_timer_only_recovery_confirmation(self):
        self.hold(); self.n.detach_requested=True; self.feed()
        self.feed(drop=.51, distance=.6)
        self.n._ceiling_recovery_ready(rospy.Time.now())
        self.clock += .3
        self.assertFalse(self.n._ceiling_recovery_ready(rospy.Time.now()))

    def test_contact_disables_position_pi_and_keeps_hold_thrust(self):
        self.hold()
        self.n.motion_controller.horizontal.integral = (.1, .1)
        status = self.feed(distance=.055, x=.25, vx=.1)
        self.assertAlmostEqual(-status.thrust_body_z_normalized, .55)
        self.assertEqual(self.n.motion_controller.horizontal.integral, (0., 0.))

    def test_excessive_drift_pauses_ascent_then_requires_hysteresis(self):
        self.takeover()
        self.feed(x=.60)  # Keep individual steps below the odometry jump guard.
        for _ in range(5):
            status = self.feed(x=.90)
        self.assertFalse(status.motion_paused)
        for _ in range(5):
            status = self.feed(x=1.10)
        self.assertTrue(status.motion_paused)
        for _ in range(6):
            status = self.feed(x=.61)
        self.assertTrue(status.motion_paused)
        for _ in range(4):
            status = self.feed(x=.60)
        self.assertTrue(status.motion_paused)  # Recovery must persist for 0.5 s.
        for _ in range(4):
            status = self.feed(x=.60)
        self.assertFalse(status.motion_paused)
        self.assertEqual(self.n.state, S.STATE_APPROACH)

    def test_contact_failure_establishes_abort_descent_pressure_reference(self):
        self.takeover()
        self.n._enter_state(S.STATE_ATTACH_CONTROL, rospy.Time.now(), 'contact')
        self.assertIsNone(self.n.hold_pressure_pa)
        self.n.detach_requested=True
        self.feed(distance=.08)
        self.assertEqual(self.n.state, S.STATE_DETACH)
        self.assertIsNotNone(self.n.hold_pressure_pa)
        self.assertIsNotNone(self.n._height_feedback(rospy.Time.now()))

    def test_flow_loss_pauses_then_latches_fault_without_resuming_old_action(self):
        self.takeover()
        for _ in range(32):
            status = self.feed(flow=False)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertTrue(status.keep_ownership)
        for _ in range(10):
            self.feed()
        self.assertEqual(self.n.state, S.STATE_ABORT)

    def test_position_reset_latches_fault_instead_of_chasing_old_reference(self):
        self.takeover()
        self.feed(x=2.)
        self.assertEqual(self.n.state, S.STATE_ABORT)

    def test_duplicate_flow_cannot_renew_feedback(self):
        self.takeover()
        f = self.n.motion_flow
        self.n._on_ceiling_flow(f)
        self.n._prepare_ceiling_motion(rospy.Time.now())
        self.assertFalse(self.n.motion_ready)

    def test_detach_remains_moving_toward_exact_threshold(self):
        self.hold(); self.n.detach_requested=True; self.feed()
        for _ in range(15):
            status = self.feed(distance=.6, drop=.499)
        self.assertEqual(self.n.state, S.STATE_DETACH)
        self.assertLess(status.target_vertical_speed_mps, -.04)

    def test_contact_angular_rate_guard_requests_controlled_detach(self):
        self.hold()
        for _ in range(4):
            self.feed(distance=.055, rate=math.radians(35.))
        self.assertEqual(self.n.state, S.STATE_DETACH)

    def test_replayed_pressure_cannot_qualify_recovery_height(self):
        self.hold(); self.n.detach_requested=True; self.feed()
        pressure = FluidPressure(fluid_pressure=self.reference_pressure)
        pressure.header.stamp=rospy.Time.from_sec(self.clock-2.)
        self.n._on_pressure(pressure)
        self.assertIsNone(self.n._height_feedback(rospy.Time.now()))

    def test_fast_descent_does_not_pass_recovery_or_timer_only_wait(self):
        self.hold(); self.n.detach_requested=True; self.feed()
        for _ in range(12):
            status = self.feed(drop=.51, distance=.6, vz=-.35)
        self.assertFalse(status.recovery_ready)
        self.assertGreater(-status.thrust_body_z_normalized, self.n.motion_controller.vertical_support)


class CeilingFinalOutputTests(unittest.TestCase):
    def setUp(self):
        self.now=rospy.Time.from_sec(10.)
        clock=mock.patch.object(rospy.Time, 'now', return_value=self.now)
        clock.start(); self.addCleanup(clock.stop)
        n=self.n=OutputMux.__new__(OutputMux)
        n.ceiling_motion_enabled=True; n.ceiling_motion_settings=CeilingMotionSettings()
        n.ceiling_candidate_timeout_s=.2; n.ceiling_retired_maneuvers=set()
        n.ceiling_maneuver_id=None; n.ceiling_last_sequence=0; n.ceiling_last_source=None
        n.ceiling_candidate_snapshot=None
        n._assisted_attitude=lambda now,q,*args:q
        n._attitude_target=lambda now,q,t:AttitudeTarget(orientation=q,thrust=t)
        n._limit_final_recovery_target=lambda now,t,*args:t
        n._limit_final_approach_target=lambda now,t,*args:t
        n.attitude_publisher=mock.Mock(); n._remember_publication=mock.Mock()
        n.last_published_control=((0.,0.,0.,1.),.53,rospy.Time.from_sec(9.98),C.SOURCE_CEILING)
        self.c=C(source=C.SOURCE_CEILING,kind=C.KIND_ATTITUDE_THRUST,phase_valid=True,
            control_phase=S.STATE_APPROACH,maneuver_id='ceiling-a',control_sequence=1,
            keep_ownership=True,valid=True,thrust_normalized=.53,ceiling_yaw_reference_rad=0.)
        self.c.header.stamp=self.now
        self.c.attitude_setpoint=Quaternion(*attitude_from_axis_yaw((.4,0.,math.sqrt(.84)),math.radians(40)))

    def test_final_publisher_applies_slew_after_all_trim_and_keeps_thrust(self):
        self.n._on_ceiling_candidate(self.c)
        result=self.n._publish_candidate(self.now,self.c)
        self.assertEqual(result,OutputMux.OUTPUT_ATTITUDE_THRUST)
        target=self.n.attitude_publisher.publish.call_args.args[0]
        q=tuple(getattr(target.orientation,k) for k in 'xyzw')
        self.assertLessEqual(math.degrees(math.acos(rotate_vector(q,(0.,0.,1.))[2])),.30+1e-6)
        self.assertAlmostEqual(target.thrust,.53)

    def test_expired_candidate_is_rejected_again_at_publication(self):
        self.n._on_ceiling_candidate(self.c)
        self.n.ceiling_candidate_snapshot=(self.c,rospy.Time.from_sec(9.))
        self.assertEqual(self.n._publish_candidate(self.now,self.c),OutputMux.OUTPUT_NONE)
        self.n.attitude_publisher.publish.assert_not_called()

    def test_replay_and_old_release_cannot_replace_current_attempt(self):
        self.n._on_ceiling_candidate(self.c)
        old=copy.deepcopy(self.c); old.maneuver_id='previous';old.keep_ownership=False
        old.control_phase=S.STATE_NORMAL_FLIGHT
        self.n._on_ceiling_candidate(old)
        self.assertIs(self.n.ceiling_candidate,self.c)
        replay=copy.deepcopy(self.c);replay.thrust_normalized=.8
        self.n._on_ceiling_candidate(replay)
        self.assertIs(self.n.ceiling_candidate,self.c)

    def test_entering_prestream_retains_original_posctl_source(self):
        n=self.n
        n._operator_mode=lambda now:OperatorCommand.MODE_CEILING
        n._handover_source_ready=lambda now:True
        n._cancel_vertical_active=lambda now:False
        n.match_fcu_thrust_on_handover=True
        n.flight_state=State(mode='POSCTL')
        n.fcu_attitude_target=AttitudeTarget(orientation=Quaternion(w=1.),thrust=.53)
        n._attitude_target=lambda now,q,t,**kw:AttitudeTarget(orientation=q,thrust=t)
        n._on_manager_phase(String(data='ENTERING'))
        n._on_fcu_attitude_target(AttitudeTarget(orientation=Quaternion(w=1.),thrust=.8))
        target=n._prestream_attitude_target(self.now+rospy.Duration(.8))
        self.assertAlmostEqual(target.thrust,.53)
        self.assertAlmostEqual(n.fcu_attitude_target.thrust,.53)


class CeilingPositionGateTests(unittest.TestCase):
    def test_queued_request_cannot_take_over_without_live_motion_and_baseline(self):
        m = Manager.__new__(Manager)
        m.ceiling_distance_gate_enabled = m.ceiling_motion_enabled = True
        m.selected_mode = OperatorCommand.MODE_CEILING
        m.status_timeout_s = .5
        now = rospy.Time.from_sec(10.)
        m.ceiling_at = now
        m.ceiling = S(activation_ready=True, motion_control_enabled=True,
            motion_ready=True, baseline_ready=False)
        m.ceiling.header.stamp = now
        self.assertFalse(m._ceiling_takeover_ready(now))
        self.assertIn('平衡姿态和推力基准', m._ceiling_wait_reason(now))
        m.ceiling.motion_ready = False
        m.ceiling.motion_reason = '光流过期'
        self.assertFalse(m._ceiling_takeover_ready(now))
        self.assertIn('光流过期', m._ceiling_wait_reason(now))

    def test_altctl_is_not_a_ceiling_entry_or_return_mode(self):
        m = Manager.__new__(Manager)
        m.selected_mode = OperatorCommand.MODE_CEILING
        self.assertEqual(m._pilot_mode(), 'POSCTL')

    def test_recovery_and_position_estimate_both_required_for_handover(self):
        m = Manager.__new__(Manager)
        m.ceiling_baro_handover_enabled = True; m.selected_mode=OperatorCommand.MODE_CEILING
        m.status_timeout_s=.5; m.estimator_timeout_s=1.5
        now = rospy.Time.from_sec(10.); m.ceiling_at=m.estimator_at=now
        m.estimator = EstimatorStatus(attitude_status_flag=True, velocity_horiz_status_flag=True,
            velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True)
        m.ceiling = S(state=S.STATE_DETACH, detach_height_ready=True, baro_fresh=True, baro_reference_ready=True)
        self.assertFalse(m._ceiling_baro_handover_ready(now))
        m.ceiling.state=S.STATE_RECOVERY_HOVER; m.ceiling.recovery_ready=True
        m.ceiling.header.stamp=now;m.ceiling.recovery_attitude_locked=True;m.ceiling.recovery_lock_time=now-rospy.Duration(.2)
        m.operator=OperatorCommand(mode=OperatorCommand.MODE_CEILING,enable_control=True,source='terminal')
        m.operator.header.stamp=now;m.operator.pilot_handover_request_time=now-rospy.Duration(.1)
        m.operator_at=now;m.operator_timeout_s=.5
        self.assertTrue(m._ceiling_baro_handover_ready(now))
        m.estimator.pos_horiz_rel_status_flag=False
        self.assertFalse(m._ceiling_baro_handover_ready(now))


if __name__ == '__main__':
    unittest.main()
