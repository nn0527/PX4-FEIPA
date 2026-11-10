"""Behavior parity and isolation for the independently copied approach path."""
import math
import copy
import unittest
from unittest import mock
import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.domain.geometry import quaternion_angle
from px4_wall_ceiling_control.domain.horizontal_equilibrium import HorizontalEquilibrium
from px4_wall_ceiling_control.msg import WallPerchStatus as W, AttachmentControlCandidate as C, OperatorCommand as O
import test_incline_integration as fixture
from test_attachment_safety import WallDecision, OutputMux
from test_incline_attachment import pose


class InclineApproachParityTests(unittest.TestCase):
    setUp = fixture.InclineBoundaryTests.setUp
    node = fixture.InclineBoundaryTests.node
    feedback = fixture.InclineBoundaryTests.feedback
    def test_copied_control_matches_wall_for_same_feedback(self):
        n = self.node()
        self.feedback(n)
        n.state = W.STATE_SLOW_APPROACH
        n._horizontal_attitude = WallDecision._horizontal_attitude.__get__(n, WallDecision)
        n.horizontal_equilibrium = HorizontalEquilibrium()
        n.horizontal_equilibrium.ready = n.horizontal_equilibrium.frozen = True
        axis = (.04, -.02, math.sqrt(1.-.04**2-.02**2))
        n.horizontal_equilibrium.axis = n.incline_approach_equilibrium.axis = axis
        n.horizontal_takeover_yaw = n.incline_approach_yaw = .7
        n.approach_altitude_m = n.incline_approach_reference = 2.
        n.approach_base_vertical_thrust = n.incline_approach_base = .49
        for height,vz,vxy in [(2.,0.,(0.,0.)),(1.8,-.2,(.3,-.1)),(2.2,.4,(-.1,.2)),(3.,.9,(.7,-.6))]:
            with self.subTest(height=height,vz=vz):
                self.clock += .02; self.feedback(n)
                n.odom.pose.pose.position.z = height
                n.odom.twist.twist.linear.z = vz
                n.horizontal_measured = vxy
                f = n._incline_feedback_at(rospy.Time.now())
                _,q_wall,t_wall,_ = n._candidate(rospy.Time.now())
                q_incline,t_incline = n._incline_approach_control(rospy.Time.now(),f)
                self.assertLess(quaternion_angle(q_wall,q_incline),1.e-7)
                self.assertAlmostEqual(t_wall,t_incline,places=7)

    def test_incline_does_not_call_wall_candidate_or_approach_controller(self):
        n = self.node(); self.feedback(n)
        n._candidate = mock.Mock(side_effect=AssertionError('wall candidate used'))
        n._horizontal_attitude = mock.Mock(side_effect=AssertionError('wall horizontal path used'))
        n._vertical_height_thrust = mock.Mock(side_effect=AssertionError('wall vertical path used'))
        f = n._incline_feedback_at(rospy.Time.now())
        self.assertTrue(n.incline.start(f,pose(0),.49))
        n.state = W.STATE_SLOW_APPROACH
        n._incline_cycle(rospy.Time.now())
        self.assertEqual(n.incline.phase,'APPROACH')
        self.assertTrue(n.control_candidate_publisher.publish.call_args[0][0].valid)
        self.assertIsNot(n.incline_approach_controller,n.horizontal_controller)
        self.assertIsNot(n.incline_approach_equilibrium,n.horizontal_equilibrium)
        self.assertEqual(n.horizontal_controller.target,(0.,0.))

    def test_incline_height_and_speed_parameters_are_independent(self):
        n = self.node(); self.feedback(n)
        n.recovery_height_to_speed_kp = 9.
        n.horizontal_approach_speed = .3
        n.approach_altitude_m = 100.
        n.approach_base_vertical_thrust = .1
        n.odom.pose.pose.position.z = 1.9
        f = n._incline_feedback_at(rospy.Time.now())
        _,thrust = n._incline_approach_control(rospy.Time.now(),f)
        self.assertAlmostEqual(n.incline_approach_target_speed,.05)
        self.assertAlmostEqual(thrust,.505)
        self.assertAlmostEqual(n.incline_settings.horizontal_approach_speed_mps,.1)

    def test_takeover_baseline_is_captured_on_edge_and_retained(self):
        n=self.node();self.feedback(n)
        n.incline_approach_reference=n.incline_approach_base=n.incline_approach_takeover=None
        n.incline_approach_last_mode='POSCTL'
        n.odom.pose.pose.position.z=1.9
        state=State(connected=True,armed=True,mode='OFFBOARD');state.header.stamp=rospy.Time.now()
        n._incline_approach_on_state(state)
        self.assertAlmostEqual(n.incline_approach_reference,1.9)
        self.assertAlmostEqual(n.incline_approach_takeover,.49)
        self.clock += .5;self.feedback(n)
        n.incline_approach_source=(AttitudeTarget(thrust=.8),rospy.Time.from_sec(self.clock-1.))
        n.odom.pose.pose.position.z=1.5
        self.assertTrue(n._incline_approach_begin(n._incline_feedback_at(rospy.Time.now())))
        self.assertAlmostEqual(n.incline_approach_reference,1.9)
        self.assertAlmostEqual(n.incline_approach_takeover,.49)

    def test_posctl_wait_is_not_reported_as_control_feedback_failure(self):
        n=self.node();self.feedback(n)
        n.flight_state.mode='POSCTL';n.manager_phase='WAITING_WALL'
        n.horizontal_velocity_reason='水平速度反馈有效'
        n._incline_cycle(rospy.Time.now())
        status=n._incline_status(rospy.Time.now(),n.incline_feedback,False)
        self.assertEqual(n.incline.phase,'WAIT')
        self.assertNotIn('失效',status.reason)
        self.assertEqual(status.horizontal_velocity_reason,'水平速度反馈有效')

    def test_incline_approach_has_far_descent_and_timeout_protection(self):
        for cause in ('far','descent','timeout','angular'):
            with self.subTest(cause=cause):
                n = self.node(); self.feedback(n)
                f = n._incline_feedback_at(rospy.Time.now())
                self.assertTrue(n.incline.start(f,pose(0),.49))
                n.state = W.STATE_SLOW_APPROACH
                self.clock += .02; self.feedback(n,front=1.81 if cause=='far' else .4)
                if cause=='descent': n.odom.twist.twist.linear.z=-.81
                if cause=='angular': n.odom.twist.twist.angular.x=3.01
                if cause=='timeout': n.incline.entered=self.clock-n.incline_settings.approach_timeout_s-.02
                n._incline_cycle(rospy.Time.now())
                self.assertEqual(n.incline.phase,'LAND')
                self.assertTrue(n.abort_land_requested)

    def test_unhealthy_range_and_paused_velocity_cannot_trigger_flip(self):
        for cause in ('range','velocity'):
            n=self.node(); self.feedback(n)
            f=n._incline_feedback_at(rospy.Time.now())
            self.assertTrue(n.incline.start(f,pose(0),.49)); n.state=W.STATE_SLOW_APPROACH
            self.clock += .02; self.feedback(n,front=.1)
            if cause=='range': n.sensor_health.front_valid=False
            else: n.horizontal_velocity_ready=False
            n._incline_cycle(rospy.Time.now())
            self.assertEqual(n.incline.phase,'APPROACH')

    def test_approach_final_thrust_matches_wall_without_flip_ramp(self):
        m=OutputMux.__new__(OutputMux);m.max_thrust=.85;m.min_thrust=.1
        from px4_wall_ceiling_control.domain.incline_attachment import InclineSettings
        m.incline_output_settings=InclineSettings(enabled=True,flip_thrust_max=.85)
        m._wall_publication_seed=mock.Mock(return_value=(pose(0),.49,rospy.Time.from_sec(self.clock-.02),C.SOURCE_WALL))
        for demand in (.37,.60):
            target=AttitudeTarget(orientation=Quaternion(*pose(0)),thrust=demand)
            incline=C(incline_active=True,control_phase=W.STATE_SLOW_APPROACH)
            wall=C(incline_active=False,control_phase=W.STATE_SLOW_APPROACH)
            self.assertAlmostEqual(m._limit_wall_publication(rospy.Time.now(),copy.deepcopy(target),incline).thrust,
                                   m._limit_wall_publication(rospy.Time.now(),copy.deepcopy(target),wall).thrust)

    def test_copied_rc_correction_matches_wall_and_keeps_private_history(self):
        m=OutputMux.__new__(OutputMux)
        m.rc_assist_enabled=m.rc_roll_yaw_enabled=True
        m.rc_ceiling_roll_pitch_enabled=False
        m.flight_state=State(connected=True,armed=True,mode='OFFBOARD')
        m.flight_state_at=rospy.Time.now();m.state_timeout_s=1.5;m.rc_at=rospy.Time.now()
        m.rc_roll_rad=m.rc_pitch_rad=m.rc_yaw_rad=0.;m.rc_trim_at=None
        m.rc_free_max_angle_deg=5.;m.rc_yaw_max_deg=15.;m.rc_slew_deg_s=90.;m.rc_yaw_slew_deg_s=20.
        m.rc_preview_publisher=mock.Mock();m.rc_age_publisher=mock.Mock()
        m._rc_ready=mock.Mock(return_value=True)
        m._calibrated_rc_fraction=lambda axis: -.8 if axis=='roll' else .2
        m._wall_control=lambda now,candidate:candidate
        candidate=C(control_phase=W.STATE_SLOW_APPROACH)
        q=Quaternion(*pose(2.))
        wall=m._calibrated_rc_attitude(rospy.Time.now(),q,C.SOURCE_WALL,candidate)
        old=(m.rc_roll_rad,m.rc_yaw_rad)
        incline=m._incline_approach_assisted_attitude(rospy.Time.now(),q)
        self.assertLess(quaternion_angle(tuple(getattr(wall,k) for k in 'xyzw'),
                                        tuple(getattr(incline,k) for k in 'xyzw')),1.e-7)
        self.assertEqual((m.rc_roll_rad,m.rc_yaw_rad),old)
        self.assertNotEqual(m.incline_rc_roll_rad,0.)

    def test_actual_publication_pipeline_uses_private_incline_path(self):
        from px4_wall_ceiling_control.domain.incline_attachment import InclineSettings
        m=OutputMux.__new__(OutputMux)
        now=rospy.Time.now()
        c=C(source=C.SOURCE_WALL,kind=C.KIND_ATTITUDE_THRUST,incline_active=True,
            control_phase=W.STATE_SLOW_APPROACH,valid=True,phase_valid=True,
            maneuver_id='incline-private',control_sequence=1,thrust_normalized=.60,
            attitude_setpoint=Quaternion(*pose(0)))
        c.header.stamp=now
        m.wall_candidate_snapshot=(c,now);m.wall_maneuver_id=c.maneuver_id
        m._wall_fresh=mock.Mock(return_value=True);m._wall_candidate_problem=mock.Mock(return_value='')
        m.wall_candidate_timeout_s=.3;m.state_timeout_s=1.5;m.status_timeout_s=.5
        m.flight_state=State(connected=True,armed=True,mode='OFFBOARD');m.flight_state_at=now
        m.wall_status=None;m.odom=Odometry();m.odom.pose.pose.orientation=Quaternion(*pose(0))
        m.operator=O(mode=O.MODE_INCLINE,enable_control=True)
        m.max_thrust=.85;m.min_thrust=.1
        m.incline_output_settings=InclineSettings(enabled=True,flip_thrust_max=.85)
        m._attitude_target=lambda at,q,t: AttitudeTarget(orientation=q,thrust=t)
        m._assisted_attitude=mock.Mock(side_effect=AssertionError('side-wall output called'))
        m._wall_publication_seed=mock.Mock(return_value=(pose(0),.49,now-rospy.Duration(.02),C.SOURCE_WALL))
        m._publish_recovery_preview=mock.Mock()
        m.attitude_publisher=mock.Mock();m.approach_target_publisher=mock.Mock()
        m.rc_assist_enabled=True;m.rc_at=now
        m._rc_ready=mock.Mock(return_value=True)
        m._calibrated_rc_fraction=lambda axis:-.8 if axis=='roll' else 0.
        m.rc_preview_publisher=mock.Mock();m.rc_age_publisher=mock.Mock()
        m.approach_previous_target=pose(2.)
        self.assertEqual(m._publish_candidate(now,c),m.OUTPUT_ATTITUDE_THRUST)
        sent=m.attitude_publisher.publish.call_args[0][0]
        self.assertAlmostEqual(sent.thrust,.60)
        self.assertNotEqual(sent.orientation.x,0.)
        self.assertEqual(m.approach_previous_target,pose(2.))
        self.assertTrue(hasattr(m,'incline_approach_previous_target'))
        m._assisted_attitude.assert_not_called()

    def test_incline_full_roll_reaches_ten_degrees_after_final_limit(self):
        from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion, rotate_vector
        m=OutputMux.__new__(OutputMux)
        now=rospy.Time.now()
        m.flight_state=State(connected=True,armed=True,mode='OFFBOARD')
        m.flight_state_at=now;m.state_timeout_s=1.5;m.status_timeout_s=.5
        m.wall_status=None;m.rc_assist_enabled=True;m.rc_at=now
        m._rc_ready=mock.Mock(return_value=True)
        m._calibrated_rc_fraction=lambda axis:1. if axis=='roll' else 0.
        m.rc_preview_publisher=mock.Mock();m.rc_age_publisher=mock.Mock()
        m.approach_target_publisher=mock.Mock()
        roll10=(math.sin(math.radians(5.)),0.,0.,math.cos(math.radians(5.)))
        m.incline_rc_roll_rad=math.radians(10.);m.incline_rc_yaw_rad=0.;m.incline_rc_at=now
        m.incline_approach_previous_target=roll10;m.incline_approach_previous_at=now
        candidate=C(control_phase=W.STATE_SLOW_APPROACH)
        assisted=m._incline_approach_assisted_attitude(now,Quaternion(*pose(0)))
        target=AttitudeTarget(orientation=assisted,thrust=.49)
        sent=m._limit_incline_approach_target(now,target,C.SOURCE_WALL,candidate)
        q=tuple(getattr(sent.orientation,k) for k in 'xyzw')
        self.assertAlmostEqual(math.degrees(euler_from_quaternion(q)[0]),10.,places=5)
        self.assertAlmostEqual(m._incline_approach_output_settings()['max_tilt_deg'],10.)
        self.assertAlmostEqual(m._incline_approach_output_settings()['rc_roll_deg'],10.)
        # Combined automatic tilt and RC input must still obey the total cone.
        assisted=m._incline_approach_assisted_attitude(now,Quaternion(*pose(5)))
        target=AttitudeTarget(orientation=assisted,thrust=.49)
        sent=m._limit_incline_approach_target(now,target,C.SOURCE_WALL,candidate)
        q=tuple(getattr(sent.orientation,k) for k in 'xyzw')
        tilt=math.degrees(math.acos(max(-1.,min(1.,rotate_vector(q,(0.,0.,1.))[2]))))
        self.assertLessEqual(tilt,10.+1e-6)

    def test_incline_output_rejects_limits_above_ten_degrees(self):
        for key in ('max_tilt_deg','rc_roll_deg'):
            m=OutputMux.__new__(OutputMux)
            with mock.patch.object(rospy,'get_param',side_effect=lambda name,default:10.1 if name.endswith('/'+key) else default):
                with self.assertRaises(ValueError):
                    m._incline_approach_output_settings()
