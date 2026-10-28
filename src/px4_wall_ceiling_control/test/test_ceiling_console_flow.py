#!/usr/bin/env python3
import unittest
import json
from unittest import mock
import rospy
from sensor_msgs.msg import Range, FluidPressure
from nav_msgs.msg import Odometry
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, OperatorCommand, SensorHealth
from test_attachment_safety import CeilingDecision
from test_flight_handover import Manager, Terminal
from std_msgs.msg import String, UInt8
from mavros_msgs.msg import State

class CeilingConsoleFlowTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.0
        self.time = mock.patch.object(rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(self.clock));self.time.start();self.addCleanup(self.time.stop)
        overrides={"~motion_control_enabled":True,"~wall_contact_distance_m":.056,"~surface_hold_thrust":.5,"~hover_thrust":.45,"~detach_drop_m":.2,"~require_mechanism_ack":False,"~detach_timeout_s":10.}
        with mock.patch.object(rospy,"get_param",side_effect=lambda k,d=None:overrides.get(k,d)),mock.patch.object(rospy,"Publisher"),mock.patch.object(rospy,"Subscriber"),mock.patch.object(rospy,"Timer"):
            self.n=CeilingDecision()
        self.n.motion_ready=True
        self.n._flight_ready=self.n._arm_requested=self.n._mode_requested=lambda *a:True
        self.odom(2.)
    def odom(self,z,vz=0):
        m=Odometry();m.pose.pose.position.z=z;m.pose.pose.orientation.w=1;m.twist.twist.linear.z=vz
        self.n._on_odom(m)
    def sample(self,t,d):
        self.clock=t;self.n._on_range(Range(range=d,min_range=.03,max_range=40.))
    def test_invalid_range_pauses_approach_and_preserves_timeout_budget(self):
        self.n.approach_settle_time_s=0.0
        self.n._enter_state(S.STATE_APPROACH,rospy.Time.now(),"test")
        self.sample(10,.5);self.n._update_state_machine(rospy.Time.now())
        for t in [11,15,25]:
            self.sample(t,255);self.odom(2.);self.n._update_state_machine(rospy.Time.now())
            self.assertEqual(self.n.state,S.STATE_APPROACH)
            status=self.n._build_status(rospy.Time.now(),.02)
            self.assertEqual(status.approach_velocity_z_enu_mps,0.)
            self.assertTrue(status.candidate_valid)
        self.sample(25.1,.5);self.odom(2.);self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_APPROACH)
        self.assertGreater(self.n._build_status(rospy.Time.now(),.02).approach_velocity_z_enu_mps,0.)
    def test_hold_invalid_range_keeps_point_five_and_allows_detach(self):
        self.n._enter_state(S.STATE_SURFACE_HOLD,rospy.Time.now(),"test")
        self.sample(10,255);self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_SURFACE_HOLD)
        status=self.n._build_status(rospy.Time.now(),.02)
        self.assertFalse(status.range_fresh);self.assertFalse(status.fault_detected)
        self.assertAlmostEqual(status.thrust_body_z_normalized,-.5)
        self.n.detach_requested=True;self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_DETACH)
        self.assertTrue(self.n._build_status(rospy.Time.now(),.02).candidate_valid)
        for t in [10.1,10.2,10.3,10.4]:
            self.clock=t;self.odom(1.8);self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
    def test_invalid_range_does_not_hide_missing_odometry(self):
        self.n.approach_settle_time_s=0.0
        self.n._enter_state(S.STATE_APPROACH,rospy.Time.now(),"test")
        self.sample(12,255);self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_ABORT)
    def baro_hold(self):
        self.n.detach_baro_height_enabled=True;self.n.detach_drop_m=.5
        self.n._on_pressure(FluidPressure(fluid_pressure=101325.))
        self.n._enter_state(S.STATE_SURFACE_HOLD,rospy.Time.now(),"hold")
    def pressure_drop(self,drop):
        p=self.n.hold_pressure_pa*(1.+drop/44330.)**(1./.190294957)
        self.n._on_pressure(FluidPressure(fluid_pressure=p))
    def test_baro_reference_is_hold_not_detach_click_or_odometry(self):
        self.baro_hold();reference=self.n.hold_pressure_pa
        self.pressure_drop(.12);self.odom(80.)
        self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        self.assertEqual(self.n.hold_pressure_pa,reference)
        self.assertEqual(self.n.detach_start_height_m,0.)
        self.assertAlmostEqual(self.n._height_feedback(rospy.Time.now()),-.12,places=6)
    def test_baro_threshold_enters_recovery_instead_of_releasing_output(self):
        self.baro_hold();self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        self.odom(1.,-2.)
        self.pressure_drop(.49);self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        self.pressure_drop(.501);self.assertTrue(self.n._detach_height_ready(rospy.Time.now()))
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
        self.assertTrue(self.n._build_status(rospy.Time.now(),.02).detach_height_ready)
        self.assertAlmostEqual(-self.n._detach_thrust(.02),self.n.hover_thrust)
    def test_baro_stale_invalid_and_missing_reference_cannot_complete(self):
        self.baro_hold();self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        self.pressure_drop(.6);self.clock=12.;self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        for value in [0.,float('nan'),255.]:
            self.n._on_pressure(FluidPressure(fluid_pressure=value))
            self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        self.n.hold_pressure_pa=None;self.n._on_pressure(FluidPressure(fluid_pressure=101330.))
        self.assertIsNone(self.n._height_feedback(rospy.Time.now()))
    def test_new_hold_uses_new_reference_and_posctl_confirmation_clears_it(self):
        self.baro_hold();self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        self.pressure_drop(.6);self.n._detach_height_ready(rospy.Time.now())
        self.n._enter_state(S.STATE_RECOVERY_HOVER,rospy.Time.now(),"recovery")
        self.n.motion_recovery_ready=True
        self.n._on_state(State(connected=True,armed=True,mode='POSCTL'))
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_NORMAL_FLIGHT)
        self.assertIsNone(self.n.hold_pressure_pa)
        self.n._on_pressure(FluidPressure(fluid_pressure=100100.))
        self.n._enter_state(S.STATE_SURFACE_HOLD,rospy.Time.now(),"second hold")
        self.assertEqual(self.n.hold_pressure_pa,100100.)
    def confirm_ceiling(self, manager):
        now = rospy.Time.now()
        manager.ceiling.header.stamp = now
        manager.ceiling.recovery_attitude_locked = True
        manager.ceiling.recovery_lock_time = now-rospy.Duration(.2)
        manager.operator = OperatorCommand(mode=OperatorCommand.MODE_CEILING,enable_control=True,source='terminal')
        manager.operator.header.stamp = now
        manager.operator.pilot_handover_request_time = now-rospy.Duration(.1)
        manager.operator_at = now
        manager.operator_timeout_s = .5

    def test_baro_manager_bypasses_speed_gate_but_never_stale_pressure(self):
        m=Manager.__new__(Manager);m.ceiling_baro_handover_enabled=True
        m.selected_mode=OperatorCommand.MODE_CEILING;m.status_timeout_s=.5
        m.ceiling_at=m.output_at=m.estimator_at=m.odom_at=rospy.Time.now()
        m.output_mode=2;m.output_timeout_s=.3;m.estimator_timeout_s=1.5;m.odom_timeout_s=.5
        m.estimator=type('Estimate',(),{'attitude_status_flag':True,'velocity_vert_status_flag':True,'velocity_horiz_status_flag':True,'pos_horiz_rel_status_flag':True,'const_pos_mode_status_flag':False})()
        m.odom=self.n.odom;m.odom.twist.twist.linear.z=-2.;m.odom.twist.twist.angular.y=2.
        m.ceiling=S(state=S.STATE_RECOVERY_HOVER,recovery_ready=True,detach_height_ready=True,baro_fresh=True,baro_reference_ready=True)
        self.confirm_ceiling(m)
        self.assertTrue(m._handover_ready(rospy.Time.now()))
        m.ceiling.baro_fresh=False;self.assertFalse(m._handover_ready(rospy.Time.now()))
        m.ceiling.baro_fresh=True;self.clock=11.;self.assertFalse(m._handover_ready(rospy.Time.now()))
    def test_baro_descent_remains_commanded_just_before_half_metre(self):
        self.baro_hold();self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        self.pressure_drop(.499);self.odom(10.,0.)
        self.assertLess(-self.n._detach_thrust(.02),self.n.hover_thrust)
    def test_baro_timer_waits_for_ch12_before_requesting_posctl_without_owner_release(self):
        with mock.patch.object(rospy,"get_param",side_effect=lambda k,d=None:d), mock.patch.object(rospy,"Publisher"), mock.patch.object(rospy,"Subscriber"), mock.patch.object(rospy,"ServiceProxy"), mock.patch.object(rospy,"Timer"):
            m=Manager()
        m.ceiling_baro_handover_enabled=True;m.phase=Manager.ACTIVE
        m.selected_mode=OperatorCommand.MODE_CEILING
        m.flight=State(connected=True,armed=True,mode='OFFBOARD');m.flight_at=rospy.Time.now()
        m.owner=1;m.output_mode=2;m.owner_at=m.output_at=rospy.Time.now()
        m.estimator=type('Estimate',(),{'attitude_status_flag':True,'velocity_vert_status_flag':True,'velocity_horiz_status_flag':True,'pos_horiz_rel_status_flag':True,'const_pos_mode_status_flag':False})()
        m.estimator_at=m.odom_at=m.ceiling_at=rospy.Time.now()
        m.odom=self.n.odom;m.odom.twist.twist.linear.z=-2.
        m.ceiling=S(state=S.STATE_RECOVERY_HOVER,recovery_ready=True,detach_height_ready=True,baro_fresh=True,baro_reference_ready=True,reason='reached')
        m._requested_mode=lambda now:OperatorCommand.MODE_CEILING
        m._request_mode=mock.Mock()
        m.ceiling.header.stamp=rospy.Time.now();m.ceiling.recovery_attitude_locked=True
        m.ceiling.recovery_lock_time=rospy.Time.now()-rospy.Duration(.2)
        with mock.patch.dict(Manager._on_timer.__globals__,kill_clear=lambda *a:True):
            m._on_timer(None)
        self.assertEqual(m.phase,Manager.ACTIVE)
        m._request_mode.assert_not_called()
        self.confirm_ceiling(m)
        with mock.patch.dict(Manager._on_timer.__globals__,kill_clear=lambda *a:True):
            m._on_timer(None)
        self.assertEqual(m.phase,Manager.RETURNING)
        m._request_mode.assert_called_once_with('POSCTL')
        self.assertEqual(m.owner,1)
    def test_ros1_hpa_and_si_pa_normalize_to_same_height(self):
        self.n.detach_baro_height_enabled=True;self.n.detach_drop_m=.5
        self.n._on_pressure(FluidPressure(fluid_pressure=975.659))
        self.n._enter_state(S.STATE_SURFACE_HOLD,rospy.Time.now(),"hold")
        self.assertAlmostEqual(self.n.hold_pressure_pa,97565.9)
        self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        pressure_pa=self.n.hold_pressure_pa*(1.+.501/44330.)**(1./.190294957)
        for value in [pressure_pa/100.,pressure_pa]:
            self.n._on_pressure(FluidPressure(fluid_pressure=value))
            self.assertAlmostEqual(self.n._height_feedback(rospy.Time.now()),-.501,places=6)
            self.assertTrue(self.n._detach_height_ready(rospy.Time.now()))
    def ack_operator(self):
        with mock.patch.object(rospy,"get_param",side_effect=lambda k,d=None:d),mock.patch.object(rospy,"Publisher"),mock.patch.object(rospy,"Subscriber"),mock.patch.object(rospy,"Timer"),mock.patch.object(rospy,"set_param"):
            n=Terminal()
        n._on_state(State(connected=True,armed=True,mode='POSCTL'))
        n.command_pub=mock.Mock();n.command_ack_pub=mock.Mock()
        return n
    def test_ceiling_json_request_is_acknowledged_and_published_immediately(self):
        n=self.ack_operator()
        with mock.patch.object(rospy,"set_param"):
            n._on_command(String(data=json.dumps({'id':'r1','command':'ceiling'})))
        reply=json.loads(n.command_ack_pub.publish.call_args.args[0].data)
        self.assertTrue(reply['accepted']);self.assertEqual(reply['id'],'r1')
        intent=n.command_pub.publish.call_args.args[0]
        self.assertEqual(intent.mode,OperatorCommand.MODE_CEILING);self.assertTrue(intent.enable_control)
    def test_duplicate_request_id_does_not_restart_intent(self):
        n=self.ack_operator();request=String(data=json.dumps({'id':'r1','command':'ceiling'}))
        with mock.patch.object(rospy,"set_param"):
            n._on_command(request);n._on_command(request)
        self.assertEqual(n.command_pub.publish.call_count,1)
        self.assertEqual(n.command_ack_pub.publish.call_count,2)
    def test_ack_rejects_active_kill_disarmed_wrong_mode_and_stale_flight(self):
        for fault in ['kill','disarmed','mode','stale']:
            n=self.ack_operator()
            if fault=='kill':n.kill_guard_enabled=True;n._kill_report=lambda:{'valid':True,'active':True}
            if fault=='disarmed':n.flight_armed=False
            if fault=='mode':n.flight_mode='AUTO.LOITER'
            if fault=='stale':n.flight_at=rospy.Time.from_sec(5.)
            n._on_command(String(data=json.dumps({'id':'r1','command':'ceiling'})))
            self.assertFalse(json.loads(n.command_ack_pub.publish.call_args.args[0].data)['accepted'])
            n.command_pub.publish.assert_not_called();self.assertEqual(n.mode,OperatorCommand.MODE_STANDBY)
    def test_ceiling_takeover_ignores_speed_magnitude_but_requires_fresh_valid_odom(self):
        m=Manager.__new__(Manager);m.ceiling_distance_gate_enabled=True;m.selected_mode=OperatorCommand.MODE_CEILING
        m.ceiling=S(activation_ready=True);m.ceiling_at=rospy.Time.now();m.status_timeout_s=.5
        m.odom=self.n.odom;m.odom_at=rospy.Time.now();m.odom_timeout_s=.5;m.odom_twist_in_child_frame=True
        for speed in (.6,-1.0):
            m.odom.twist.twist.linear.z=speed
            self.assertTrue(m._ceiling_takeover_ready(rospy.Time.now()))
        m.odom_at=rospy.Time.from_sec(5.)
        self.assertFalse(m._ceiling_takeover_ready(rospy.Time.now()))
        self.assertIn('里程计',m._ceiling_wait_reason(rospy.Time.now()))
    def test_ceiling_decision_enters_approach_with_speed_above_old_gate(self):
        self.odom(2.,.6)
        for t in [10.,10.1,10.2,10.3]:self.sample(t,.8)
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_CEILING_ARMED)
        self.clock += .01
        self.n._on_ceiling_owner(UInt8(data=1))
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_APPROACH)
    def test_one_metre_single_frame_qualifies_without_hold_time(self):
        self.sample(10,1.01);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.1,1.);self.assertTrue(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.11,1.01);self.assertTrue(self.n._activation_ready(rospy.Time.now()))
        self.clock=10.8;self.assertFalse(self.n._activation_ready(rospy.Time.now()))

    def test_invalid_and_stale_frames_cannot_qualify_entry(self):
        self.sample(10,.8);self.assertTrue(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.1,255);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.2,.8);self.assertTrue(self.n._activation_ready(rospy.Time.now()))
        self.clock=11.;self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.sample(11.1,.8);self.assertTrue(self.n._activation_ready(rospy.Time.now()))

    def test_health_invalid_resets_gate_between_valid_ranges(self):
        for t in [10,10.1,10.2,10.3]:self.sample(t,.8)
        self.n._on_sensor_health(SensorHealth(ceiling_valid=False))
        self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.sample(10.4,.8);self.assertTrue(self.n._activation_ready(rospy.Time.now()))
    def test_click_restarts_gate_even_if_top_was_already_near(self):
        for t in [10,10.1,10.2,10.3]:self.sample(t,.8)
        op=OperatorCommand(mode=OperatorCommand.MODE_CEILING,enable_control=True)
        self.n._on_operator(op);self.assertFalse(self.n._activation_ready(rospy.Time.now()))
    def test_contact_single_frame_qualifies_without_median_or_dwell(self):
        self.n.state=S.STATE_ATTACH_CONTROL
        self.sample(10,.0561);self.assertFalse(self.n._attach_confirmed(rospy.Time.now()))
        self.sample(10.1,.056);self.assertTrue(self.n._attach_confirmed(rospy.Time.now()))
        self.sample(10.11,.06);self.assertTrue(self.n._attach_confirmed(rospy.Time.now()))
        self.clock=10.8;self.assertFalse(self.n._attach_confirmed(rospy.Time.now()))

    def test_hold_uses_exact_point_five_thrust(self):
        self.assertAlmostEqual(self.n._surface_hold_thrust(),.5)
        self.n.state=S.STATE_SURFACE_HOLD;self.sample(10,.05)
        status=self.n._build_status(rospy.Time.now(),.02)
        candidate=self.n._build_control_candidate(status)
        self.assertAlmostEqual(candidate.thrust_normalized,.5)
    def test_detach_captures_reference_and_requires_twenty_cm(self):
        self.n.state=S.STATE_SURFACE_HOLD;self.sample(10,.05)
        self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        self.assertAlmostEqual(self.n.detach_start_height_m,2.)
        self.sample(10.1,.11);self.odom(1.9)
        self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        for t in [10.2,10.3,10.4]:
            self.clock=t;self.odom(1.8);ready=self.n._detach_height_ready(rospy.Time.now())
        self.assertTrue(ready)
    def test_stale_height_and_fast_descent_block_handover(self):
        self.n.state=S.STATE_DETACH;self.n.detach_start_height_m=2.
        for t in [10,10.1,10.2]:
            self.clock=t;self.odom(1.8,-.5);self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        self.clock=12.;self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
    def test_repeated_timer_without_new_odom_does_not_confirm_descent(self):
        self.n.state=S.STATE_DETACH;self.n.detach_start_height_m=2.;self.odom(1.8)
        self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
        self.clock=10.3;self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))
    def test_height_controller_reduces_thrust_then_brakes_descent(self):
        self.n.detach_start_height_m=2.
        self.odom(2.,0);self.assertLess(-self.n._detach_thrust(.02),.45)
        self.odom(1.8,-.5);self.assertGreater(-self.n._detach_thrust(.02),.45)
        self.assertLessEqual(-self.n._detach_thrust(.02),.53)
    def test_detach_state_waits_for_height_then_recovers(self):
        self.n.state=S.STATE_SURFACE_HOLD;self.sample(10,.05)
        self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),"detach")
        for t in [10.1,10.2,10.3,10.4]:
            self.sample(t,.26);self.odom(1.9);self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_DETACH)
        for t in [10.5,10.6,10.7,10.8]:
            self.sample(t,.26);self.odom(1.8);self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
        self.n.recovery_hover_time_s=.2
        self.clock=11.01;self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_NORMAL_FLIGHT)
    def test_height_missing_and_timeout_latch_fault(self):
        for failure in ["height", "timeout"]:
            self.n.state=S.STATE_DETACH;self.n.state_entered_at=rospy.Time.from_sec(10)
            self.n.detach_start_height_m=2.;self.sample(10,.26)
            if failure=="height":self.odom(float("nan"))
            else:
                self.clock=21.;self.sample(21.,.26);self.odom(2.)
            self.n._update_state_machine(rospy.Time.now())
            self.assertEqual(self.n.state,S.STATE_ABORT)
            self.assertTrue(self.n.detach_failed)
    def test_rpc_does_not_request_offboard_after_gate_is_lost(self):
        m=Manager.__new__(Manager);m.phase=Manager.ENTERING;m.selected_mode=OperatorCommand.MODE_CEILING
        m._requested_mode=lambda *a:OperatorCommand.MODE_CEILING
        m._preflight_ready=m._stream_ready=lambda *a:True;m._gated_wall_selected=lambda:False
        m._ceiling_takeover_ready=lambda *a:False;m.set_mode=mock.Mock()
        with mock.patch.object(rospy,"wait_for_service"),mock.patch.dict(Manager._send_mode_request.__globals__,kill_clear=lambda *a:True):
            m._send_mode_request("OFFBOARD")
        m.set_mode.assert_not_called()
    def test_front_mask_persists_after_release_and_blocks_wall(self):
        n=Terminal.__new__(Terminal);n.mode=OperatorCommand.MODE_STANDBY;n.detaching=False;n.was_offboard=False;n.flight_mode="POSCTL";n.front_sensor_masked=False
        with mock.patch.object(rospy,"set_param") as write:
            n._on_command(String(data="ceiling"))
            write.assert_called_with("/attachment/front_sensor_masked",True)
        self.assertTrue(n.front_sensor_masked)
        n._release_intent();n._on_command(String(data="wall"))
        self.assertTrue(n.front_sensor_masked);self.assertEqual(n.mode,OperatorCommand.MODE_STANDBY)
    def test_front_mask_only_restores_on_confirmed_handover(self):
        n=Terminal.__new__(Terminal);n.mode=OperatorCommand.MODE_CEILING;n.detaching=False;n.was_offboard=False;n.front_sensor_masked=True;n.front_mask_seen_offboard=False
        with mock.patch.object(rospy,"set_param") as write:
            n._on_state(State(connected=True,armed=True,mode="POSCTL"))
            self.assertTrue(n.front_sensor_masked);write.assert_not_called()
            n._on_state(State(connected=True,armed=True,mode="OFFBOARD"))
            n._on_state(State(connected=False,armed=True,mode="POSCTL"))
            self.assertTrue(n.front_sensor_masked);write.assert_not_called()
            n._on_state(State(connected=True,armed=True,mode="POSCTL"))
            self.assertFalse(n.front_sensor_masked);write.assert_called_with("/attachment/front_sensor_masked",False)
    def test_operator_restart_clears_front_mask(self):
        with mock.patch.object(rospy,"get_param",side_effect=lambda k,d=None:d),mock.patch.object(rospy,"set_param") as write,mock.patch.object(rospy,"Publisher"),mock.patch.object(rospy,"Subscriber"),mock.patch.object(rospy,"Timer"):
            n=Terminal()
        self.assertFalse(n.front_sensor_masked);write.assert_called_with("/attachment/front_sensor_masked",False)
    def test_manager_requires_fresh_activation_confirmation(self):
        m=Manager.__new__(Manager);m.ceiling_distance_gate_enabled=True;m.selected_mode=OperatorCommand.MODE_CEILING
        m.status_timeout_s=.5;m.ceiling_at=rospy.Time.now();m.ceiling=S(activation_ready=False)
        m.odom=self.n.odom;m.odom_at=rospy.Time.now();m.odom_timeout_s=.5;m.odom_twist_in_child_frame=True
        self.assertFalse(m._ceiling_takeover_ready(rospy.Time.now()))
        m.ceiling.activation_ready=True;self.assertTrue(m._ceiling_takeover_ready(rospy.Time.now()))
        self.clock=11.;self.assertFalse(m._ceiling_takeover_ready(rospy.Time.now()))

if __name__=="__main__":unittest.main()
