"""Current-entry descent, sample-gated PI and DETACH-only 1.5 m boundary."""
import unittest
from unittest import mock
import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget
from px4_wall_ceiling_control.domain.ceiling_motion import CeilingMotionController, CeilingMotionSettings
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S
import test_ceiling_motion as motion


class DescentControllerTests(unittest.TestCase):
    def setUp(self):
        self.c=CeilingMotionController(CeilingMotionSettings())
        self.c.capture((0.,0.),0.,(0.,0.,1.),.52)
        self.c.vz_target=-.15

    def test_log_like_stall_accumulates_reduction_and_is_bounded(self):
        initial=self.c.vertical_pi(.039,1.,.30,.64)
        for _ in range(100):last=self.c.vertical_pi(.039,1.,.30,.64,.1)
        self.assertLess(last,initial-.10)
        self.assertAlmostEqual(self.c.vertical_integral,-.12)
        self.assertGreaterEqual(last,.30)

    def test_no_new_samples_or_long_gap_cannot_accumulate(self):
        for dt in (0.,0.,1.,2.,-1.):self.c.vertical_pi(.039,1.,.30,.64,dt)
        self.assertEqual(self.c.vertical_integral,0.)

    def test_saturation_stops_windup_and_fast_descent_unwinds(self):
        self.c.vertical_support=.31
        for _ in range(100):self.c.vertical_pi(0.,1.,.30,.64,.1)
        self.assertEqual(self.c.vertical_integral,0.)
        self.c.vertical_support=.52;self.c.vertical_integral=-.1
        thrust=self.c.vertical_pi(-.5,1.,.30,.64,.1)
        self.assertGreater(self.c.vertical_integral,-.1)
        self.assertGreater(thrust,.52-.1)

    def test_descent_slows_near_target_and_reverses_after_overshoot(self):
        self.assertEqual(self.c.descent_velocity(.4,.15,.5),-.15)
        self.assertAlmostEqual(self.c.descent_velocity(.01,.15,.5),-.005)
        self.assertEqual(self.c.descent_velocity(0.,.15,.5),0.)
        self.assertGreater(self.c.descent_velocity(-.02,.15,.5),0.)

    def test_overspeed_brake_overrides_negative_integral_and_preserves_limits(self):
        self.c.vertical_integral=-.12
        requested=self.c.vertical_pi(-.20,1.,.30,.64)
        thrust=self.c.descent_braking_thrust(requested,-.20,1.,.30,.64)
        self.assertAlmostEqual(thrust,.55)
        self.assertGreater(thrust,requested)
        self.assertEqual(self.c.vertical_integral,-.12)
        self.assertEqual(self.c.descent_braking_thrust(.40,-2.,.8,.30,.64),.64)
        self.assertEqual(self.c.descent_braking_thrust(.60,-.20,1.,.30,.64),.60)

    def test_brake_only_activates_above_ten_cm_per_second_downward(self):
        for measured in (-.10,-.099,0.,.20):
            self.assertEqual(self.c.descent_braking_thrust(.40,measured,1.,.30,.64),.40)
        self.assertGreater(self.c.descent_braking_thrust(.40,-.101,1.,.30,.64),.52)

    def test_quick_entry_ramp_reaches_baseline_in_point_three_seconds(self):
        self.c.begin_detach_transition(.55,.52)
        self.assertEqual(self.c.detach_transition_thrust(.46,0.),.55)
        values=[self.c.detach_transition_thrust(.46,.05) for _ in range(6)]
        self.assertAlmostEqual(values[-1],.52)
        self.assertTrue(all(a>=b for a,b in zip([.55]+values,values)))
        self.assertTrue(all(abs(a-b)<=.00501 for a,b in zip([.55]+values,values)))
        self.assertTrue(self.c.detach_transition_active)
        first_pi=self.c.detach_transition_thrust(.46,.02)
        self.assertAlmostEqual(first_pi,.516)
        for _ in range(30):value=self.c.detach_transition_thrust(.46,.02)
        self.assertAlmostEqual(value,.46)
        self.assertFalse(self.c.detach_transition_active)

    def test_pauses_and_callback_gap_do_not_skip_the_ramp(self):
        self.c.begin_detach_transition(.55,.52)
        self.assertEqual(self.c.detach_transition_thrust(.46,5.,advance=False),.55)
        self.c.detach_transition_thrust(.46,5.)
        self.assertAlmostEqual(self.c.detach_transition_elapsed,.05)
        self.assertGreater(self.c.detach_transition_command,.52)

    def test_braking_is_not_delayed_by_pressure_reduction(self):
        self.c.begin_detach_transition(.55,.52)
        thrust=self.c.detach_transition_thrust(.60,.02,brake=True)
        self.assertGreaterEqual(thrust,.60)


class DescentLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.fx=motion.CeilingMotionLifecycleTests()
        self.fx.extra_params={'~detach_height_velocity_enabled':True,'~detach_fused_height_enabled':True,
            '~detach_drop_m':.4,'~detach_height_tolerance_m':.02,'~detach_height_hold_s':.3,
            '~detach_settled_speed_mps':.05,'~detach_max_drop_m':1.5}
        self.fx.setUp();self.addCleanup(self.fx.doCleanups)
        self.n=self.fx.n;self.fx.hold()

    def begin(self,drop=0.):
        self.fx.feed(distance=.055,drop=drop)
        self.n.detach_requested=True
        self.fx.feed(distance=.055,drop=drop)
        self.assertEqual(self.n.state,S.STATE_DETACH)

    def test_fast_descent_brakes_without_waiting_for_entry_ramp_or_integral(self):
        self.begin()
        self.n.motion_controller.vertical_integral=-.12
        status=self.fx.feed(distance=.6,drop=.1,vz=-.20)
        support=self.n.motion_controller.vertical_support
        self.assertGreaterEqual(-status.thrust_body_z_normalized,support+.03-1e-6)
        self.assertGreaterEqual(status.target_vertical_speed_mps,0.)
        self.assertIn('增加推力制动',status.reason)

    def test_recovery_overspeed_brake_preserves_xy_control_and_ownership(self):
        self.begin()
        for drop in (.1,.2,.3,.4):self.fx.feed(distance=.6,drop=drop)
        for _ in range(8):self.fx.feed(distance=.6,drop=.4)
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
        self.n.motion_controller.vertical_integral=-.12
        status=self.fx.feed(distance=.6,drop=.45,vz=-.20,x=.15)
        self.assertGreater(-status.thrust_body_z_normalized,self.n.motion_controller.vertical_support)
        self.assertGreaterEqual(status.target_vertical_speed_mps,0.)
        self.assertTrue(status.keep_ownership)
        self.assertIn('增加推力制动',status.reason)

    def applied(self,owner=1):
        target=AttitudeTarget(thrust=.47,orientation=Quaternion(w=1.))
        target.header.stamp=rospy.Time.now();self.n._on_ceiling_output(target)
        return self.fx.feed(distance=.055,owner=owner)

    def test_target_uses_detach_entry_not_hold_reference_and_does_not_drift(self):
        self.begin(.08)
        self.assertAlmostEqual(self.n.detach_start_height_m,-.08)
        for drop in (.1,.2,.3):status=self.fx.feed(distance=.6,drop=drop)
        self.assertAlmostEqual(status.detach_target_height_m,-.48,places=5)
        self.assertAlmostEqual(status.detach_target_drop_m,.4,places=5)
        self.assertAlmostEqual(self.n.detach_start_height_m,-.08)

    def test_height_and_slow_speed_must_hold_on_new_samples_before_recovery(self):
        self.begin()
        for drop in (.1,.2,.3,.4):self.fx.feed(distance=.6,drop=drop,vz=-.2)
        for _ in range(6):status=self.fx.feed(distance=.6,drop=.4,vz=-.2)
        self.assertEqual(self.n.state,S.STATE_DETACH)
        self.assertFalse(status.detach_height_ready)
        for _ in range(6):status=self.fx.feed(distance=.6,drop=.4,vz=0.)
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)
        self.assertTrue(status.detach_height_ready)
        self.assertAlmostEqual(self.n.motion_recovery_height,-.4)

    def test_timer_repeats_cannot_confirm_height(self):
        self.begin()
        for drop in (.1,.2,.3,.4):self.fx.feed(distance=.6,drop=drop)
        for _ in range(30):self.n._detach_height_ready(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_DETACH)
        self.assertFalse(self.n._detach_height_ready(rospy.Time.now()))

    def test_no_forced_minimum_descent_close_to_target(self):
        self.begin()
        for _ in range(20):self.applied()
        for drop in (.1,.2,.3):self.fx.feed(distance=.6,drop=drop,vz=-.1)
        for _ in range(20):status=self.fx.feed(distance=.6,drop=.37,vz=-.1)
        self.assertEqual(self.n.state,S.STATE_DETACH)
        self.assertAlmostEqual(status.target_vertical_speed_mps,-.015,places=5)

    def test_over_1_5_metres_aborts_but_exact_boundary_does_not(self):
        self.begin()
        for drop in (.2,.4,.6,.8,1.,1.2,1.4,1.5):
            self.fx.feed(distance=.6,drop=drop,vz=-.15)
        self.assertEqual(self.n.state,S.STATE_DETACH)
        status=self.fx.feed(distance=.6,drop=1.501,vz=-.15)
        self.assertEqual(self.n.state,S.STATE_ABORT)
        self.assertTrue(status.detach_failed)
        self.assertIn('descent boundary exceeded',status.reason)

    def test_descent_boundary_is_not_checked_in_recovery(self):
        self.begin()
        self.n._enter_state(S.STATE_RECOVERY_HOVER,rospy.Time.now(),'test recovery')
        with mock.patch.object(self.n,'_height_feedback',return_value=-1.6), \
             mock.patch.object(self.n,'_ceiling_recovery_ready',return_value=False):
            self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(self.n.state,S.STATE_RECOVERY_HOVER)

    def test_real_output_and_samples_required_for_pi_and_owner_loss_freezes_it(self):
        self.begin()
        for _ in range(8):self.fx.feed(distance=.055)
        self.assertEqual(self.n.motion_controller.vertical_integral,0.)
        for _ in range(35):self.applied()
        value=self.n.motion_controller.vertical_integral
        self.assertLess(value,-.01)
        for _ in range(8):self.applied(owner=0)
        self.assertEqual(self.n.motion_controller.vertical_integral,value)
        for _ in range(5):self.n._build_status(rospy.Time.now(),.02)
        self.assertEqual(self.n.motion_controller.vertical_integral,value)

    def test_abort_clears_integral_recovery_preserves_it(self):
        self.begin();self.n.motion_controller.vertical_integral=-.04
        self.n._enter_state(S.STATE_RECOVERY_HOVER,rospy.Time.now(),'test')
        self.assertEqual(self.n.motion_controller.vertical_integral,-.04)
        self.n._enter_state(S.STATE_ABORT,rospy.Time.now(),'test')
        self.assertEqual(self.n.motion_controller.vertical_integral,0.)

    def test_entry_is_continuous_at_hold_thrust_and_then_slews_to_pi(self):
        self.n.surface_hold_thrust=.55
        self.fx.feed(distance=.055)
        self.n.detach_requested=True
        first=self.fx.feed(dt=.02,distance=.055)
        self.assertAlmostEqual(-first.thrust_body_z_normalized,.55)
        self.assertEqual(self.n.motion_controller.vertical_integral,0.)
        baseline=self.n.motion_controller.detach_transition_baseline
        previous=.55
        for _ in range(15):
            output=AttitudeTarget(thrust=previous,orientation=Quaternion(w=1.))
            output.header.stamp=rospy.Time.now();self.n._on_ceiling_output(output)
            status=self.fx.feed(dt=.02,distance=.055)
            current=-status.thrust_body_z_normalized
            self.assertLessEqual(current,previous+1e-6)
            self.assertLessEqual(previous-current,.00401)
            self.assertEqual(self.n.motion_controller.vertical_integral,0.)
            previous=current
        self.assertAlmostEqual(previous,baseline,places=5)
        self.assertEqual(status.target_vertical_speed_mps,0.)
        output=AttitudeTarget(thrust=previous,orientation=Quaternion(w=1.))
        output.header.stamp=rospy.Time.now();self.n._on_ceiling_output(output)
        status=self.fx.feed(dt=.02,distance=.055)
        self.assertLess(status.target_vertical_speed_mps,0.)
        self.assertLessEqual(abs(-status.thrust_body_z_normalized-previous),.00401)

    def test_fresh_actual_output_is_used_for_ramp_entry(self):
        self.n.surface_hold_thrust=.55
        self.fx.feed(distance=.055)
        output=AttitudeTarget(thrust=.548,orientation=Quaternion(w=1.))
        output.header.stamp=rospy.Time.now();self.n._on_ceiling_output(output)
        self.n._enter_state(S.STATE_DETACH,rospy.Time.now(),'test fresh output')
        self.assertAlmostEqual(self.n.motion_controller.detach_transition_entry,.548)


if __name__=='__main__':unittest.main()
