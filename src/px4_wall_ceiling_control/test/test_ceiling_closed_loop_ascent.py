"""Distance-closed ascent and contact confirmation, including loss/timeout boundaries."""
import sys
from pathlib import Path
import unittest
import rospy
from sensor_msgs.msg import Range
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_ceiling_motion as lifecycle


class ClosedLoopAscentTests(unittest.TestCase):
    def setUp(self):
        self.f = lifecycle.CeilingMotionLifecycleTests(methodName='runTest')
        self.f.extra_params = {'~wall_contact_distance_m': .067, '~abort_land_on_fault': True}
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.n = self.f.n

    def take_over(self, distance=3.):
        for _ in range(14):
            self.f.feed(distance=distance)
        self.assertTrue(self.n._activation_ready(rospy.Time.now()))
        self.f.mode = 'OFFBOARD'
        self.f.feed(distance=distance)
        self.f.feed(distance=distance)
        self.assertEqual(self.n.state, S.STATE_APPROACH)

    def test_valid_distant_ceiling_can_take_over_and_invalid_range_cannot(self):
        for d in (1.01, 3., 10., 30.):
            self.f.feed(distance=d)
            self.assertTrue(self.n._activation_ready(rospy.Time.now()))
        self.f.feed(distance=255.)
        self.assertFalse(self.n._activation_ready(rospy.Time.now()))
        self.take_over(3.)
        self.assertGreater(self.n.approach_budget_s, 12.)

    def test_far_start_keeps_ascending_after_old_twelve_second_deadline(self):
        self.take_over(3.)
        for _ in range(140):
            self.f.feed(distance=3.)
        self.assertEqual(self.n.state, S.STATE_APPROACH)
        self.assertGreater(self.n.motion_last_target_vz, 0.)

    def test_distance_loop_slows_near_contact_and_never_exceeds_speed_cap(self):
        self.take_over()
        speeds = []
        for d in (3., .5, .2, .1, .07):
            self.f.feed(distance=d)
            speeds.append(self.n._closed_loop_approach_velocity(rospy.Time.now()))
        self.assertTrue(all(0. <= v <= .15 for v in speeds))
        self.assertGreater(speeds[0], speeds[-1])
        self.assertLess(speeds[-1], .02)
        previous = self.n.motion_last_target_vz
        self.f.feed(distance=255.)
        self.assertEqual(self.n._closed_loop_approach_velocity(rospy.Time.now()), 0.)
        self.assertLessEqual(self.n.motion_last_target_vz, previous)
        for _ in range(10):
            self.f.feed(distance=255.)
        self.assertEqual(self.n.motion_last_target_vz, 0.)

    def test_single_near_frame_enters_contact_control_never_hold(self):
        self.take_over()
        self.f.feed(distance=.055)
        self.assertEqual(self.n.state, S.STATE_ATTACH_CONTROL)
        self.assertEqual(self.n.motion_controller.vz_target, 0.)
        self.assertEqual(self.n.motion_last_target_vz, 0.)
        self.assertAlmostEqual(self.n.attach_thrust_command,self.n._surface_hold_thrust())

    def test_contact_uses_hold_thrust_immediately_but_needs_stable_samples(self):
        self.take_over()
        self.f.feed(distance=.055)
        started = self.f.clock
        previous = self.n.attach_thrust_command
        while self.n.state == S.STATE_ATTACH_CONTROL and self.f.clock-started < 4.5:
            self.f.feed(distance=.055)
            current = self.n.attach_thrust_command
            self.assertAlmostEqual(current,self.n._surface_hold_thrust())
            previous = current
        self.assertEqual(self.n.state, S.STATE_SURFACE_HOLD)
        self.assertGreaterEqual(self.f.clock-started, .5)
        self.assertLessEqual(abs(previous-self.n.surface_hold_thrust), .00501)

    def test_repeated_timer_cannot_confirm_contact(self):
        self.take_over()
        self.f.feed(distance=.055)
        self.n.distance_filtered_m = .055
        self.n.attach_thrust_command = self.n.surface_hold_thrust
        self.n._closed_loop_contact_ready(rospy.Time.now())
        for _ in range(30):
            self.f.clock += .02
            self.n._closed_loop_contact_ready(rospy.Time.now())
        self.assertFalse(self.n._closed_loop_contact_ready(rospy.Time.now()))

    def test_unstable_contact_cannot_enter_hold(self):
        self.take_over()
        self.f.feed(distance=.055)
        for _ in range(20):
            self.f.feed(distance=.055, vz=.12)
        self.assertEqual(self.n.state, S.STATE_ATTACH_CONTROL)
        self.assertFalse(self.n._closed_loop_contact_ready(rospy.Time.now()))

    def test_fast_arrival_brakes_before_contact_control(self):
        self.take_over()
        self.f.feed(distance=.06, vz=.15)
        self.assertEqual(self.n.state, S.STATE_APPROACH)
        self.assertLess(self.n._closed_loop_approach_velocity(rospy.Time.now()), .01)
        self.f.feed(distance=.06, vz=.05)
        self.assertEqual(self.n.state, S.STATE_ATTACH_CONTROL)

    def test_range_loss_stops_ascent_then_aborts_instead_of_waiting_forever(self):
        self.take_over()
        for _ in range(25):
            self.f.feed(distance=255.)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertIn('flow height lost', self.n.last_reason)

    def test_contact_range_loss_cannot_increase_pressure_or_enter_hold(self):
        self.take_over()
        self.f.feed(distance=.065)
        thrust = self.n.attach_thrust_command
        for _ in range(5):
            self.f.feed(distance=255.)
        self.assertEqual(self.n.state, S.STATE_ATTACH_CONTROL)
        self.assertAlmostEqual(self.n.attach_thrust_command, thrust)
        self.assertIsNone(self.n.contact_stable_since)

    def test_total_budget_runs_even_when_range_wait_freezes_state_timer(self):
        self.take_over()
        self.n.approach_budget_s = .3
        for _ in range(5):
            self.f.feed(distance=255.)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertIn('time budget', self.n.last_reason)

    def test_ascent_travel_limit_aborts(self):
        self.take_over()
        self.n.approach_ascent_limit_m = .2
        for i in range(1, 5):
            self.f.feed(distance=3.-i*.1, drop=-i*.1)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertIn('travel limit', self.n.last_reason)

    def test_flow_distance_invalid_blocks_ready_and_contact_confirmation(self):
        self.take_over()
        self.f.feed(distance=.055)
        self.f.feed(distance=.055, flow_distance=0.)
        self.assertFalse(self.n.motion_ready)
        self.assertFalse(self.n._closed_loop_contact_ready(rospy.Time.now()))

    def test_unstable_contact_times_out_instead_of_becoming_hold(self):
        self.take_over()
        self.f.feed(distance=.065)
        for _ in range(55):
            self.f.feed(distance=.065,vz=.12)
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertIn('stable ceiling', self.n.last_reason)

    def test_too_close_contact_uses_hold_thrust_and_can_confirm(self):
        self.take_over()
        self.n.surface_hold_thrust=.55
        self.f.feed(distance=.024,range_min=0.)
        self.assertEqual(self.n.state,S.STATE_ATTACH_CONTROL)
        self.assertAlmostEqual(self.n.attach_thrust_command,.55)
        for _ in range(10):status=self.f.feed(distance=.014,range_min=0.)
        self.assertEqual(self.n.state,S.STATE_SURFACE_HOLD)
        self.assertAlmostEqual(-status.thrust_body_z_normalized,.55)

    def test_fast_in_range_approach_uses_exact_hold_pressure_not_velocity_formula(self):
        self.take_over()
        self.n.surface_hold_thrust=.55
        self.n.motion_controller.vertical_support=.35
        status=self.f.feed(distance=.06,vz=.15)
        self.assertEqual(self.n.state,S.STATE_APPROACH)
        self.assertAlmostEqual(-status.thrust_body_z_normalized,.55)
        self.assertEqual(status.target_vertical_speed_mps,0.)
        self.assertEqual(status.approach_velocity_z_enu_mps,0.)

    def test_contact_speed_limit_is_independent_of_detach_limit(self):
        self.take_over()
        self.n.detach_settled_speed_mps=.01
        self.assertEqual(self.n.contact_settled_speed_mps,.1)
        for _ in range(12):self.f.feed(distance=.024,vz=.06,range_min=0.)
        self.assertEqual(self.n.state,S.STATE_SURFACE_HOLD)

    def test_boundary_is_inclusive_and_far_sample_does_not_select_hold_pressure(self):
        self.take_over()
        self.n.surface_hold_thrust=.55
        self.f.feed(distance=.068,vz=.15)
        self.assertFalse(self.n._contact_pressure_requested(rospy.Time.now()))
        self.f.feed(distance=.067,vz=.15)
        self.assertTrue(self.n._contact_pressure_requested(rospy.Time.now()))
        self.assertAlmostEqual(self.n.attach_thrust_command,.55)

    def test_paused_or_invalid_feedback_cannot_select_direct_pressure(self):
        self.take_over()
        self.f.feed(distance=.06,vz=.15)
        self.n.motion_paused=True
        self.assertFalse(self.n._contact_pressure_requested(rospy.Time.now()))
        self.n.motion_paused=False
        self.f.feed(distance=255.)
        self.assertFalse(self.n._contact_pressure_requested(rospy.Time.now()))


    def test_old_near_frame_followed_by_far_frame_cannot_enter_contact(self):
        self.take_over()
        now = rospy.Time.now()
        self.n._on_range(Range(range=.055, min_range=.03, max_range=40.))
        self.n._on_range(Range(range=.10, min_range=.03, max_range=40.))
        self.n._update_state_machine(now)
        self.assertEqual(self.n.state, S.STATE_APPROACH)

    def test_complete_new_contact_then_detach_and_stable_recovery(self):
        self.take_over()
        for _ in range(42):
            self.f.feed(distance=.055)
        self.assertEqual(self.n.state, S.STATE_SURFACE_HOLD)
        self.n.detach_requested = True
        self.f.feed(distance=.055)
        self.assertEqual(self.n.state, S.STATE_DETACH)
        self.f.feed(distance=.6, drop=.51, vz=-.2)
        self.assertEqual(self.n.state, S.STATE_RECOVERY_HOVER)
        for _ in range(10):
            status = self.f.feed(distance=.6, drop=.51)
        self.assertTrue(status.recovery_ready)
        self.assertTrue(status.recovery_attitude_locked)
        self.assertTrue(status.keep_ownership)
        self.assertEqual(self.n.state, S.STATE_RECOVERY_HOVER)  # Still requires CH12 consent.

    def test_initial_world_height_is_not_an_activation_limit(self):
        for _ in range(14):
            self.f.feed(distance=3., drop=-18.)
        self.f.mode = 'OFFBOARD'
        for _ in range(2):
            self.f.feed(distance=3., drop=-18.)
        self.assertEqual(self.n.state, S.STATE_APPROACH)
        self.assertAlmostEqual(self.n.approach_start_z, 20.)
        self.assertAlmostEqual(self.n.approach_ascent_limit_m, 3.3)


if __name__ == '__main__':
    unittest.main()
