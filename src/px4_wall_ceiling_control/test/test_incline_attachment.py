"""Incline transitions and wall/ceiling isolation; no flight commands."""
from dataclasses import replace
import math
import unittest

from px4_wall_ceiling_control.domain.geometry import hover_and_pitched_quaternions, quaternion_angle
from px4_wall_ceiling_control.domain.incline_attachment import InclineAttachment, InclineFeedback, InclineSettings


def pose(deg):
    return hover_and_pitched_quaternions(0., math.radians(deg))[1]


class InclineLogicTests(unittest.TestCase):
    def setUp(self):
        self.settings = InclineSettings(enabled=True, flip_thrust_max=.75)
        self.c = InclineAttachment(self.settings, .85)
        self.f = InclineFeedback(10., pose(0), (0., 0., 0.), 2., 0., 0., 10.,
            front=.4, top=.5, top_stamp=10., front_valid=True, top_valid=True,
            pose_valid=True, control_valid=True, applied_q=pose(0), applied_thrust=.49,
            output_valid=True, output_stamp=10.)
        self.assertTrue(self.c.start(self.f, pose(0), .49))

    def tick(self, **changes):
        now = self.f.now + .02
        self.f = replace(self.f, now=now, pose_stamp=now, top_stamp=now,
            output_stamp=now, q=self.c.q, applied_q=self.c.q, applied_thrust=self.c.thrust)
        self.f = replace(self.f, **changes)
        self.c.step(self.f)

    def contact(self, angle=45):
        self.tick(front=.10)
        for _ in range(400):
            if self.c.tilt(self.c.q) >= math.radians(angle):
                break
            self.tick()
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        applied_q, applied_thrust = self.c.q, self.c.thrust
        self.tick(top=.065)
        self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
        self.assertEqual(self.c.q, applied_q)
        self.assertEqual(self.c.thrust, applied_thrust)
        for _ in range(45):
            self.tick(top=.065)
        self.assertEqual(self.c.phase, 'HOLD')

    def test_estimate_falls_back_without_position(self):
        self.tick()
        self.assertIsNone(self.c.estimated_angle)
        self.assertEqual(self.c.phase, 'APPROACH')

    def test_estimate_plane_and_reject_outlier(self):
        for i in range(10):
            self.tick(x=i*.012, y=0., top=.5+i*.012)
        self.assertAlmostEqual(self.c.estimated_angle, math.pi/4)
        self.tick(x=.12, y=0., top=.9)
        self.assertIsNone(self.c.estimated_angle)

    def test_estimated_infeasible_plane_lands(self):
        self.c.s = replace(self.settings, friction_coefficient=.3)
        for i in range(10):
            self.tick(x=i*.012, y=0., top=.5+i*.012)
            if self.c.phase != 'APPROACH':
                break
        self.assertEqual(self.c.phase, 'LAND')

    def test_contact_angle_is_recorded(self):
        self.contact(45)
        self.assertAlmostEqual(self.c.contact_angle, self.c.tilt(self.c.contact_q), places=2)

    def test_hold_ignores_optional_friction_support_estimate(self):
        self.contact()
        self.c.required_thrust = self.c.thrust+.01
        self.tick()
        self.assertEqual(self.c.phase, 'HOLD')

    def test_requires_explicit_calibration_and_enable(self):
        self.assertTrue(InclineSettings().error(.85))
        self.assertTrue(replace(self.settings, enabled=False).error(.85))
        for value in (0., .1, .9, math.nan, '0.7', True):
            self.assertTrue(replace(self.settings, flip_thrust_max=value).error(.85))
        self.assertFalse(self.settings.error(.85))

    def test_invalid_parameter_geometry_and_budget(self):
        for changes in ({'flip_pitch_rate_deg_s': 20.}, {'flip_max_tilt_deg': 90.},
                        {'flip_trigger_distance_m': 2.1}, {'top_contact_hold_s': 2.},
                        {'flip_duration_timeout_s': 5.}):
            self.assertTrue(replace(self.settings, **changes).error(.85))

    def test_contact_at_unknown_angles_and_return_to_hover(self):
        for angle in (30, 45, 60):
            self.setUp()
            self.contact(angle)
            self.c.step(self.f, detach=True)
            self.assertEqual(self.c.phase, 'RETURN')
            for _ in range(900):
                self.tick(top_valid=False)
                if self.c.phase in ('EXIT', 'LAND'):
                    break
            self.assertEqual(self.c.phase, 'EXIT', (angle, self.c.reason))
            self.assertFalse(self.c.fault)
            self.assertLessEqual(self.c.tilt(self.c.q), math.radians(5))

    def test_hold_does_not_freeze_or_emit_motor_outputs(self):
        self.contact()
        q, thrust = self.c.q, self.c.thrust
        for _ in range(20):
            self.tick(top=.064)
        self.assertEqual(self.c.q, q)
        self.assertEqual(self.c.thrust, thrust)

    def test_out_of_bounds_output_hands_over_in_every_active_phase(self):
        for phase in ('APPROACH', 'SLOW_FLIP', 'CONTACT_SETTLE', 'HOLD',
                      'RETURN', 'RECOVER', 'EXIT'):
            for thrust in (.05, .8, math.nan):
                self.setUp()
                self.c.phase = phase
                self.tick(applied_thrust=thrust)
                self.assertEqual(self.c.phase, 'LAND', (phase, thrust))
                self.assertTrue(self.c.fault)

    def test_repeated_contact_sample_cannot_confirm_hold(self):
        self.tick(front=.1)
        self.tick(top=.065)
        stamp = self.f.top_stamp
        for _ in range(45):
            self.tick(top_stamp=stamp)
        self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
        for _ in range(10):
            self.tick(top_stamp=stamp)
        self.assertEqual(self.c.phase, 'LAND')
        self.assertTrue(self.c.fault)

    def test_tracking_pause_stops_angle_and_thrust_but_timeout_runs(self):
        self.tick(front=.1)
        self.c.q = pose(20)
        initial_q, initial_t = self.c.q, self.c.thrust
        self.tick(q=pose(0))
        self.assertEqual(self.c.q, initial_q)
        self.assertEqual(self.c.thrust, initial_t)
        self.f = replace(self.f, now=self.c.entered+8.01)
        self.c.step(self.f)
        self.assertEqual(self.c.phase, 'LAND')

    def test_no_contact_tilt_cap_aborts(self):
        self.tick(front=.1)
        self.tick(q=pose(65))
        self.assertEqual(self.c.phase, 'LAND')

    def test_no_contact_thrust_cap_aborts(self):
        self.tick(front=.1)
        self.tick(applied_thrust=.75)
        self.assertEqual(self.c.phase, 'LAND')

    def test_contact_stops_on_boundary_without_extra_increment(self):
        self.tick(front=.1)
        self.tick(top=.065, q=pose(65), applied_q=pose(65), applied_thrust=.75)
        self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
        self.assertAlmostEqual(self.c.thrust, .75)

    def test_missing_top_during_flip_continues_search(self):
        self.tick(front=.1)
        self.tick(top_valid=False)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.assertFalse(self.c.fault)

    def test_approach_ignores_both_range_interruptions_but_not_timeout(self):
        for _ in range(100):
            self.tick(front_valid=False, top_valid=False, front=math.nan, top=math.nan)
        self.assertEqual(self.c.phase, 'APPROACH')
        self.assertFalse(self.c.fault)
        self.f = replace(self.f, now=self.c.entered+20.01)
        self.c.step(self.f)
        self.assertEqual(self.c.phase, 'LAND')
        self.assertEqual(self.c.fault, '接近超时')

    def test_one_real_18cm_frame_latches_flip_through_range_loss(self):
        self.tick(front=.18, front_valid=False)
        self.assertEqual(self.c.phase, 'APPROACH')
        self.tick(front=.180000007, front_valid=True)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        for _ in range(100):
            self.tick(front=math.nan, top=math.nan, front_valid=False, top_valid=False)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.assertFalse(self.c.fault)

    def test_contact_confirmation_pauses_during_range_interruption(self):
        self.tick(front=.18)
        self.tick(top=.065)
        self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
        for _ in range(150):
            self.tick(front_valid=False, top_valid=False)
        self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
        self.assertFalse(self.c.fault)
        for _ in range(45):
            self.tick(front_valid=False, top_valid=True, top=.065)
        self.assertEqual(self.c.phase, 'HOLD')
        self.assertFalse(self.c.fault)

    def test_return_and_recovery_finish_without_either_range(self):
        self.contact(30)
        self.c.step(replace(self.f, front_valid=False, top_valid=False), detach=True)
        self.assertEqual(self.c.phase, 'RETURN')
        for _ in range(900):
            self.tick(front_valid=False, top_valid=False)
            if self.c.phase in ('EXIT', 'LAND'):
                break
        self.assertEqual(self.c.phase, 'EXIT', self.c.reason)
        self.assertFalse(self.c.fault)

    def test_abort_during_contact_latches_land_without_restart(self):
        self.contact()
        q, thrust = self.c.q, self.c.thrust
        self.c.abort(self.f, 'cancel')
        self.assertEqual(self.c.phase, 'LAND')
        began = self.c.entered
        self.c.abort(replace(self.f, now=self.f.now+1), 'cancel again')
        self.assertEqual(self.c.entered, began)
        self.assertEqual(self.c.fault, 'cancel')
        self.assertEqual((self.c.q, self.c.thrust), (q, thrust))
        self.f = replace(self.f, now=began+10.01)
        self.c.step(self.f)
        self.assertEqual(self.c.phase, 'LAND')

    def test_hold_ignores_top_range_loss_and_distance(self):
        self.contact()
        for _ in range(100):
            self.tick(top_valid=False)
        self.assertEqual(self.c.phase, 'HOLD')
        for distance in (.081, .5, 5.):
            for _ in range(20):
                self.tick(top=distance)
            self.assertEqual(self.c.phase, 'HOLD')
        self.assertFalse(self.c.fault)

    def test_hold_ignores_body_rate_with_locked_attitude(self):
        self.contact()
        self.tick(body_rate=(0., math.radians(100.), 0.))
        self.assertEqual(self.c.phase, 'HOLD')

    def test_hold_attitude_error_above_5_degrees_lands_immediately(self):
        self.contact(45)
        locked_angle = math.degrees(self.c.tilt(self.c.contact_q))
        self.tick(q=pose(locked_angle + 4.99))
        self.assertEqual(self.c.phase, 'HOLD')
        self.tick(q=pose(locked_angle + 5.01))
        self.assertEqual(self.c.phase, 'LAND')
        self.assertIn('HOLD', self.c.fault)

    def test_hold_explicit_detach_without_top_range_still_returns(self):
        self.contact()
        self.f = replace(self.f, top_valid=False)
        self.c.step(self.f, detach=True)
        self.assertEqual(self.c.phase, 'RETURN')

    def test_flip_starts_at_18cm_not_above(self):
        self.tick(front=.1801)
        self.assertEqual(self.c.phase, 'APPROACH')
        self.tick(front=.18)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')

    def test_abort_from_every_active_phase_requests_land(self):
        for phase in self.c.ACTIVE:
            self.setUp()
            self.c.phase = phase
            self.c.abort(self.f, 'failure')
            self.assertEqual(self.c.phase, 'LAND', phase)
            self.assertEqual(self.c.fault, 'failure')

    def test_missing_pose_or_output_requests_fault_handover(self):
        self.tick(front=.1)
        self.tick(pose_valid=False)
        self.assertEqual(self.c.phase, 'LAND')
        self.setUp()
        self.tick(front=.1)
        self.tick(output_valid=False)
        self.assertEqual(self.c.phase, 'LAND')

    def test_replayed_output_does_not_seed_return_velocity(self):
        self.contact()
        self.c.step(self.f, detach=True)
        stamp = self.f.output_stamp
        for _ in range(10):
            self.tick(output_stamp=stamp)
        self.assertIsNone(self.c.planner)

    def test_clock_reversal_requests_handover(self):
        self.c.step(replace(self.f, now=9.))
        self.assertEqual(self.c.phase, 'LAND')

    def test_motion_step_is_bounded_after_stall(self):
        self.tick(front=.1)
        before = self.c.q
        self.f = replace(self.f, now=self.f.now+1., output_stamp=self.f.now+1.)
        self.c.step(self.f)
        self.assertLessEqual(quaternion_angle(before, self.c.q), math.radians(.5)+1e-8)


if __name__ == '__main__':
    unittest.main()
