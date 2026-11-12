"""Incline transitions and wall/ceiling isolation; no flight commands."""
from dataclasses import replace
import math
import unittest
from unittest.mock import Mock

from px4_wall_ceiling_control.domain.geometry import (hover_and_pitched_quaternions,
    quaternion_angle, quaternion_multiply, rotate_vector)
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
            if self.c.tilt(self.c.q) >= math.radians(angle-6):
                break
            self.tick()
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        for _ in range(27):
            self.tick(q=pose(angle), applied_q=pose(angle), top=.2)
        applied_q, applied_thrust = self.c.q, self.c.thrust
        self.tick(q=pose(angle), applied_q=pose(angle-9), top=.055)
        self.assertEqual(self.c.phase, 'HOLD')
        self.assertEqual(self.c.q, pose(angle))
        self.assertEqual(self.c.thrust, applied_thrust)
        for _ in range(45):
            self.tick(top=.055)
        self.assertEqual(self.c.phase, 'HOLD')

    def test_estimate_falls_back_without_position(self):
        self.tick()
        self.assertIsNone(self.c.estimated_angle)
        self.assertEqual(self.c.phase, 'APPROACH')

    def warm_still_pitch(self, error=9., top_valid=False):
        self.tick(front=.1)
        for _ in range(30):
            self.tick(q=pose(27), applied_q=pose(27+error), top=.2, top_valid=top_valid)
            if self.c.phase != 'SLOW_FLIP':
                break

    def test_early_still_pitch_at_or_below_8deg_does_not_stop_or_hold(self):
        for error in (0., 4., 8.):
            self.setUp()
            self.warm_still_pitch(error)
            angle = self.c.flip_angle
            self.tick(q=pose(27), applied_q=pose(27+error), top=.055, top_valid=True)
            self.assertEqual(self.c.phase, 'SLOW_FLIP')
            self.assertGreater(self.c.flip_angle, angle)

    def test_over_8deg_uses_recent_window_without_another_half_second_delay(self):
        self.warm_still_pitch(4.)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.tick(q=pose(27), applied_q=pose(36), top_valid=False)
        self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
        self.assertEqual(self.c.q, pose(36))

    def test_frozen_target_waits_without_top_or_angle_or_thrust_growth(self):
        self.warm_still_pitch()
        self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
        q, thrust = self.c.q, self.c.thrust
        for _ in range(60):
            self.tick(q=pose(27), applied_q=q, top_valid=False)
            self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
            self.assertEqual(self.c.q, q)
            self.assertEqual(self.c.thrust, thrust)
        self.tick(q=pose(27), applied_q=q, top=.069, top_valid=True)
        self.assertEqual(self.c.phase, 'HOLD')
        self.assertEqual(self.c.q, pose(27))

    def test_wait_requires_one_new_valid_strictly_below_7cm_frame(self):
        self.warm_still_pitch(top_valid=True)
        stamp = self.c.contact_top_stamp
        for changes in ({'top': .069, 'top_stamp': stamp},
                        {'top': .069, 'top_valid': False}, {'top': .070},
                        {'top': .071}, {'top': 0.}, {'top': math.nan}):
            self.tick(**dict({'q': pose(27), 'applied_q': pose(36), 'top_valid': True}, **changes))
            self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
        self.tick(q=pose(27), applied_q=pose(36), top=.069, top_valid=True)
        self.assertEqual(self.c.phase, 'HOLD')

    def test_moving_pitch_above_8deg_does_not_freeze_from_endpoints(self):
        self.tick(front=.1)
        for i in range(60):
            angle = 20.+1.2*math.sin(i*math.pi/12.5)
            self.tick(q=pose(angle), applied_q=pose(angle+9), top=.055)
            self.assertEqual(self.c.phase, 'SLOW_FLIP')

    def test_strict_one_degree_boundary_does_not_complete_search(self):
        self.tick(front=.1)
        self.c.flip_pitch_history = []
        for i in range(26):
            angle = 20.+i*.04
            self.tick(q=pose(angle), applied_q=pose(angle+9), top=.055)
            self.assertEqual(self.c.phase, 'SLOW_FLIP')

    def test_full_half_second_then_single_top_frame_can_hold_directly(self):
        self.tick(front=.1)
        for _ in range(25):
            self.tick(q=pose(27), applied_q=pose(36), top=.055)
            self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.tick(q=pose(27), applied_q=pose(36), top=.055)
        self.assertEqual(self.c.phase, 'HOLD')

    def test_duplicate_and_gapped_pose_cannot_complete_window(self):
        self.tick(front=.1)
        stamp = self.f.pose_stamp
        for _ in range(30):
            self.tick(pose_stamp=stamp, q=pose(27), applied_q=pose(36), top=.055)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        for _ in range(20):
            self.tick(q=pose(27), applied_q=pose(36), top=.055)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.f = replace(self.f, now=self.f.now+.25)
        for _ in range(20):
            self.tick(q=pose(27), applied_q=pose(36), top=.055)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')

    def test_pitch_over_13deg_aborts_immediately_in_flip_and_wait(self):
        for wait in (False, True):
            for error in (-13.001, 13.001):
                self.setUp()
                if wait:
                    self.warm_still_pitch()
                else:
                    self.tick(front=.1)
                self.tick(q=pose(27), applied_q=pose(27+error), top=.055, top_valid=True)
                self.assertEqual(self.c.phase, 'LAND')
                self.assertIn('13.000deg', self.c.fault)

    def test_exactly_13deg_does_not_abort(self):
        self.tick(front=.1)
        self.tick(q=pose(27), applied_q=pose(40), top_valid=False)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.assertFalse(self.c.fault)

    def test_wait_keeps_feedback_cancel_and_height_guards(self):
        for changes in ({'pose_valid': False}, {'output_valid': False}):
            self.setUp(); self.warm_still_pitch()
            self.tick(**changes)
            self.assertEqual(self.c.phase, 'LAND')
        self.setUp(); self.warm_still_pitch()
        self.c.step(self.f, cancel=True)
        self.assertEqual(self.c.phase, 'LAND')
        self.setUp(); self.warm_still_pitch()
        for _ in range(12):
            self.tick(q=pose(27), applied_q=pose(36), height=1.69, top_valid=False)
        self.assertEqual(self.c.phase, 'LAND')

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

    def test_single_contact_sample_enters_hold_without_confirmation_window(self):
        self.tick(front=.1)
        self.c._flip_search_complete = Mock(return_value=True)
        self.tick(top=.055)
        stamp = self.f.top_stamp
        for _ in range(45):
            self.tick(top_stamp=stamp)
        self.assertEqual(self.c.phase, 'HOLD')
        for _ in range(10):
            self.tick(top_stamp=stamp)
        self.assertEqual(self.c.phase, 'HOLD')
        self.assertFalse(self.c.fault)

    def test_large_yaw_error_does_not_pause_flip_support_or_timeout(self):
        self.tick(front=.1)
        yaw = (0., 0., math.sin(math.radians(20.)/2), math.cos(math.radians(20.)/2))
        initial_angle, initial_t = self.c.flip_angle, self.c.thrust
        for _ in range(10):
            previous = self.c.flip_angle
            self.tick(q=yaw)
            self.assertEqual(self.c.phase, 'SLOW_FLIP')
            self.assertGreater(self.c.flip_angle, previous)
            self.assertGreater(quaternion_angle(self.c.q, yaw), math.radians(10.))
        self.assertGreater(self.c.flip_angle, initial_angle)
        self.assertGreater(self.c.thrust, initial_t)
        self.f = replace(self.f, now=self.c.entered+8.01)
        self.c.step(self.f)
        self.assertEqual(self.c.phase, 'LAND')

    def test_proposed_attitude_error_does_not_block_flip_increment(self):
        self.tick(front=.1)
        self.c.flip_rate = math.radians(10.)
        yaw = (0., 0., math.sin(math.radians(9.999)/2), math.cos(math.radians(9.999)/2))
        self.assertLess(quaternion_angle(self.c.q, yaw), math.radians(10.))
        self.tick(q=yaw)
        self.assertGreater(self.c.flip_angle, 0.)
        self.assertGreater(quaternion_angle(self.c.q, yaw), math.radians(10.))

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
        self.c._flip_search_complete = Mock(return_value=True)
        self.tick(top=.055, q=pose(65), applied_q=pose(65), applied_thrust=.75)
        self.assertEqual(self.c.phase, 'HOLD')
        self.assertAlmostEqual(self.c.thrust, .75)

    def test_missing_top_during_flip_continues_search(self):
        self.tick(front=.1)
        self.tick(top_valid=False)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.assertFalse(self.c.fault)

    def test_one_frame_below_7cm_holds_without_settle_or_rate_confirmation(self):
        self.tick(front=.1)
        self.c._flip_search_complete = Mock(return_value=True)
        self.tick(top=.069, q=pose(5), body_rate=(0., math.radians(100.), 0.))
        self.assertEqual(self.c.phase, 'HOLD')
        self.assertEqual(self.c.q, pose(5))
        self.assertEqual(self.c.thrust, self.f.applied_thrust)
        self.assertTrue(self.c.contact_seen)
        self.assertFalse(self.c.fault)

    def test_7cm_boundary_and_invalid_top_do_not_enter_hold(self):
        self.tick(front=.1)
        self.c._flip_search_complete = Mock(return_value=True)
        for distance, valid in ((.071, True), (.070, True), (.069, False),
                                (math.nan, False), (math.inf, False)):
            self.tick(top=distance, top_valid=valid)
            self.assertEqual(self.c.phase, 'CONTACT_SETTLE')
            self.assertFalse(self.c.contact_seen)
            self.assertFalse(self.c.fault)
        self.tick(top=.069, top_valid=True)
        self.assertEqual(self.c.phase, 'HOLD')

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

    def test_one_frame_hold_remains_latched_through_range_interruption(self):
        self.tick(front=.18)
        self.c._flip_search_complete = Mock(return_value=True)
        self.tick(top=.055)
        self.assertEqual(self.c.phase, 'HOLD')
        for _ in range(150):
            self.tick(front_valid=False, top_valid=False)
        self.assertEqual(self.c.phase, 'HOLD')
        self.assertFalse(self.c.fault)
        for _ in range(45):
            self.tick(front_valid=False, top_valid=True, top=.055)
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

    def test_hold_attitude_error_above_10_degrees_lands_immediately(self):
        self.contact(45)
        locked_angle = math.degrees(self.c.tilt(self.c.contact_q))
        self.tick(q=pose(locked_angle + 9.99))
        self.assertEqual(self.c.phase, 'HOLD')
        self.tick(q=pose(locked_angle + 10.01))
        self.assertEqual(self.c.phase, 'LAND')
        self.assertIn('HOLD', self.c.fault)

    def test_hold_follows_twist_about_frozen_contact_axis(self):
        for angle in (30, 45, 60):
            self.setUp()
            self.contact(angle)
            reference, thrust = self.c.contact_q, self.c.thrust
            locked_z = rotate_vector(reference, (0., 0., 1.))
            for twist_deg in (30, 90, 179, -179, -90, 270):
                half = math.radians(twist_deg)/2.
                measured = quaternion_multiply(reference,
                    (0., 0., math.sin(half), math.cos(half)))
                self.tick(q=measured, top_valid=False)
                self.assertEqual(self.c.phase, 'HOLD', (angle, twist_deg))
                self.assertLess(quaternion_angle(self.c.q, measured), 1e-7)
                self.assertEqual(self.c.contact_q, reference)
                self.assertEqual(self.c.thrust, thrust)
                for a, b in zip(rotate_vector(self.c.q, (0., 0., 1.)), locked_z):
                    self.assertAlmostEqual(a, b, places=12)
                self.tick(q=tuple(-v for v in measured))
                self.assertEqual(self.c.phase, 'HOLD')
                self.assertLess(quaternion_angle(self.c.q, measured), 1e-7)

    def test_hold_axis_limit_is_independent_of_twist_and_keeps_reference(self):
        self.contact(30)
        reference = self.c.contact_q
        half = math.radians(120)/2.
        twist = (0., 0., math.sin(half), math.cos(half))
        for deviation in (6., 9.99, 10., 10.01):
            measured = quaternion_multiply(
                quaternion_multiply(reference, pose(deviation)), twist)
            self.tick(q=measured)
            self.assertEqual(self.c.phase, 'LAND' if deviation > 10. else 'HOLD')
            self.assertEqual(self.c.contact_q, reference)
            if deviation <= 10.:
                expected = quaternion_multiply(reference, twist)
                self.assertLess(quaternion_angle(self.c.q, expected), 1e-7)
        self.assertIn('Z轴', self.c.fault)

    def test_hold_world_yaw_jump_that_moves_contact_axis_still_lands(self):
        self.contact(45)
        world_yaw = hover_and_pitched_quaternions(math.radians(131.), 0.)[0]
        self.tick(q=quaternion_multiply(world_yaw, self.c.contact_q))
        self.assertEqual(self.c.phase, 'LAND')
        self.assertIn('Z轴', self.c.fault)

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


class InclineFlipSupportTests(unittest.TestCase):
    def setUp(self):
        InclineLogicTests.setUp(self)
        # These regressions isolate feedback/search behavior without entry boost.
        self.settings = replace(self.settings, flip_entry_boost=0.)
        self.c.s = self.settings

    tick = InclineLogicTests.tick
    def test_descent_compensation_and_search_continue_with_large_yaw_error(self):
        self.tick(front=.18)
        yaw = (0., 0., math.sin(math.radians(20.)/2), math.cos(math.radians(20.)/2))
        for _ in range(10):
            self.tick(q=yaw, applied_q=pose(0), height=1.95, vz=-.4)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.assertGreater(self.c.flip_angle, 0.)
        self.assertAlmostEqual(self.c.thrust, .53)
        self.assertGreater(self.c.flip_pressure_thrust, .49)

    def test_delayed_output_echo_cannot_reset_internal_support_ramp(self):
        self.tick(front=.18)
        for _ in range(20):
            previous = self.c.thrust
            self.tick(applied_thrust=.49, height=1.95, vz=-.4)
            self.assertLessEqual(self.c.thrust-previous, .004+1e-9)
        self.assertAlmostEqual(self.c.thrust, .57)
        self.assertAlmostEqual(self.c.flip_support_correction, .08)

    def test_level_no_descent_preserves_slow_search_ramp(self):
        self.tick(front=.18)
        for _ in range(20):
            self.tick(q=pose(0), height=2., vz=0., applied_thrust=.49)
        self.assertLessEqual(self.c.thrust, .49+.03*.4+1e-9)
        self.assertGreater(self.c.thrust, .500)

    def test_takeover_height_loss_is_not_erased_at_flip_entry(self):
        self.tick(front=.18, height=1.9)
        self.tick(height=1.9, vz=0.)
        self.assertEqual(self.c.reference_height, 2.)
        self.assertAlmostEqual(self.c.flip_support_requested, .52)

    def test_descent_correction_releases_when_vertical_motion_recovers(self):
        self.tick(front=.18)
        for _ in range(20):
            self.tick(q=pose(0), applied_thrust=.49, height=1.95, vz=-.4)
        high = self.c.thrust
        for _ in range(10):
            self.tick(q=pose(0), applied_thrust=.49, height=2., vz=.2)
        self.assertLess(self.c.thrust, high)
        self.assertGreaterEqual(self.c.thrust, self.c.flip_entry_thrust)

    def test_support_respects_collective_cap_and_contact_freezes_sent_value(self):
        self.tick(front=.18, applied_thrust=.74)
        self.c._flip_search_complete = Mock(return_value=True)
        self.tick(height=1.9, vz=-.4)
        self.assertLessEqual(self.c.thrust, .75)
        self.tick(top=.055, applied_thrust=.742, height=1.9, vz=-.4)
        self.assertEqual(self.c.phase, 'HOLD')
        self.assertAlmostEqual(self.c.thrust, .742)
        self.tick(height=1.8, vz=-.6, applied_thrust=.742)
        self.assertAlmostEqual(self.c.thrust, .742)

    def test_nonfinite_vertical_feedback_aborts_before_compensating(self):
        for change in ({'height': math.nan}, {'vz': math.inf}):
            self.setUp(); self.tick(front=.18)
            thrust = self.c.thrust
            self.tick(**change)
            self.assertEqual(self.c.phase, 'LAND')
            self.assertEqual(self.c.thrust, thrust)

    def test_support_parameters_reject_excess_and_nonfinite(self):
        for change in ({'flip_support_slew_per_s': .6},
                       {'flip_support_slew_per_s': .01},
                       {'flip_support_velocity_kp': .6},
                       {'flip_support_height_kp': .6},
                       {'flip_support_correction_max': .13},
                       {'flip_support_velocity_kp': math.nan}):
            self.assertTrue(replace(self.settings, **change).error(.85))


class InclineEntryBoostTests(unittest.TestCase):
    setUp = InclineLogicTests.setUp
    tick = InclineLogicTests.tick

    def test_front_gate_starts_both_boost_and_angular_search_without_entry_step(self):
        for _ in range(10):
            self.tick(front=.181)
        self.assertEqual(self.c.phase, 'APPROACH')
        self.assertEqual(self.c.flip_boost_elapsed, 0.)
        self.tick(front=.18)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.assertAlmostEqual(self.c.thrust, .49)
        self.assertEqual(self.c.flip_boost_value, 0.)
        entry_q = self.c.q
        self.tick()
        self.assertGreater(self.c.thrust, .49)
        self.assertGreater(quaternion_angle(entry_q, self.c.q), 0.)

    def test_level_flight_reaches_collective_boost_without_waiting_for_descent(self):
        self.tick(front=.18)
        for _ in range(20):
            previous = self.c.thrust
            self.tick(q=pose(0), height=2., vz=0., applied_thrust=.49)
            self.assertLessEqual(abs(self.c.thrust-previous), .004+1e-9)
        self.assertAlmostEqual(self.c.flip_boost_value, .04)
        self.assertAlmostEqual(self.c.thrust, .53)
        self.assertGreater(quaternion_angle(self.c.flip_seed, self.c.q), 0.)

    def test_boost_and_descent_share_one_correction_budget(self):
        self.tick(front=.18)
        for _ in range(25):
            self.tick(q=pose(0), height=1.95, vz=-.4, applied_thrust=.49)
        self.assertAlmostEqual(self.c.flip_boost_value, .04)
        self.assertAlmostEqual(self.c.flip_support_correction, .08)
        self.assertAlmostEqual(self.c.flip_support_requested, .57)
        self.assertAlmostEqual(self.c.thrust, .57)

    def test_ascending_feedback_can_cancel_entry_boost(self):
        self.tick(front=.18)
        for _ in range(20):
            self.tick(q=pose(0), height=2., vz=.3, applied_thrust=.49)
        self.assertLess(self.c.flip_support_correction, 0.)
        self.assertLessEqual(self.c.thrust, .49+.03*.4+1e-9)

    def test_early_contact_freezes_sent_thrust_and_stops_boost(self):
        self.tick(front=.18)
        self.c._flip_search_complete = Mock(return_value=True)
        for _ in range(5):
            self.tick()
        elapsed, boost = self.c.flip_boost_elapsed, self.c.flip_boost_value
        self.tick(top=.055, applied_thrust=.497)
        self.assertEqual(self.c.phase, 'HOLD')
        for _ in range(25):
            self.tick()
        self.assertEqual(self.c.flip_boost_elapsed, elapsed)
        self.assertEqual(self.c.flip_boost_value, boost)
        self.assertAlmostEqual(self.c.thrust, .497)

    def test_invalid_feedback_stops_boost_before_output_change(self):
        for changes in ({'output_valid': False}, {'pose_valid': False},
                        {'control_valid': False}):
            self.setUp(); self.tick(front=.18); self.tick()
            previous = self.c.thrust, self.c.flip_boost_elapsed
            self.tick(**changes)
            self.assertEqual(self.c.phase, 'LAND')
            self.assertEqual((self.c.thrust, self.c.flip_boost_elapsed), previous)

    def test_stall_cannot_skip_boost_ramp(self):
        self.tick(front=.18)
        self.tick(now=self.f.now+1.)
        self.assertAlmostEqual(self.c.flip_boost_elapsed, .05)
        self.assertLessEqual(self.c.thrust-.49, .01+1e-9)
        self.assertLess(self.c.flip_boost_value, .04)

    def test_invalid_boost_settings_are_rejected_and_zero_disables_feedforward(self):
        for changes in ({'flip_entry_boost': -.01}, {'flip_entry_boost': .081},
                        {'flip_entry_boost': math.nan}, {'flip_entry_boost': True},
                        {'flip_entry_boost_rise_s': 0.},
                        {'flip_entry_boost_rise_s': .1},
                        {'flip_entry_boost_rise_s': 8.}):
            self.assertTrue(replace(self.settings, **changes).error(.85))
        self.assertFalse(replace(self.settings, flip_entry_boost=0.).error(.85))


class InclineFlipAbortTests(unittest.TestCase):
    def setUp(self):
        self.c = InclineAttachment(InclineSettings(enabled=True, flip_thrust_max=.75), .85)
        self.f = InclineFeedback(10., pose(0), (0., 0., 0.), 2., 0., 0., 10.,
            front=.18, top=.5, top_stamp=10., front_valid=True, top_valid=True,
            pose_valid=True, control_valid=True, applied_q=pose(0), applied_thrust=.49,
            output_valid=True, output_stamp=10.)
        self.assertTrue(self.c.start(self.f, pose(0), .49))
        self.c.step(self.f)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')

    @staticmethod
    def rotation(roll=0., pitch=0., yaw=0.):
        r, p, y = (math.radians(v)/2 for v in (roll, pitch, yaw))
        return quaternion_multiply((0., 0., math.sin(y), math.cos(y)),
            quaternion_multiply((0., math.sin(p), 0., math.cos(p)),
                                (math.sin(r), 0., 0., math.cos(r))))

    def tick(self, **changes):
        now = self.f.now+.02
        self.f = replace(self.f, now=now, pose_stamp=now, output_stamp=now,
            top_stamp=now, q=pose(0), applied_q=pose(0), height=2., top=.5)
        self.f = replace(self.f, **changes)
        self.c.step(self.f)

    def test_sustained_roll_or_height_loss_aborts_and_latches(self):
        for changes, reason in (({'q': self.rotation(roll=15.)}, '横滚'),
                                ({'height': 1.69}, '掉高')):
            self.setUp()
            for _ in range(10):
                self.tick(**changes)
                self.assertEqual(self.c.phase, 'SLOW_FLIP')
            q, thrust = self.c.q, self.c.thrust
            self.tick(**changes)
            self.assertEqual(self.c.phase, 'LAND')
            self.assertIn(reason, self.c.fault)
            self.assertEqual((self.c.q, self.c.thrust), (q, thrust))
            self.tick()
            self.assertEqual(self.c.phase, 'LAND')
            self.assertFalse(self.c.start(self.f, pose(0), .49))

    def test_transient_deviation_recovers_and_clears_confirmation(self):
        for changes in ({'q': self.rotation(roll=15.)},
                        {'height': 1.69}):
            self.setUp()
            for _ in range(9):
                self.tick(**changes)
            self.tick()
            for _ in range(9):
                self.tick(**changes)
            self.assertEqual(self.c.phase, 'SLOW_FLIP')
            self.assertFalse(self.c.fault)

    def test_duplicate_pose_sample_does_not_accumulate_confirmation(self):
        for _ in range(30):
            self.tick(q=self.rotation(roll=15.), height=1.69, pose_stamp=10.)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')

    def test_pose_gap_restarts_confirmation_window(self):
        self.tick(q=self.rotation(roll=15.))
        self.tick(q=self.rotation(roll=15.), pose_stamp=self.f.pose_stamp+.30)
        stamp = self.f.pose_stamp
        for i in range(1, 10):
            self.tick(q=self.rotation(roll=15.), pose_stamp=stamp+.02*i)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.tick(q=self.rotation(roll=15.), pose_stamp=stamp+.20)
        self.assertEqual(self.c.phase, 'LAND')

    def test_below_threshold_and_height_increase_do_not_abort(self):
        for _ in range(30):
            self.tick(q=self.rotation(roll=9.9, pitch=7.9), height=1.701)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        for _ in range(30):
            self.tick(height=2.5)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')

    def test_matching_large_flip_angle_is_not_pitch_error(self):
        q = self.rotation(roll=3., pitch=55.)
        self.c.q = q
        for _ in range(20):
            self.tick(q=q, applied_q=q)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.assertFalse(self.c.fault)

    def test_published_target_is_used_instead_of_planned_attitude(self):
        self.c.q = self.rotation(pitch=20.)
        for _ in range(20):
            self.tick(q=pose(0), applied_q=pose(0))
        self.assertEqual(self.c.phase, 'SLOW_FLIP')

    def test_yaw_error_alone_retains_existing_pause_behavior(self):
        for _ in range(30):
            self.tick(q=self.rotation(yaw=40.))
        self.assertEqual(self.c.phase, 'SLOW_FLIP')
        self.assertFalse(self.c.fault)

    def test_height_baseline_is_flip_entry_not_approach_entry(self):
        self.c = InclineAttachment(self.c.s, .85)
        self.assertTrue(self.c.start(self.f, pose(0), .49))
        self.tick(height=1.)
        self.assertEqual(self.c.flip_entry_height, 1.)
        for _ in range(15):
            self.tick(height=.9)
        self.assertEqual(self.c.phase, 'SLOW_FLIP')

    def test_confirmed_fault_has_priority_over_new_contact(self):
        for _ in range(10):
            self.tick(q=self.rotation(roll=15.))
        self.tick(q=self.rotation(roll=15.), top=.055)
        self.assertEqual(self.c.phase, 'LAND')
        self.assertIn('横滚', self.c.fault)

    def test_new_height_guard_is_not_applied_in_other_phases(self):
        # Existing phase mechanics initialize RETURN/RECOVER through the real path.
        for phase in ('APPROACH', 'HOLD', 'RETURN', 'RECOVER', 'EXIT'):
            with self.subTest(phase=phase):
                fixture = InclineLogicTests()
                fixture.setUp()
                if phase != 'APPROACH':
                    fixture.contact(30)
                if phase in ('RETURN', 'RECOVER', 'EXIT'):
                    fixture.c.step(fixture.f, detach=True)
                    if phase in ('RECOVER', 'EXIT'):
                        for _ in range(900):
                            fixture.tick(top_valid=False)
                            if fixture.c.phase == phase:
                                break
                        self.assertEqual(fixture.c.phase, phase)
                elif phase == 'CONTACT_SETTLE':
                    fixture.c._enter('CONTACT_SETTLE', fixture.f.now, 'test')
                fixture.c._flip_motion_fault = Mock(side_effect=AssertionError('flip guard outside flip'))
                fixture.c.flip_entry_height = fixture.f.height
                for _ in range(15):
                    fixture.tick(height=fixture.c.flip_entry_height-.4)
                self.assertNotEqual(fixture.c.phase, 'LAND', (phase, fixture.c.reason))
                self.assertFalse(fixture.c.fault)
                fixture.c._flip_motion_fault.assert_not_called()


if __name__ == '__main__':
    unittest.main()
