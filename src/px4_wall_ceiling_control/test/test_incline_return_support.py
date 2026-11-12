"""HOLD-seeded return support with hover floor and bounded collective fall."""
from dataclasses import replace
import math
import unittest

import test_incline_attachment as fixtures
from px4_wall_ceiling_control.domain.departure_height import DepartureHeightController
from px4_wall_ceiling_control.domain.incline_return_support import InclineReturnSupport


class ReturnSupportTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.height = DepartureHeightController(max_thrust=.85, thrust_slew=.2)
        self.height.begin(4.23, .50, .588, self.now)
        self.c = InclineReturnSupport(self.height, .588, math.cos(math.radians(30)), .20, .05)

    def tick(self, **changes):
        self.now += .02
        data = dict(now=self.now, height=4.23, vz=0.,
                    vertical_fraction=math.cos(math.radians(30)),
                    actual_thrust=self.c.value, height_kp=.30, velocity_kp=.30)
        data.update(changes)
        return self.c.update(**data)

    def test_entry_command_is_exact_final_hold_collective(self):
        self.assertEqual(self.c.value, .588)
        self.assertEqual(self.c.hover_floor, .5)
        self.assertAlmostEqual(self.c.vertical_base, .588*math.cos(math.radians(30)))

    def test_recorded_ascent_feedback_cannot_cancel_tilt_support(self):
        for z, vz in ((4.44, 1.05), (4.61, 1.03), (4.69, .94), (4.59, .70), (4.32, .63)):
            previous = self.c.value
            output = self.tick(height=z, vz=vz)
            self.assertGreaterEqual(output, .5/math.cos(math.radians(30))-1e-9)
            self.assertLessEqual(previous-output, .001+1e-9)
        self.assertAlmostEqual(self.c.requested, .5/math.cos(math.radians(30)))

    def test_collective_can_decrease_as_attitude_levels_but_never_below_hover(self):
        values = []
        for i in range(151):
            angle = math.radians(30.*(1.-i/150.))
            previous = self.c.value
            output = self.tick(height=5., vz=1., vertical_fraction=math.cos(angle))
            self.assertGreaterEqual(output, .5)
            self.assertLessEqual(previous-output, .001+1e-9)
            values.append(output)
        self.assertLess(values[-1], values[0])
        self.assertAlmostEqual(values[-1], .5)

    def test_no_extra_level_confirmation_or_timed_release_stage(self):
        previous = self.c.value
        output = self.tick(vertical_fraction=1.)
        self.assertLess(output, previous)
        self.assertGreaterEqual(output, .5)

    def test_descent_can_add_support_with_cap_and_rise_slew(self):
        for _ in range(100):
            previous = self.c.value
            output = self.tick(height=3.8, vz=-1.)
            self.assertGreaterEqual(output, previous)
            self.assertLessEqual(output-previous, .004+1e-9)
            self.assertLessEqual(output, .85)
        self.assertGreater(self.c.value, .588)

    def test_delayed_actual_echo_cannot_reset_the_bounded_command_ramp(self):
        values = [self.tick(height=5., vz=1., vertical_fraction=1., actual_thrust=.588)
                  for _ in range(100)]
        self.assertTrue(all(0. <= a-b <= .001+1e-9 for a, b in zip(values, values[1:])))
        self.assertAlmostEqual(values[-1], .5)

    def test_callback_stall_does_not_cause_a_catchup_thrust_drop(self):
        self.now += 2.
        previous = self.c.value
        output = self.tick(height=5., vz=1., vertical_fraction=1.)
        self.assertLessEqual(previous-output, .05*.05+1e-9)

    def test_tilt_compensation_saturates_inside_collective_cap(self):
        for _ in range(100):
            self.tick(vertical_fraction=.4, vz=-1.)
        self.assertLessEqual(self.c.value, .85)
        self.assertTrue(self.c.saturated)


class ReturnSupportStateTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.InclineLogicTests()
        self.fixture.setUp()
        self.fixture.contact(30)
        self.c = self.fixture.c
        self.c.step(self.fixture.f, detach=True)

    def tick(self, **changes):
        self.fixture.tick(**changes)

    def test_initial_return_keeps_last_actual_hold_collective(self):
        self.assertEqual(self.c.phase, 'RETURN')
        self.assertEqual(self.c.thrust, self.fixture.f.applied_thrust)
        for _ in range(25):
            previous = self.c.thrust
            self.tick(q=fixtures.pose(30), applied_q=fixtures.pose(30), height=5., vz=1.)
            self.assertGreaterEqual(self.c.thrust, self.c.base)
            self.assertLessEqual(previous-self.c.thrust, .001+1e-9)

    def test_recovery_does_not_acquire_a_new_5_degree_or_release_wait(self):
        self.c.phase = 'RECOVER'
        self.c.recovery_started = self.fixture.f.now
        self.c.q = self.c.hover
        for _ in range(30):
            self.tick(q=fixtures.pose(8), applied_q=fixtures.pose(0),
                      body_rate=(0., math.radians(15), 0.))
        self.assertEqual(self.c.phase, 'EXIT')
        self.assertTrue(self.c.recover_stable)
        self.assertGreaterEqual(self.c.thrust, self.c.base)

    def test_actual_recovery_entry_recaptures_height_without_commanding_descent(self):
        from test_incline_return_parity import Planner
        self.c.planner = Planner(30, 0, done=True)
        self.tick(q=fixtures.pose(8), height=5.)
        self.assertEqual(self.c.phase, 'RECOVER')
        self.assertEqual(self.c.reference_height, 5.)
        self.assertEqual(self.c.support.reference, 5.)
        self.assertEqual(self.c.support.target_speed, 0.)
        self.assertGreaterEqual(self.c.thrust, self.c.base)

    def test_invalid_entry_below_hover_floor_aborts_instead_of_jumping_collective(self):
        self.fixture.setUp()
        self.c = self.fixture.c
        self.c.phase = 'HOLD'
        self.fixture.f = replace(self.fixture.f, applied_thrust=self.c.base-.02)
        self.c._return(self.fixture.f)
        self.assertEqual(self.c.phase, 'LAND')
        self.assertIn('悬停基准', self.c.fault)

    def test_drop_slew_cannot_exceed_return_rise_envelope(self):
        self.assertTrue(replace(self.c.s, return_thrust_drop_slew_per_s=.21).error(.85))


class ReturnFinalPublicationTests(unittest.TestCase):
    def test_normal_return_fall_is_slow_but_abort_envelope_is_preserved(self):
        import test_incline_integration as boundary
        from px4_wall_ceiling_control.domain.incline_attachment import InclineSettings
        m = boundary.OutputMux.__new__(boundary.OutputMux)
        m.max_thrust, m.min_thrust = .85, .1
        m.incline_output_settings = InclineSettings(enabled=True, flip_thrust_max=.75)
        now = boundary.rospy.Time.from_sec(10.)
        m._wall_publication_seed = lambda unused: (
            fixtures.pose(0), .588, boundary.rospy.Time.from_sec(9.98), boundary.C.SOURCE_WALL)
        for phase, expected in ((boundary.W.STATE_DETACH_ROTATE, .587),
                                (boundary.W.STATE_RECOVER, .587),
                                (boundary.W.STATE_EXIT, .587),
                                (boundary.W.STATE_ABORT, .584)):
            target = boundary.AttitudeTarget(orientation=boundary.Quaternion(*fixtures.pose(0)), thrust=.2)
            result = m._limit_incline_publication(now, target,
                boundary.C(incline_active=True, control_phase=phase))
            self.assertAlmostEqual(result.thrust, expected, places=6)


if __name__ == '__main__':
    unittest.main()
