"""Damping must survive the same recovery paths that continue range support."""
import math
import importlib.util
import os
import sys
import unittest
from dataclasses import replace
from unittest import mock

if importlib.util.find_spec('test_range_height_integration') is None:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_range_height_integration as fixtures

W, at = fixtures.W, fixtures.at


class RangeRecoveryDampingTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.FastRangeVelocityControlTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def seed(self, **changes):
        self.n, self.snapshot = self.fixture.seeded(**changes)
        return self.n

    def enter_abort(self, reason='detach rotation timeout'):
        self.n._enter_state(W.STATE_ABORT, at(self.fixture.seconds), reason)

    def test_rotation_timeout_keeps_fast_damping_when_slow_sign_is_wrong(self):
        n = self.seed(height=1.26, velocity=.478, fast_velocity=-.389)
        reference, base = n.detach_vertical_controller.reference, n.detach_vertical_controller.base
        self.enter_abort()
        self.assertTrue(n.detach_range_faulted)
        with mock.patch.object(n.detach_vertical_controller, 'update_range', wraps=n.detach_vertical_controller.update_range) as update:
            value = n._range_vertical_thrust(at(self.fixture.seconds))
        self.assertEqual(update.call_args.args[1], -.389)
        self.assertGreater(n.detach_vertical_controller.requested, base)
        self.assertGreater(value, .50)
        self.assertEqual((n.detach_vertical_controller.reference, n.detach_vertical_controller.base), (reference, base))
        self.assertTrue(n.detach_range_faulted)
        self.assertEqual(n.detach_task_phase, 'FAULT')
        self.assertIn('rotation timeout', n.detach_range_failure_cause)
        self.assertIn('range PD', n.detach_vertical_reason)

    def test_rising_reversal_brakes_in_abort_instead_of_continuing_old_climb_command(self):
        n = self.seed(height=1.133, velocity=-.082, fast_velocity=.956)
        self.enter_abort()
        n._range_vertical_thrust(at(self.fixture.seconds))
        self.assertLess(n.detach_vertical_controller.correction, 0.)
        self.assertLess(n.detach_vertical_controller.requested, n.detach_vertical_controller.base)

    def test_reanchored_target_uses_fast_damping_and_preserves_reference(self):
        n = self.seed(height=1.26, velocity=.478, fast_velocity=-.389)
        n.state = W.STATE_RECOVER
        n.detach_range.target_revision = 1
        n.detach_range.target = 1.5
        n.detach_vertical_controller.reanchor(1.5)
        with mock.patch.object(n.detach_vertical_controller, 'update_range', wraps=n.detach_vertical_controller.update_range) as update:
            n._range_vertical_thrust(at(self.fixture.seconds))
        self.assertEqual(update.call_args.args[1], -.389)
        self.assertEqual(n.detach_vertical_controller.reference, 1.5)
        self.assertEqual(n.detach_range.target, 1.5)
        self.assertGreater(n.detach_vertical_controller.correction, 0.)

    def test_missing_fast_velocity_in_abort_holds_owned_actual_output_not_slow_feedback(self):
        n = self.seed(velocity=-.4, fast_velocity_valid=False)
        self.enter_abort()
        n.detach_vertical_controller.integral = .03
        with mock.patch.object(n.detach_vertical_controller, 'update_range', wraps=n.detach_vertical_controller.update_range) as update:
            value = n._range_vertical_thrust(at(self.fixture.seconds))
        self.assertIsNone(update.call_args.args[1])
        self.assertTrue(math.isfinite(value))
        self.assertAlmostEqual(n.detach_vertical_controller.requested, n.detach_vertical_controller.base)
        self.assertEqual(n.detach_vertical_controller.integral, 0.)
        self.assertIn('range P', n.detach_vertical_reason)

    def test_missing_velocity_after_abort_keeps_p_and_original_fault(self):
        n = self.seed(velocity=0., fast_velocity_valid=False)
        self.enter_abort()
        for delta in (0., .3, .4):
            seconds = self.fixture.seconds+delta
            self.fixture.fx.feedback(n, seconds, tilt=0.)
            self.fixture.fx.output(n, seconds, tilt=0., thrust=.50)
            self.assertTrue(math.isfinite(n._range_vertical_thrust(at(seconds))))
        self.assertFalse(n.detach_fast_velocity_fault)
        self.assertEqual(n.detach_range_failure_cause, 'detach rotation timeout')
        self.assertEqual(n.detach_task_phase, 'FAULT')

    def test_fault_exit_does_not_switch_back_to_slow_velocity(self):
        n = self.seed(velocity=.05, fast_velocity=-.4)
        n.detach_range_faulted = True
        n.state = W.STATE_EXIT
        with mock.patch.object(n.detach_vertical_controller, 'update_range', wraps=n.detach_vertical_controller.update_range) as update:
            n._range_vertical_thrust(at(self.fixture.seconds))
        self.assertEqual(update.call_args.args[1], -.4)
        self.assertTrue(n.detach_range_faulted)
        self.assertFalse(n.detach_completed)

    def test_slow_stable_but_fast_motion_cannot_mark_abort_height_settled(self):
        n = self.seed(velocity=0., fast_velocity=.4)
        self.enter_abort()
        self.assertFalse(n._range_height_settled(at(self.fixture.seconds)))
        n.detach_range.read_snapshot.return_value = replace(self.snapshot, fast_velocity=0., fast_velocity_valid=False)
        self.assertFalse(n._range_height_settled(at(self.fixture.seconds)))


    def test_abort_before_first_range_lock_uses_the_same_damping_path(self):
        self.n = self.fixture.n
        self.enter_abort()
        self.assertFalse(self.n._range_fast_control_active())
        self.fixture.frame(10.25)
        self.assertTrue(self.n._range_fast_control_active())
        self.assertTrue(self.n.detach_range_faulted)
        self.assertEqual(self.n.detach_task_phase, 'FAULT')
        first_target = self.n.detach_range.target
        self.n._range_vertical_thrust(at(10.25))
        self.assertFalse(self.n.detach_range.read_snapshot(10.25).fast_velocity_valid)
        self.fixture.frame(10.5,distance=1.15)
        with mock.patch.object(self.n.detach_vertical_controller, 'update_range', wraps=self.n.detach_vertical_controller.update_range) as update:
            self.n._range_vertical_thrust(at(10.5))
        self.assertAlmostEqual(update.call_args.args[1], -.36, places=6)
        self.assertEqual(self.n.detach_range.target, first_target)
        self.assertEqual(self.n.detach_task_phase, 'FAULT')


    def test_descent_safety_and_thrust_use_same_velocity_in_every_range_phase(self):
        n = self.seed(velocity=-1.2, fast_velocity=-.2)
        for state in (W.STATE_DETACH_ROTATE, W.STATE_RECOVER, W.STATE_ABORT, W.STATE_EXIT):
            for slow, fast, safe in ((-1.2, -.2, True), (.2, -.81, False)):
                with self.subTest(state=state, slow=slow, fast=fast):
                    n.state = state
                    n.detach_range.read_snapshot.return_value = replace(
                        self.snapshot, velocity=slow, fast_velocity=fast)
                    self.assertEqual(n._vertical_speed_safe(), safe)
                    with mock.patch.object(n.detach_vertical_controller, 'update_range',
                                           wraps=n.detach_vertical_controller.update_range) as update:
                        n._range_vertical_thrust(at(self.fixture.seconds))
                    self.assertEqual(update.call_args.args[1], fast)

    def test_descent_limit_boundary_is_unchanged(self):
        n = self.seed(velocity=-2., fast_velocity=-.8)
        self.assertEqual(n.max_down_velocity_mps, .8)
        self.assertTrue(n._vertical_speed_safe())
        n.detach_range.read_snapshot.return_value = replace(self.snapshot, fast_velocity=-.8001)
        self.assertFalse(n._vertical_speed_safe())

    def test_invalid_fast_speed_never_substitutes_slow_speed_in_safety_or_thrust(self):
        n = self.seed(velocity=-2., fast_velocity_valid=False)
        self.assertTrue(n._vertical_speed_safe())
        with mock.patch.object(n.detach_vertical_controller, 'update_range', wraps=n.detach_vertical_controller.update_range) as update:
            self.assertTrue(math.isfinite(n._range_vertical_thrust(at(self.fixture.seconds))))
        self.assertIsNone(update.call_args.args[1])
        self.assertEqual(n.detach_vertical_controller.correction, 0.)
        self.assertFalse(n.detach_fast_velocity_fault)


if __name__ == '__main__':
    unittest.main()
