"""Regression coverage for incline return and captured wall handover parity."""
from dataclasses import replace
import math
import unittest

import test_incline_attachment as fixtures
pose = fixtures.pose
from px4_wall_ceiling_control.domain.geometry import quaternion_angle


class Planner:
    def __init__(self, angle, proposed, done=False):
        self.angle = math.radians(angle)
        self.rate, self.accel = .12, .23
        self.proposed = math.radians(proposed)
        self.max_rate = math.radians(30)
        self.max_angle = math.radians(65)
        self.segment = object()
        self.done = done

    def step(self, dt, goal):
        self.angle = self.proposed
        self.rate, self.accel = .34, .45
        return self.angle


class InclineReturnParityTests(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.InclineLogicTests()
        fixture.setUp()
        self.c, self.f = fixture.c, fixture.f
        self.f = replace(self.f, q=pose(30), applied_q=pose(30), top_valid=False)
        self.c._return(self.f)

    def step(self, **changes):
        now = self.f.now+.02
        values = dict(now=now, pose_stamp=now, output_stamp=now)
        values.update(changes)
        self.f = replace(self.f, **values)
        self.c.step(self.f)

    def test_limit_30_is_valid_but_above_wall_envelope_rejected(self):
        self.assertEqual(self.c.s.detach_max_tracking_error_deg, 30.)
        self.assertFalse(self.c.s.error(.85))
        self.assertTrue(replace(self.c.s, detach_max_tracking_error_deg=30.01).error(.85))
        self.assertEqual(self.c.s.flip_max_tracking_error_deg, 10.)

    def test_error_reducing_step_allowed_while_already_over_limit(self):
        self.c.planner = Planner(30, 29)
        self.step(q=pose(-15))
        self.assertFalse(self.c.return_tracking_limited)
        self.assertLess(quaternion_angle(self.c.q, self.f.q), math.radians(45))
        self.assertGreater(quaternion_angle(self.c.q, self.f.q), math.radians(30))

    def test_error_increasing_step_over_limit_restores_kinematics(self):
        self.c.planner = Planner(30, 31)
        old = self.c.q
        self.step(q=pose(0))
        self.assertTrue(self.c.return_tracking_limited)
        self.assertEqual(self.c.q, old)
        self.assertAlmostEqual(self.c.planner.angle, math.radians(30))
        self.assertEqual((self.c.planner.rate, self.c.planner.accel), (.12, .23))
        self.assertIsNone(self.c.planner.segment)
        self.assertFalse(self.c.planner.done)

    def test_error_increasing_step_inside_limit_is_allowed(self):
        self.c.planner = Planner(30, 31)
        self.step(q=pose(5))
        self.assertFalse(self.c.return_tracking_limited)
        self.assertGreater(self.c.axis.angle(self.c.q), math.radians(30))

    def finish(self, pitch=8, rate=30):
        self.c.planner = Planner(30, 0, done=True)
        self.step(q=pose(pitch), body_rate=(0., math.radians(rate), 0.))

    def test_recover_entry_below_10_without_rate_or_dwell_gate(self):
        self.finish()
        self.assertEqual(self.c.phase, 'RECOVER')
        self.assertFalse(self.c.recover_stable)

    def test_recover_entry_excludes_10_and_requires_finished_trajectory(self):
        self.finish(pitch=10)
        self.assertEqual(self.c.phase, 'RETURN')
        self.c.planner.done = False
        self.step(q=pose(8))
        self.assertEqual(self.c.phase, 'RETURN')

    def test_negative_pitch_after_completion_requests_land_like_wall(self):
        self.finish(pitch=-1)
        self.assertEqual(self.c.phase, 'LAND')
        self.assertIn('负', self.c.reason)

    def test_handover_uses_attitude_dwell_not_height_speed_and_stays_ready(self):
        self.finish()
        for _ in range(27):
            self.step(q=pose(8), body_rate=(0., math.radians(15), 0.),
                      height=5., vz=1., horizontal_speed=2.)
        self.assertEqual(self.c.phase, 'EXIT')
        self.assertTrue(self.c.recover_stable)
        self.step(q=pose(8))
        self.assertEqual(self.c.phase, 'EXIT')
        self.assertTrue(self.c.recover_stable)
        self.step(body_rate=(0., math.radians(21), 0.))
        self.assertEqual(self.c.phase, 'RECOVER')
        self.assertFalse(self.c.recover_stable)

    def test_repeated_pose_samples_cannot_complete_handover(self):
        self.finish()
        frozen = self.f.pose_stamp
        for _ in range(40):
            self.step(q=pose(8), body_rate=(0., 0., 0.), pose_stamp=frozen)
        self.assertEqual(self.c.phase, 'RECOVER')
        self.assertFalse(self.c.recover_stable)


class InclineCapturedHandoverTests(unittest.TestCase):
    def setUp(self):
        import test_incline_integration as boundary
        self.boundary = boundary
        self.fixture = boundary.InclineBoundaryTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.n = self.fixture.node()
        self.fixture.feedback(self.n)
        now = boundary.rospy.Time.now()
        f = self.n._incline_feedback_at(now)
        self.assertTrue(self.n.incline.start(f, pose(0), .49))
        self.n.incline.phase = 'EXIT'
        self.n.incline.recover_stable = True
        self.n.state = boundary.W.STATE_EXIT
        self.status = self.n._incline_status(now, f, True)

    def test_adapter_and_candidate_select_captured_handover(self):
        self.assertTrue(self.status.recover_attitude_locked)
        self.assertTrue(self.status.recover_stable)
        self.assertTrue(self.status.detach_completed and self.status.recovery_completed)
        c = self.n._build_control_candidate(self.status, 'incline-test', 1)
        self.assertTrue(c.recover_attitude_locked and c.incline_active)

    def test_shared_manager_still_requires_fcu_and_fusion_and_live_stability(self):
        b = self.boundary
        m = b.Manager.__new__(b.Manager)
        now = b.rospy.Time.now()
        m.wall, m.wall_at = self.status, now
        m.status_timeout_s = m.output_timeout_s = m.odom_timeout_s = .5
        m.selected_mode = b.O.MODE_WALL
        m.output_mode, m.output_at = 2, now
        m.odom, m.odom_at = self.n.odom, now
        m.odom_twist_in_child_frame = True
        m._range_px4_rc_failure = b.mock.Mock(return_value='')
        m._wall_fusion_handover_ready = b.mock.Mock(return_value=True)
        self.assertTrue(m._handover_ready(now))
        m._range_px4_rc_failure.return_value = 'FCU estimator not ready'
        self.assertFalse(m._handover_ready(now))
        m._range_px4_rc_failure.return_value = ''
        m._wall_fusion_handover_ready.return_value = False
        self.assertFalse(m._handover_ready(now))
        m._wall_fusion_handover_ready.return_value = True
        m.wall.recover_stable = False
        self.assertFalse(m._handover_ready(now))


if __name__ == '__main__':
    unittest.main()
