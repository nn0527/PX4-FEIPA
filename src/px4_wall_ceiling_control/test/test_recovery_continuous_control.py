"""Leveling is continuous and independent of valid velocity correction weight."""
import importlib.util
import math
import os
import sys
import unittest
from unittest import mock

if importlib.util.find_spec('test_wall_horizontal_velocity') is None:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from std_msgs.msg import UInt8
from px4_wall_ceiling_control.domain.recovery_attitude_limits import axis_yaw, attitude_from_axis_yaw
from px4_wall_ceiling_control.msg import WallPerchStatus as W
import test_wall_horizontal_velocity as horizontal
import test_candidate_latency as mux
import test_range_height_integration as range_fixtures


def q(tilt=0., yaw=0.):
    tilt, yaw = math.radians(tilt), math.radians(yaw)
    return attitude_from_axis_yaw((math.sin(tilt), 0., math.cos(tilt)), yaw)


def angle(a,b):
    a,b = axis_yaw(a)[0],axis_yaw(b)[0]
    return math.degrees(math.acos(max(-1.,min(1.,sum(x*y for x,y in zip(a,b))))))


class RecoveryContinuousFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.f = horizontal.HorizontalFlightTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.n = self.f.n
        self.n.state = W.STATE_ABORT
        self.n.horizontal_takeover_yaw = 0.
        self.n.q_hover = q(0.,1.912)
        self.n._flight_ready = lambda now: True
        self.n.flight_state = State(connected=True,armed=True,mode='OFFBOARD')
        self.actual = q()
        self.f.ready()

    def tick(self,t,*,vx=.1,rate=0.,tilt=0.,quality=80,actual=None):
        self.n.flight_state_received_at = rospy.Time.from_sec(t)
        self.f.feed(t,vx=vx,rate=math.radians(rate),tilt=tilt,quality=quality)
        now = rospy.Time.now()
        self.n._on_horizontal_output_owner(UInt8(data=2))
        message = AttitudeTarget(orientation=Quaternion(*(actual or self.actual)),thrust=.525)
        message.header.stamp = now
        self.n._on_horizontal_output(message)
        result = self.n._horizontal_attitude(now,self.n.q_hover)
        self.actual = result
        return result

    def warm(self):
        for i in range(1,22):
            self.tick(10.5+i*.02)
        self.assertGreater(self.n.recovery_horizontal_weight,.999)

    def test_active_and_invalid_flow_fallback_share_frozen_heading(self):
        self.warm()
        active = self.tick(11.)
        fallback = self.tick(11.02,quality=0)
        self.assertAlmostEqual(axis_yaw(active)[1],0.)
        self.assertAlmostEqual(axis_yaw(fallback)[1],0.)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertAlmostEqual(angle(fallback,q()),0.,places=6)

    def test_rate_only_suspension_keeps_healthy_epoch_but_not_velocity_output(self):
        self.warm()
        gate = self.n.horizontal_gate_at
        self.n.recovery_horizontal_controller.integral = (.02,-.01)
        value = self.tick(11.,rate=60.)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertEqual(self.n.horizontal_gate_at,gate)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(.02,-.01))
        self.assertEqual(self.n.recovery_horizontal_weight,0.)
        self.assertAlmostEqual(angle(value,q()),0.,places=6)
        self.assertIn('角速度',self.n.horizontal_velocity_reason)
        self.assertFalse(self.n.horizontal_velocity_stable)

    def test_partial_rate_weight_freezes_integral_and_reduces_correction(self):
        self.warm()
        self.n.recovery_horizontal_controller.integral = (.02,-.01)
        self.tick(11.,rate=37.5)
        self.assertAlmostEqual(self.n.recovery_horizontal_rate_weight,.5)
        self.assertLessEqual(self.n.recovery_horizontal_weight,.5)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(.02,-.01))
        self.assertFalse(self.n.recovery_horizontal_controller.integral_active)
        self.assertEqual(self.n.recovery_integral_budget,0.)

    def test_rate_recovery_blends_from_actual_output_without_reacquiring_sensors(self):
        self.warm()
        gate = self.n.horizontal_gate_at
        self.tick(11.,rate=60.)
        value = self.tick(11.02,actual=q(2.))
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_gate_at,gate)
        fraction = .02/.15
        weight = fraction*fraction*(3.-2.*fraction)
        self.assertAlmostEqual(self.n.recovery_horizontal_weight,weight)
        self.assertLess(angle(value,q(2.)),7.*weight+.001)
        self.assertTrue(self.n.recovery_soft_reentry)
        self.assertAlmostEqual(self.n.recovery_blend_duration_s,.15)
        self.assertFalse(self.n.recovery_horizontal_controller.integral_active)

    def test_fast_rotation_never_establishes_a_new_sensor_epoch(self):
        self.n._reset_horizontal_gate('new episode')
        for t in (11.,11.25,11.5):
            self.tick(t,rate=60.)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertIsNone(self.n.horizontal_gate_at)

    def test_invalid_flow_during_rate_suspension_still_clears_feedback_and_integral(self):
        self.warm()
        self.tick(11.,rate=60.)
        self.n.recovery_horizontal_controller.integral = (.02,-.01)
        self.tick(11.02,rate=60.,quality=0)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(0.,0.))
        self.tick(11.04,rate=60.)
        self.assertFalse(self.n.horizontal_velocity_ready)

    def test_large_tilt_still_resets_even_with_continuous_valid_velocity(self):
        self.warm()
        self.tick(11.,tilt=11.,rate=20.)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertIsNone(self.n.horizontal_gate_at)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(0.,0.))

    def test_stale_flow_source_cannot_be_softened_into_valid_feedback(self):
        self.warm()
        self.f.time = 11.
        self.n.horizontal_flow.header.stamp = rospy.Time.from_sec(9.)
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(0.,0.))

    def test_delayed_tick_cannot_finish_reentry_in_one_catchup_step(self):
        self.tick(10.52)
        self.tick(10.90)
        self.assertLessEqual(self.n.recovery_blend_elapsed,.07000001)
        before = self.n.recovery_blend_elapsed
        self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertEqual(self.n.recovery_blend_elapsed,before)

    def test_new_maneuver_or_feedback_reset_discards_the_previous_blend(self):
        self.warm()
        self.n._reset_horizontal_gate('new maneuver')
        self.assertIsNone(self.n.recovery_blend_seed)
        self.assertEqual(self.n.recovery_blend_elapsed,0.)
        self.assertEqual(self.n.recovery_horizontal_weight,0.)

    def test_real_feedback_recovery_reconfirms_then_blends_from_actual_output(self):
        self.warm()
        self.tick(11.,quality=0)
        self.assertIsNone(self.n.recovery_blend_seed)
        self.tick(11.02,actual=q(2.))
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.tick(11.20,actual=q(2.))
        value = self.tick(11.34,actual=q(2.))
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertLess(self.n.recovery_horizontal_weight,.1)
        self.assertLess(angle(value,q(2.)),.1)
        self.assertFalse(self.n.recovery_horizontal_controller.integral_active)

    def test_nonfinite_rate_cannot_preserve_an_established_epoch(self):
        self.warm()
        self.tick(11.,rate=float('nan'))
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertIsNone(self.n.horizontal_gate_at)

    def test_invalid_estimator_during_rate_pause_is_a_real_feedback_failure(self):
        self.warm()
        self.tick(11.,rate=60.)
        self.n.horizontal_estimator.velocity_horiz_status_flag = False
        value = self.n._horizontal_attitude(rospy.Time.now(),self.n.q_hover)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(0.,0.))
        self.assertAlmostEqual(angle(value,q()),0.,places=6)


class EarlyTiltPriorityTests(unittest.TestCase):
    setUp = RecoveryContinuousFeedbackTests.setUp
    tick = RecoveryContinuousFeedbackTests.tick
    warm = RecoveryContinuousFeedbackTests.warm

    def test_partial_tilt_weight_freezes_integral_without_resetting_feedback(self):
        self.warm()
        gate = self.n.horizontal_gate_at
        self.n.recovery_horizontal_controller.integral = (.02,-.01)
        self.tick(11.,tilt=6.5)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.horizontal_gate_at,gate)
        self.assertAlmostEqual(self.n.recovery_tilt_guard.weight,.5)
        self.assertAlmostEqual(self.n.recovery_horizontal_weight,.5)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(.02,-.01))
        self.assertFalse(self.n.recovery_horizontal_controller.integral_active)
        self.assertEqual(self.n.recovery_integral_budget,0.)
        self.assertIn('姿态回正优先',self.n.horizontal_velocity_reason)

    def test_eight_degrees_levels_without_clearing_healthy_epoch_or_heading(self):
        self.warm()
        gate = self.n.horizontal_gate_at
        self.n.recovery_horizontal_controller.integral = (.02,-.01)
        value = self.tick(11.,tilt=8.)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertEqual(self.n.horizontal_gate_at,gate)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(.02,-.01))
        self.assertAlmostEqual(angle(value,q()),0.,places=6)
        self.assertAlmostEqual(axis_yaw(value)[1],0.)

    def test_level_hold_then_resume_blends_from_actual_output(self):
        self.warm();self.tick(11.,tilt=8.)
        self.tick(11.1,tilt=5.)
        self.tick(11.3,tilt=5.)
        self.assertFalse(self.n.horizontal_velocity_active)
        value = self.tick(11.4,tilt=5.,actual=q(2.))
        self.assertTrue(self.n.horizontal_velocity_active)
        self.assertLess(self.n.recovery_horizontal_weight,.3)
        # A 100ms test tick consumes only the existing 50ms blend cap.
        self.assertAlmostEqual(self.n.recovery_blend_elapsed,.05)
        fraction = .05/.15
        self.assertAlmostEqual(self.n.recovery_horizontal_weight,fraction*fraction*(3.-2.*fraction))
        self.assertLess(angle(value,q(2.)),7.*self.n.recovery_horizontal_weight+.001)
        self.assertFalse(self.n.recovery_horizontal_controller.integral_active)

    def test_seven_degrees_does_not_release_a_level_first_latch(self):
        self.warm();self.tick(11.,tilt=8.)
        for t in (11.1,11.3,11.5):
            self.tick(t,tilt=7.)
            self.assertTrue(self.n.horizontal_velocity_ready)
            self.assertFalse(self.n.horizontal_velocity_active)
        self.assertTrue(self.n.recovery_tilt_guard.paused)

    def test_hard_tilt_reset_cannot_erase_required_level_confirmation(self):
        self.warm();self.tick(11.,tilt=11.)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertTrue(self.n.recovery_tilt_guard.paused)
        for t in (11.1,11.3,11.5):self.tick(t,tilt=7.)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertFalse(self.n.horizontal_velocity_active)
        self.assertTrue(self.n.recovery_tilt_guard.paused)

    def test_real_feedback_fault_during_level_hold_still_clears_integral(self):
        self.warm();self.tick(11.,tilt=8.);self.tick(11.1,tilt=4.)
        self.n.recovery_horizontal_controller.integral = (.02,-.01)
        self.tick(11.2,tilt=4.,quality=0)
        self.assertFalse(self.n.horizontal_velocity_ready)
        self.assertEqual(self.n.recovery_horizontal_controller.integral,(0.,0.))
        self.assertTrue(self.n.recovery_tilt_guard.paused)
        self.assertIsNone(self.n.recovery_tilt_guard.since_source)

    def test_approach_does_not_use_recovery_tilt_priority(self):
        self.warm();self.n.state = W.STATE_SLOW_APPROACH
        self.tick(11.,tilt=8.)
        self.assertTrue(self.n.horizontal_velocity_ready)
        self.assertTrue(self.n.horizontal_velocity_active)
        self.assertFalse(self.n.recovery_tilt_guard.paused)


class AbortPublicationContinuityTests(unittest.TestCase):
    def setUp(self):
        self.f = mux.CandidateLatencyTest()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.n = self.f.node
        self.f.accept(self.f.candidate())
        self.sequence = 1

    def seed(self,tilt=0.,yaw=0.):
        target = AttitudeTarget(orientation=Quaternion(*q(tilt,yaw)),thrust=.525)
        self.n._remember_publication(target,2,self.f.now)

    def publish(self,tilt=0.,yaw=0.):
        self.f.advance()
        self.sequence += 1
        c = self.f.candidate(phase=W.STATE_ABORT,sequence=self.sequence,acquire=False)
        c.attitude_setpoint = Quaternion(*q(tilt,yaw))
        c.thrust_normalized = .70
        self.f.accept(c)
        self.assertEqual(self.n._publish_candidate(self.f.now,c),self.n.OUTPUT_ATTITUDE_THRUST)
        result = self.n.attitude_publisher.publish.call_args.args[0]
        return tuple(getattr(result.orientation,k) for k in 'xyzw')

    def test_near_level_abort_limits_ordinary_axis_and_yaw_steps(self):
        self.seed(2.)
        value = self.publish(5.,60.)
        self.assertLessEqual(angle(value,q(2.)),.200001)
        self.assertLessEqual(abs(math.degrees(axis_yaw(value)[1])),.400001)
        self.assertFalse(self.n.wall_publication_slew['abort_level_immediate'])
        self.assertTrue(self.n.wall_publication_slew['abort_near_level_slew'])

    def test_repeated_active_inactive_targets_cannot_bypass_final_slew(self):
        self.seed(2.)
        previous = q(2.)
        for i in range(12):
            value = self.publish(0. if i%2 else -5.,1.912 if i%2 else 0.)
            self.assertLessEqual(angle(previous,value),.200001)
            a,b = axis_yaw(previous)[1],axis_yaw(value)[1]
            self.assertLessEqual(abs(math.degrees(math.atan2(math.sin(b-a),math.cos(b-a)))),.400001)
            previous = value

    def test_large_published_axis_still_levels_immediately_but_yaw_is_bounded(self):
        self.seed(80.)
        value = self.publish(0.,60.)
        self.assertAlmostEqual(angle(value,q()),0.,places=6)
        self.assertTrue(self.n.wall_publication_slew['abort_level_immediate'])
        self.assertLessEqual(abs(math.degrees(axis_yaw(value)[1])),.400001)

    def test_large_measured_tilt_keeps_emergency_leveling_even_with_small_seed(self):
        self.seed(2.)
        self.n.odom.pose.pose.orientation = Quaternion(*q(30.))
        value = self.publish()
        self.assertAlmostEqual(angle(value,q()),0.,places=6)
        self.assertTrue(self.n.wall_publication_slew['abort_level_immediate'])

    def test_missing_history_does_not_suppress_valid_emergency_level_command(self):
        value = self.publish()
        self.assertAlmostEqual(angle(value,q()),0.,places=6)
        self.assertFalse(self.n.wall_publication_slew['seed_available'])


class RangeAdapterGainTests(unittest.TestCase):
    def setUp(self):
        self.f = range_fixtures.FastRangeVelocityControlTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)

    def test_normal_abort_and_reanchored_range_paths_use_range_gain_not_legacy_gain(self):
        n,s = self.f.seeded(height=1.24,velocity=-.1,fast_velocity=.2)
        for state,revision in ((W.STATE_DETACH_ROTATE,0),(W.STATE_ABORT,0),(W.STATE_RECOVER,1),(W.STATE_EXIT,1)):
            n.state = state
            n.detach_range.target_revision = revision
            with mock.patch.object(n.detach_vertical_controller,'update_range',wraps=n.detach_vertical_controller.update_range) as update:
                n._range_vertical_thrust(range_fixtures.at(self.f.seconds))
            self.assertEqual(update.call_args.kwargs['velocity_kp'],.18)
            self.assertEqual(n.detach_vertical_controller.velocity_kp,.30)
        status = n._build_status(range_fixtures.at(self.f.seconds))
        self.assertIn('"control_velocity_kp": 0.18',status.detach_range_config_json)


if __name__ == '__main__':
    unittest.main()
