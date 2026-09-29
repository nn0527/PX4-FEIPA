#!/usr/bin/env python3

import importlib.util
import os
import unittest

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry

from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    CeilingAttachmentStatus,
    OperatorCommand,
)


SCRIPT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..", "scripts", "bench", "ceiling_motor_bench_output_node.py",
    )
)
SPEC = importlib.util.spec_from_file_location("ceiling_motor_bench_output_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
BenchOutput = MODULE.CeilingMotorBenchOutputNode


class CeilingMotorBenchTest(unittest.TestCase):
    def make_ready_node(self):
        node = BenchOutput.__new__(BenchOutput)
        node.output_enabled = True
        node.motor_enabled = True
        node.attach_thrust = 0.18
        node.hold_thrust = 0.12
        node.max_active_s = 4.0
        node.state_timeout_s = 1.5
        node.odom_timeout_s = 0.5
        node.operator_timeout_s = 0.5
        node.status_timeout_s = 0.2
        node.candidate_timeout_s = 0.2
        node.active_since_wall = None
        node.active_timeout_latched = False
        now = rospy.Time.from_sec(10.0)
        node.flight_state = State(connected=True, armed=True, mode="OFFBOARD")
        node.flight_state_at = now
        node.odom = Odometry()
        node.odom.pose.pose.orientation.w = 1.0
        node.odom_at = now
        node.operator = OperatorCommand(
            mode=OperatorCommand.MODE_CEILING, enable_control=True
        )
        node.operator_at = now
        node.status = CeilingAttachmentStatus(
            state=CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
            candidate_valid=True,
            flight_state_fresh=True,
            range_fresh=True,
            odom_fresh=True,
        )
        node.status.header.stamp = now
        node.status_at = now
        node.candidate = AttachmentControlCandidate(
            source=AttachmentControlCandidate.SOURCE_CEILING,
            kind=AttachmentControlCandidate.KIND_ATTITUDE_THRUST,
            valid=True,
            thrust_normalized=0.6,
        )
        node.candidate.header.stamp = now
        node.candidate_at = now
        return node, now

    def test_thrust_only_in_attach_and_hold_with_all_gates_ready(self):
        node, now = self.make_ready_node()
        self.assertAlmostEqual(node._desired_thrust(now, wall_now=100.0), 0.18)
        node.status.state = CeilingAttachmentStatus.STATE_SURFACE_HOLD
        self.assertAlmostEqual(node._desired_thrust(now, wall_now=100.1), 0.12)
        node.status.state = CeilingAttachmentStatus.STATE_DETACH
        self.assertEqual(node._desired_thrust(now), 0.0)

    def test_disarmed_wrong_mode_or_stale_range_forces_zero(self):
        node, now = self.make_ready_node()
        node.flight_state.armed = False
        self.assertEqual(node._desired_thrust(now), 0.0)
        node.flight_state.armed = True
        node.flight_state.mode = "STABILIZED"
        self.assertEqual(node._desired_thrust(now), 0.0)
        node.flight_state.mode = "OFFBOARD"
        node.status.range_fresh = False
        self.assertEqual(node._desired_thrust(now), 0.0)

    def test_mismatched_candidate_or_released_operator_forces_zero(self):
        node, now = self.make_ready_node()
        node.candidate.header.stamp = rospy.Time.from_sec(9.9)
        self.assertEqual(node._desired_thrust(now), 0.0)
        node.candidate.header.stamp = now
        node.operator.enable_control = False
        self.assertEqual(node._desired_thrust(now), 0.0)

    def test_active_time_latches_zero_until_disarmed_and_released(self):
        node, now = self.make_ready_node()
        self.assertAlmostEqual(node._desired_thrust(now, wall_now=100.0), 0.18)
        later = rospy.Time.from_sec(14.0)
        node.flight_state_at = later
        node.odom_at = later
        node.operator_at = later
        node.status_at = later
        node.candidate_at = later
        self.assertEqual(node._desired_thrust(later, wall_now=104.0), 0.0)
        self.assertTrue(node.active_timeout_latched)
        node.operator.enable_control = False
        node.flight_state.armed = False
        self.assertEqual(node._desired_thrust(later), 0.0)
        self.assertFalse(node.active_timeout_latched)


if __name__ == "__main__":
    unittest.main()
