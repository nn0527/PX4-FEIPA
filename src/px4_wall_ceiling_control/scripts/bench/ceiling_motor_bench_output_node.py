#!/usr/bin/env python3
"""Bounded, zero-thrust-first MAVROS output for a propeller-free ceiling bench.

This node is the only MAVROS setpoint publisher in the dedicated bench launch.
It never arms PX4 or changes modes.  APPROACH is observed from paperboard range
changes; it is deliberately not translated into a motion command on a fixed rig.
"""

import math
import time

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt8

from px4_wall_ceiling_control.adapters.mavros_setpoint import MavrosSetpointBuilder
from px4_wall_ceiling_control.adapters.odometry import odometry_is_finite
from px4_wall_ceiling_control.freshness import is_fresh
from px4_wall_ceiling_control.msg import (
    AttachmentControlCandidate,
    CeilingAttachmentStatus,
    OperatorCommand,
)


class CeilingMotorBenchOutputNode:
    OUTPUT_DISABLED = 0
    OUTPUT_ZERO_THRUST = 1
    OUTPUT_BENCH_THRUST = 2

    def __init__(self):
        self.output_enabled = bool(rospy.get_param("~output_enabled", False))
        self.motor_enabled = bool(rospy.get_param("~motor_enabled", False))
        self.max_thrust = float(rospy.get_param("~max_thrust", 0.20))
        self.attach_thrust = float(rospy.get_param("~attach_thrust", 0.18))
        self.hold_thrust = float(rospy.get_param("~hold_thrust", 0.12))
        self.max_active_s = float(rospy.get_param("~max_active_s", 4.0))
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 1.5))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.5))
        self.operator_timeout_s = float(rospy.get_param("~operator_timeout_s", 0.5))
        self.candidate_timeout_s = float(rospy.get_param("~candidate_timeout_s", 0.2))
        self.status_timeout_s = float(rospy.get_param("~status_timeout_s", 0.2))

        if not (0.0 < self.max_thrust <= 0.25):
            raise ValueError("bench max_thrust must be in (0, 0.25]")
        if not (0.0 <= self.hold_thrust <= self.attach_thrust <= self.max_thrust):
            raise ValueError("bench thrust values must satisfy 0 <= hold <= attach <= max")
        if not (0.0 < self.max_active_s <= 10.0):
            raise ValueError("bench max_active_s must be in (0, 10]")

        self.flight_state = None
        self.flight_state_at = None
        self.odom = None
        self.odom_at = None
        self.operator = None
        self.operator_at = None
        self.status = None
        self.status_at = None
        self.candidate = None
        self.candidate_at = None
        self.active_since_wall = None
        self.active_timeout_latched = False
        self.last_orientation = Quaternion(w=1.0)

        self.builder = MavrosSetpointBuilder(0.0, 0.0, self.max_thrust)
        self.setpoint_publisher = None
        if self.output_enabled:
            self.setpoint_publisher = rospy.Publisher(
                rospy.get_param(
                    "~attitude_setpoint_topic", "/mavros/setpoint_raw/attitude"
                ),
                AttitudeTarget,
                queue_size=10,
            )
        self.mode_publisher = rospy.Publisher(
            rospy.get_param("~mode_topic", "/bench/motor_output_mode"),
            UInt8,
            queue_size=10,
            latch=True,
        )
        self.thrust_publisher = rospy.Publisher(
            rospy.get_param("~thrust_topic", "/bench/motor_thrust_command"),
            Float32,
            queue_size=10,
        )

        rospy.Subscriber(
            rospy.get_param("~state_topic", "/mavros/state"),
            State,
            self._on_state,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/mavros/local_position/odom"),
            Odometry,
            self._on_odom,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~operator_topic", "/bench/operator/command"),
            OperatorCommand,
            self._on_operator,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~status_topic", "/bench/ceiling_attachment/status"),
            CeilingAttachmentStatus,
            self._on_status,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~candidate_topic", "/bench/ceiling_attachment/control_candidate"
            ),
            AttachmentControlCandidate,
            self._on_candidate,
            queue_size=10,
        )
        rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
        if rate_hz < 10.0:
            raise ValueError("bench setpoint stream must be at least 10 Hz")
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate_hz), self._on_timer)
        rospy.logwarn(
            "ceiling motor bench output=%s motor=%s max thrust=%.3f max active=%.1fs",
            self.output_enabled,
            self.motor_enabled,
            self.max_thrust,
            self.max_active_s,
        )

    def _on_state(self, message):
        self.flight_state = message
        self.flight_state_at = rospy.Time.now()

    def _on_odom(self, message):
        self.odom = message
        self.odom_at = rospy.Time.now()
        if odometry_is_finite(message):
            q = message.pose.pose.orientation
            values = (q.x, q.y, q.z, q.w)
            norm = math.sqrt(sum(value * value for value in values))
            if norm > 1.0e-6:
                self.last_orientation = Quaternion(
                    x=q.x / norm,
                    y=q.y / norm,
                    z=q.z / norm,
                    w=q.w / norm,
                )

    def _on_operator(self, message):
        self.operator = message
        self.operator_at = rospy.Time.now()

    def _on_status(self, message):
        self.status = message
        self.status_at = rospy.Time.now()

    def _on_candidate(self, message):
        self.candidate = message
        self.candidate_at = rospy.Time.now()

    def _operator_enabled(self, now):
        return bool(
            is_fresh(self.operator_at, self.operator_timeout_s, now)
            and self.operator is not None
            and self.operator.mode == OperatorCommand.MODE_CEILING
            and self.operator.enable_control
        )

    def _ready_for_motor(self, now):
        return bool(
            self.output_enabled
            and self.motor_enabled
            and is_fresh(self.flight_state_at, self.state_timeout_s, now)
            and self.flight_state is not None
            and self.flight_state.connected
            and self.flight_state.armed
            and self.flight_state.mode.upper() == "OFFBOARD"
            and is_fresh(self.odom_at, self.odom_timeout_s, now)
            and self.odom is not None
            and odometry_is_finite(self.odom)
            and self._operator_enabled(now)
            and is_fresh(self.status_at, self.status_timeout_s, now)
            and self.status is not None
            and self.status.candidate_valid
            and self.status.flight_state_fresh
            and self.status.range_fresh
            and self.status.odom_fresh
            and is_fresh(self.candidate_at, self.candidate_timeout_s, now)
            and self.candidate is not None
            and self.candidate.valid
            and self.candidate.source == AttachmentControlCandidate.SOURCE_CEILING
            and self.candidate.kind
            == AttachmentControlCandidate.KIND_ATTITUDE_THRUST
            and self.candidate.header.stamp == self.status.header.stamp
            and math.isfinite(self.candidate.thrust_normalized)
            and self.candidate.thrust_normalized > 0.0
        )

    def _desired_thrust(self, now, wall_now=None):
        if wall_now is None:
            wall_now = time.monotonic()
        if not self._operator_enabled(now):
            self.active_since_wall = None
            if self.flight_state is not None and not self.flight_state.armed:
                self.active_timeout_latched = False
        if not self._ready_for_motor(now) or self.active_timeout_latched:
            return 0.0
        if self.status.state == CeilingAttachmentStatus.STATE_ATTACH_CONTROL:
            thrust = self.attach_thrust
        elif self.status.state == CeilingAttachmentStatus.STATE_SURFACE_HOLD:
            thrust = self.hold_thrust
        else:
            self.active_since_wall = None
            return 0.0
        if self.active_since_wall is None:
            self.active_since_wall = wall_now
        if wall_now - self.active_since_wall >= self.max_active_s:
            self.active_timeout_latched = True
            rospy.logerr("ceiling motor bench active-time limit reached; thrust latched zero")
            return 0.0
        return thrust

    def _on_timer(self, _event):
        now = rospy.Time.now()
        if not self.output_enabled:
            self.mode_publisher.publish(UInt8(data=self.OUTPUT_DISABLED))
            self.thrust_publisher.publish(Float32(data=0.0))
            return

        # Stream zero thrust before OFFBOARD entry and whenever any input gate
        # fails.  No other node in the bench launch publishes MAVROS setpoints.
        thrust = self._desired_thrust(now)
        target = self.builder.attitude_thrust(now, self.last_orientation, thrust)
        if target is None:
            rospy.logerr_throttle(1.0, "ceiling motor bench orientation invalid")
            return
        self.setpoint_publisher.publish(target)
        self.mode_publisher.publish(
            UInt8(
                data=(
                    self.OUTPUT_BENCH_THRUST
                    if thrust > 0.0
                    else self.OUTPUT_ZERO_THRUST
                )
            )
        )
        self.thrust_publisher.publish(Float32(data=thrust))


if __name__ == "__main__":
    rospy.init_node("ceiling_motor_bench_output_node")
    CeilingMotorBenchOutputNode()
    rospy.spin()
