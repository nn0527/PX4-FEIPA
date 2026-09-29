#!/usr/bin/env python3
"""Central safety gate for the legacy generic distance-control chain."""

import math

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

from px4_wall_ceiling_control.freshness import is_fresh, is_valid_range
from px4_wall_ceiling_control.msg import OperatorCommand, SupervisorState


class FlightSupervisorNode:
    def __init__(self):
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 1.0))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.5))
        self.operator_timeout_s = float(rospy.get_param("~operator_timeout_s", 0.5))
        self.range_timeout_s = float(rospy.get_param("~range_timeout_s", 0.3))
        self.require_offboard_for_motion = bool(
            rospy.get_param("~require_offboard_for_motion", True)
        )
        self.require_armed_for_motion = bool(
            rospy.get_param("~require_armed_for_motion", True)
        )
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))

        self.flight_state = None
        self.flight_state_at = None
        self.odom = None
        self.odom_at = None
        self.operator = None
        self.operator_at = None
        self.ceiling_range = None
        self.ceiling_range_at = None
        self.wall_range = None
        self.wall_range_at = None
        self.last_state = None

        state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")
        odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        operator_topic = rospy.get_param("~operator_topic", "/operator/command")
        ceiling_topic = rospy.get_param(
            "~ceiling_range_topic", "/sensor/ceiling/range"
        )
        wall_topic = rospy.get_param("~wall_range_topic", "/sensor/front/range")
        output_topic = rospy.get_param("~output_topic", "/supervisor/state")

        self.publisher = rospy.Publisher(output_topic, SupervisorState, queue_size=10)
        rospy.Subscriber(state_topic, State, self._on_state, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber(
            operator_topic, OperatorCommand, self._on_operator, queue_size=10
        )
        rospy.Subscriber(ceiling_topic, Range, self._on_ceiling_range, queue_size=10)
        rospy.Subscriber(wall_topic, Range, self._on_wall_range, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 1.0)), self._on_timer
        )

    def _on_state(self, message):
        self.flight_state = message
        self.flight_state_at = rospy.Time.now()

    def _on_odom(self, message):
        self.odom = message
        self.odom_at = rospy.Time.now()

    def _on_operator(self, message):
        self.operator = message
        self.operator_at = rospy.Time.now()

    def _on_ceiling_range(self, message):
        self.ceiling_range = message
        self.ceiling_range_at = rospy.Time.now()

    def _on_wall_range(self, message):
        self.wall_range = message
        self.wall_range_at = rospy.Time.now()

    @staticmethod
    def _valid_odom(message):
        if message is None:
            return False
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return all(math.isfinite(value) for value in values)

    def _base_message(self, now):
        result = SupervisorState()
        result.header.stamp = now
        result.state = SupervisorState.STATE_WAIT_LINK
        result.active_mode = SupervisorState.MODE_NONE
        result.setpoint_stream_allowed = False
        result.motion_allowed = False
        result.mavros_connected = bool(
            is_fresh(self.flight_state_at, self.state_timeout_s, now)
            and self.flight_state is not None
            and self.flight_state.connected
        )
        result.armed = bool(result.mavros_connected and self.flight_state.armed)
        result.offboard = bool(
            result.mavros_connected
            and self.flight_state.mode.upper() == "OFFBOARD"
        )
        result.operator_fresh = is_fresh(
            self.operator_at, self.operator_timeout_s, now
        )
        result.odom_fresh = bool(
            is_fresh(self.odom_at, self.odom_timeout_s, now)
            and self._valid_odom(self.odom)
        )
        result.ceiling_range_fresh = bool(
            is_fresh(self.ceiling_range_at, self.range_timeout_s, now)
            and is_valid_range(self.ceiling_range)
        )
        result.wall_range_fresh = bool(
            is_fresh(self.wall_range_at, self.range_timeout_s, now)
            and is_valid_range(self.wall_range)
        )
        result.fault_mask = SupervisorState.FAULT_NONE
        result.reason = "waiting for MAVROS"
        return result

    def _evaluate(self, now):
        result = self._base_message(now)

        if not result.mavros_connected:
            result.fault_mask |= SupervisorState.FAULT_MAVROS
            return result

        if not result.operator_fresh:
            result.state = SupervisorState.STATE_FAULT
            result.fault_mask |= SupervisorState.FAULT_OPERATOR
            result.reason = "operator command timeout"
            return result

        if self.operator.mode == OperatorCommand.MODE_ABORT:
            result.state = SupervisorState.STATE_FAULT
            result.fault_mask |= SupervisorState.FAULT_OPERATOR
            result.reason = "operator abort or RC failure"
            return result
        if self.operator.mode == OperatorCommand.MODE_MANUAL:
            result.state = SupervisorState.STATE_MANUAL
            result.reason = "manual mode requested"
            return result
        if (
            self.operator.mode == OperatorCommand.MODE_STANDBY
            or not self.operator.enable_control
        ):
            result.state = SupervisorState.STATE_STANDBY
            result.reason = "waiting for deadman and tracking selection"
            return result

        if not result.odom_fresh:
            result.state = SupervisorState.STATE_FAULT
            result.fault_mask |= SupervisorState.FAULT_ODOMETRY
            result.reason = "odometry unavailable or invalid"
            return result

        if self.operator.mode == OperatorCommand.MODE_CEILING:
            result.active_mode = SupervisorState.MODE_CEILING
            range_message = self.ceiling_range
            range_received_at = self.ceiling_range_at
            active_state = SupervisorState.STATE_CEILING_ACTIVE
        elif self.operator.mode == OperatorCommand.MODE_WALL:
            result.active_mode = SupervisorState.MODE_WALL
            range_message = self.wall_range
            range_received_at = self.wall_range_at
            active_state = SupervisorState.STATE_WALL_ACTIVE
        else:
            result.state = SupervisorState.STATE_FAULT
            result.fault_mask |= SupervisorState.FAULT_UNKNOWN_MODE
            result.reason = "unknown operator mode"
            return result

        selected_range_valid = bool(
            is_fresh(range_received_at, self.range_timeout_s, now)
            and is_valid_range(range_message)
        )
        if not selected_range_valid:
            result.state = SupervisorState.STATE_FAULT
            if result.active_mode == SupervisorState.MODE_CEILING:
                result.fault_mask |= SupervisorState.FAULT_CEILING_RANGE
            else:
                result.fault_mask |= SupervisorState.FAULT_WALL_RANGE
            result.reason = "selected range unavailable or invalid"
            return result

        # Pre-stream is permitted only after every non-PX4 prerequisite is valid.
        result.setpoint_stream_allowed = True
        if self.require_offboard_for_motion and not result.offboard:
            result.state = SupervisorState.STATE_PRESTREAM
            result.reason = "neutral setpoint pre-stream; waiting for OFFBOARD"
            return result
        if self.require_armed_for_motion and not self.flight_state.armed:
            result.state = SupervisorState.STATE_PRESTREAM
            result.reason = "neutral setpoint stream; waiting for arming"
            return result

        result.state = active_state
        result.motion_allowed = True
        result.reason = "selected tracking mode active"
        return result

    def _on_timer(self, _event):
        result = self._evaluate(rospy.Time.now())
        self.publisher.publish(result)
        if result.state != self.last_state:
            rospy.loginfo(
                "supervisor state=%d active_mode=%d (%s)",
                result.state,
                result.active_mode,
                result.reason,
            )
            self.last_state = result.state


if __name__ == "__main__":
    rospy.init_node("flight_supervisor_node")
    FlightSupervisorNode()
    rospy.spin()
