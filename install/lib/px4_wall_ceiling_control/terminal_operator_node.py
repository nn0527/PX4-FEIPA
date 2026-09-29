#!/usr/bin/env python3
"""One-shot terminal commands for a persistent attachment operator intent."""

import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String

from px4_wall_ceiling_control.msg import OperatorCommand


class TerminalOperatorNode:
    def __init__(self):
        self.mode = OperatorCommand.MODE_STANDBY
        self.detaching = False
        self.detach_pulse = False
        self.was_offboard = False
        self.flight_mode = ""
        self.command_pub = rospy.Publisher(
            "/operator/command", OperatorCommand, queue_size=10
        )
        self.ceiling_detach_pub = rospy.Publisher(
            "/ceiling_attachment/detach", Bool, queue_size=10
        )
        self.wall_detach_pub = rospy.Publisher(
            "/wall_perch/detach", Bool, queue_size=10
        )
        rospy.Subscriber(
            "/attachment/terminal_command", String, self._on_command, queue_size=10
        )
        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)

    def _on_command(self, message):
        command = message.data.strip().lower()
        if command in ("ceiling", "wall"):
            if self.mode != OperatorCommand.MODE_STANDBY or self.detaching:
                rospy.logwarn("terminal operator: finish current cycle before %s", command)
                return
            if self.flight_mode != "ALTCTL":
                rospy.logwarn("terminal operator: start only from PX4 ALTCTL")
                return
            self.mode = (
                OperatorCommand.MODE_CEILING
                if command == "ceiling"
                else OperatorCommand.MODE_WALL
            )
            self.was_offboard = False
            rospy.loginfo("terminal operator: %s requested", command)
        elif command == "detach":
            if self.mode not in (
                OperatorCommand.MODE_CEILING,
                OperatorCommand.MODE_WALL,
            ):
                rospy.logwarn("terminal operator: no attachment to detach")
                return
            self.detaching = True
            self.detach_pulse = True
            rospy.loginfo("terminal operator: detach requested")
        elif command == "cancel":
            # Release the behavior request. Decision nodes then execute their
            # recovery path; this does not force a premature mode change.
            self.mode = OperatorCommand.MODE_STANDBY
            self.detaching = False
            self.detach_pulse = False
            rospy.logwarn("terminal operator: behavior cancel requested")
        else:
            rospy.logwarn("terminal operator: use ceiling, wall, detach or cancel")

    def _on_state(self, message):
        self.flight_mode = message.mode.upper()
        if self.flight_mode == "OFFBOARD":
            self.was_offboard = True
        elif self.was_offboard and self.flight_mode == "ALTCTL":
            self.mode = OperatorCommand.MODE_STANDBY
            self.detaching = False
            self.detach_pulse = False
            self.was_offboard = False

    def _on_timer(self, _event):
        command = OperatorCommand()
        command.header.stamp = rospy.Time.now()
        command.mode = self.mode
        command.enable_control = self.mode in (
            OperatorCommand.MODE_CEILING,
            OperatorCommand.MODE_WALL,
        )
        command.source = "terminal"
        command.reason = "detach pending" if self.detaching else "terminal request"
        self.command_pub.publish(command)
        if self.detach_pulse:
            if self.mode == OperatorCommand.MODE_CEILING:
                self.ceiling_detach_pub.publish(Bool(data=True))
            elif self.mode == OperatorCommand.MODE_WALL:
                self.wall_detach_pub.publish(Bool(data=True))
            self.detach_pulse = False


if __name__ == "__main__":
    rospy.init_node("terminal_operator_node")
    TerminalOperatorNode()
    rospy.spin()
