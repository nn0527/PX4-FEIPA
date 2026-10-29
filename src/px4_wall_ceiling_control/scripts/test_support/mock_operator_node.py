#!/usr/bin/env python3
"""Bench-only operator intent without RC switches, flight modes or PX4 state."""

import rospy
from std_msgs.msg import String

from px4_wall_ceiling_control.msg import OperatorCommand

MODES = {
    "standby": OperatorCommand.MODE_STANDBY,
    "ceiling": OperatorCommand.MODE_CEILING,
    "wall": OperatorCommand.MODE_WALL,
    "abort": OperatorCommand.MODE_ABORT,
}


class MockOperatorNode:
    def __init__(self):
        command_topic = rospy.get_param("~command_topic", "/operator/command")
        request_topic = str(rospy.get_param("~request_topic", "")).strip()
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.mode = self._parse_mode(rospy.get_param("~mode", "standby"))

        self.publisher = rospy.Publisher(command_topic, OperatorCommand, queue_size=10)
        if request_topic:
            rospy.Subscriber(request_topic, String, self._on_request, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 1.0)), self._on_timer
        )
        rospy.logwarn(
            "mock operator active; bench-only intent on %s, no RC or flight mode checks",
            command_topic,
        )

    @staticmethod
    def _parse_mode(value):
        name = str(value).strip().lower()
        if name not in MODES:
            raise ValueError("unsupported bench operator mode: " + str(value))
        return MODES[name]

    def _on_request(self, message):
        try:
            self.mode = self._parse_mode(message.data)
        except ValueError as error:
            rospy.logwarn(str(error))
            return
        rospy.loginfo("bench operator mode -> %s", message.data.strip().lower())

    def _on_timer(self, _event):
        command = OperatorCommand()
        command.header.stamp = rospy.Time.now()
        command.mode = self.mode
        command.enable_control = self.mode in (
            OperatorCommand.MODE_CEILING,
            OperatorCommand.MODE_WALL,
        )
        command.source = "bench"
        command.reason = "bench operator"
        self.publisher.publish(command)


if __name__ == "__main__":
    rospy.init_node("mock_operator_node")
    MockOperatorNode()
    rospy.spin()
