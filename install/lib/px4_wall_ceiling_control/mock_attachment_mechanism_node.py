#!/usr/bin/env python3
"""Bench-only attachment mechanism acknowledgement simulator."""

import rospy

from px4_wall_ceiling_control.msg import (
    AttachmentMechanismCommand,
    AttachmentMechanismStatus,
)


class MockAttachmentMechanismNode:
    def __init__(self):
        self.release_delay_s = float(rospy.get_param("~release_delay_s", 0.20))
        self.command = None
        self.command_received_at = None
        self.publisher = rospy.Publisher(
            rospy.get_param("~status_topic", "/attachment/mechanism_status"),
            AttachmentMechanismStatus,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~command_topic", "/attachment/mechanism_command"),
            AttachmentMechanismCommand,
            self._on_command,
            queue_size=10,
        )
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 2.0)), self._on_timer
        )
        rospy.logwarn(
            "mock attachment mechanism active; no physical actuator is controlled"
        )

    def _on_command(self, message):
        if (
            self.command is None
            or message.sequence != self.command.sequence
            or message.action != self.command.action
        ):
            self.command_received_at = rospy.Time.now()
        self.command = message

    def _on_timer(self, _event):
        if self.command is None or self.command_received_at is None:
            return
        now = rospy.Time.now()
        status = AttachmentMechanismStatus()
        status.header.stamp = now
        status.source = self.command.source
        status.sequence = self.command.sequence
        status.acknowledged = True
        status.released = bool(
            self.command.action == AttachmentMechanismCommand.ACTION_RELEASE
            and (now - self.command_received_at).to_sec() >= self.release_delay_s
        )
        status.fault = False
        status.reason = "mock release confirmed" if status.released else "mock acknowledged"
        self.publisher.publish(status)


if __name__ == "__main__":
    rospy.init_node("mock_attachment_mechanism_node")
    MockAttachmentMechanismNode()
    rospy.spin()
