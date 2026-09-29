#!/usr/bin/env python3
"""Fail-closed output gate for physical attachment mechanism commands."""

import copy

import rospy
from std_msgs.msg import UInt8

from px4_wall_ceiling_control.freshness import is_fresh
from px4_wall_ceiling_control.msg import AttachmentMechanismCommand


class AttachmentMechanismOutputNode:
    OWNER_CEILING = 1

    def __init__(self):
        self.output_enabled = bool(rospy.get_param("~output_enabled", False))
        self.require_ceiling_owner = bool(
            rospy.get_param("~require_ceiling_owner", True)
        )
        self.candidate_timeout_s = float(
            rospy.get_param("~candidate_timeout_s", 0.20)
        )
        self.owner_timeout_s = float(rospy.get_param("~owner_timeout_s", 0.30))

        self.candidate = None
        self.candidate_at = None
        self.owner = 0
        self.owner_at = None

        self.command_publisher = rospy.Publisher(
            rospy.get_param("~command_topic", "/attachment/mechanism_command"),
            AttachmentMechanismCommand,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~candidate_topic", "/attachment/mechanism_candidate"
            ),
            AttachmentMechanismCommand,
            self._on_candidate,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~owner_topic", "/attachment/output_owner"),
            UInt8,
            self._on_owner,
            queue_size=10,
        )
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 2.0)), self._on_timer
        )

        if not self.output_enabled:
            rospy.logwarn("attachment mechanism output is disabled")

    def _on_candidate(self, message):
        self.candidate = message
        self.candidate_at = rospy.Time.now()

    def _on_owner(self, message):
        self.owner = int(message.data)
        self.owner_at = rospy.Time.now()

    def _authorized(self, now):
        if not self.output_enabled:
            return False, "mechanism output disabled"
        if not is_fresh(self.candidate_at, self.candidate_timeout_s, now):
            return False, "mechanism candidate stale"
        if self.candidate is None:
            return False, "mechanism candidate unavailable"
        if self.candidate.source != AttachmentMechanismCommand.SOURCE_CEILING:
            return False, "mechanism candidate source is not ceiling"
        if self.candidate.action not in (
            AttachmentMechanismCommand.ACTION_NONE,
            AttachmentMechanismCommand.ACTION_HOLD,
            AttachmentMechanismCommand.ACTION_RELEASE,
            AttachmentMechanismCommand.ACTION_STOP,
        ):
            return False, "mechanism candidate action is invalid"
        if self.require_ceiling_owner:
            if not is_fresh(self.owner_at, self.owner_timeout_s, now):
                return False, "attachment owner heartbeat stale"
            if self.owner != self.OWNER_CEILING:
                return False, "ceiling does not own attachment output"
        return True, "authorized"

    def _on_timer(self, _event):
        now = rospy.Time.now()
        authorized, reason = self._authorized(now)
        if not authorized:
            if self.output_enabled:
                rospy.logwarn_throttle(1.0, "mechanism output inhibited: %s", reason)
            return
        command = copy.deepcopy(self.candidate)
        command.header.stamp = now
        self.command_publisher.publish(command)


if __name__ == "__main__":
    rospy.init_node("attachment_mechanism_output_node")
    AttachmentMechanismOutputNode()
    rospy.spin()
