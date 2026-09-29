#!/usr/bin/env python3
"""Validate and independently distribute the ceiling and front ranges."""

import math

import rospy
from sensor_msgs.msg import Range

from px4_wall_ceiling_control.freshness import is_fresh, is_valid_range
from px4_wall_ceiling_control.msg import SensorHealth


class SensorRangeManagerNode:
    """One input manager with independent, event-driven data outputs.

    The manager deliberately has no knowledge of operator mode or attachment
    output ownership.  Decisions may observe the sensors they need, while the
    downstream output mux remains the only control authority arbiter.
    """

    def __init__(self):
        self.input_timeout_s = float(rospy.get_param("~input_timeout_s", 0.30))
        health_rate_hz = float(rospy.get_param("~health_publish_rate_hz", 10.0))

        self.ceiling_received_at = None
        self.front_received_at = None
        self.ceiling_latest_valid = False
        self.front_latest_valid = False
        self.ceiling_sequence = 0
        self.front_sequence = 0
        self.ceiling_invalid_count = 0
        self.front_invalid_count = 0

        self.ceiling_publisher = rospy.Publisher(
            rospy.get_param("~ceiling_output_topic", "/sensor/ceiling/range"),
            Range,
            queue_size=10,
        )
        self.front_publisher = rospy.Publisher(
            rospy.get_param("~front_output_topic", "/sensor/front/range"),
            Range,
            queue_size=10,
        )
        self.health_publisher = rospy.Publisher(
            rospy.get_param("~health_topic", "/sensor/health"),
            SensorHealth,
            queue_size=10,
            latch=True,
        )

        rospy.Subscriber(
            rospy.get_param("~ceiling_input_topic", "/sensor/ceiling/raw"),
            Range,
            self._on_ceiling,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~front_input_topic", "/sensor/front/raw"),
            Range,
            self._on_front,
            queue_size=10,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(health_rate_hz, 1.0)), self._on_timer
        )

    def _on_ceiling(self, message):
        now = rospy.Time.now()
        self.ceiling_latest_valid = is_valid_range(message)
        if not self.ceiling_latest_valid:
            self.ceiling_invalid_count += 1
            rospy.logwarn_throttle(1.0, "invalid ceiling range input")
            return
        self.ceiling_received_at = now
        self.ceiling_sequence += 1
        self.ceiling_publisher.publish(message)

    def _on_front(self, message):
        now = rospy.Time.now()
        self.front_latest_valid = is_valid_range(message)
        if not self.front_latest_valid:
            self.front_invalid_count += 1
            rospy.logwarn_throttle(1.0, "invalid front range input")
            return
        self.front_received_at = now
        self.front_sequence += 1
        self.front_publisher.publish(message)

    def _channel_valid(self, received_at, latest_valid, now):
        return bool(
            latest_valid and is_fresh(received_at, self.input_timeout_s, now)
        )

    @staticmethod
    def _age(received_at, now):
        if received_at is None:
            return math.inf
        return max(0.0, (now - received_at).to_sec())

    def _build_health(self, now):
        status = SensorHealth()
        status.header.stamp = now
        status.ceiling_valid = self._channel_valid(
            self.ceiling_received_at, self.ceiling_latest_valid, now
        )
        status.front_valid = self._channel_valid(
            self.front_received_at, self.front_latest_valid, now
        )
        status.ceiling_age_s = self._age(self.ceiling_received_at, now)
        status.front_age_s = self._age(self.front_received_at, now)
        status.ceiling_sequence = self.ceiling_sequence
        status.front_sequence = self.front_sequence
        status.ceiling_invalid_count = self.ceiling_invalid_count
        status.front_invalid_count = self.front_invalid_count
        return status

    def _on_timer(self, _event):
        self.health_publisher.publish(self._build_health(rospy.Time.now()))


if __name__ == "__main__":
    rospy.init_node("sensor_range_manager_node")
    SensorRangeManagerNode()
    rospy.spin()
