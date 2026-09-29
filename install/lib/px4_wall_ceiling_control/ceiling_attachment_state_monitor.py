#!/usr/bin/env python3
"""Print ceiling-attachment transitions and a one-shot FAULT snapshot."""

import sys
import time
from datetime import datetime
from threading import Lock

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

from px4_wall_ceiling_control.msg import (
    AttachmentMechanismStatus,
    CeilingAttachmentStatus,
    SensorHealth,
)


class CeilingAttachmentStateMonitor:
    def __init__(self):
        self.previous_state = None
        self.previous_name = None
        self.lock = Lock()
        self.latest = {}
        self.bell = bool(rospy.get_param("~bell", True))
        self.status_topic = rospy.get_param(
            "~status_topic", "/ceiling_attachment/status"
        )
        self.range_topic = rospy.get_param("~range_topic", "/sensor/ceiling/range")
        self.health_topic = rospy.get_param("~health_topic", "/sensor/health")
        self.state_topic = rospy.get_param("~state_topic", "/mock_mavros/state")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/mock_mavros/local_position/odom"
        )
        self.mechanism_topic = rospy.get_param(
            "~mechanism_topic", "/attachment/mechanism_status"
        )

        rospy.Subscriber(self.range_topic, Range, self.on_range, queue_size=20)
        rospy.Subscriber(
            self.health_topic, SensorHealth, self.on_health, queue_size=20
        )
        rospy.Subscriber(
            self.state_topic, State, self.on_flight_state, queue_size=20
        )
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber(
            self.mechanism_topic,
            AttachmentMechanismStatus,
            self.on_mechanism,
            queue_size=20,
        )
        rospy.Subscriber(
            self.status_topic,
            CeilingAttachmentStatus,
            self.on_status,
            queue_size=20,
        )

    def _remember(self, name, message):
        with self.lock:
            self.latest[name] = (message, time.monotonic())

    def on_range(self, message):
        self._remember("range", message)

    def on_health(self, message):
        self._remember("health", message)

    def on_flight_state(self, message):
        self._remember("state", message)

    def on_odom(self, message):
        self._remember("odom", message)

    def on_mechanism(self, message):
        self._remember("mechanism", message)

    @staticmethod
    def _format_message(topic, sample, now):
        if sample is None:
            return "{}: 尚未收到消息".format(topic)
        message, received_at = sample
        age_s = max(0.0, now - received_at)
        return "{} (监控器接收于 {:.3f} 秒前):\n{}".format(
            topic, age_s, message
        )

    def on_status(self, status):
        with self.lock:
            if status.state == self.previous_state:
                return
            previous_name = (
                "启动监控" if self.previous_state is None else self.previous_name
            )
            ring = self.previous_state is not None and self.bell
            self.previous_state = status.state
            self.previous_name = status.state_name
            latest = (
                dict(self.latest)
                if status.state == CeilingAttachmentStatus.STATE_FAULT
                else None
            )

        transition = "{} -> {}".format(previous_name, status.state_name)
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = (
            "[吸顶状态 {}] {} | 原因: {} | 顶部: {:.3f} m | 压缩量: {:.3f} m"
            " | 故障次数: {}"
        ).format(
            timestamp,
            transition,
            status.reason or "无",
            status.ceiling_distance_filtered_m,
            status.compression_m,
            status.fault_count,
        )
        if status.state == CeilingAttachmentStatus.STATE_FAULT:
            line = "[警告] " + line

        output = "\a" if ring and sys.stdout.isatty() else ""
        output += line + "\n"
        if latest is not None:
            now = time.monotonic()
            output += "[FAULT 快照] 最近接收的消息；接收时间差不代表传感器样本年龄\n"
            output += self._format_message(self.status_topic, (status, now), now) + "\n"
            for name, topic in (
                ("range", self.range_topic),
                ("health", self.health_topic),
                ("state", self.state_topic),
                ("odom", self.odom_topic),
                ("mechanism", self.mechanism_topic),
            ):
                output += self._format_message(topic, latest.get(name), now) + "\n"
            output += "[FAULT 快照结束]\n"
        sys.stdout.write(output)
        sys.stdout.flush()


if __name__ == "__main__":
    rospy.init_node("ceiling_attachment_state_monitor")
    CeilingAttachmentStateMonitor()
    rospy.spin()
