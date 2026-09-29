#!/usr/bin/env python3
"""Print wall-perch transitions and a one-shot diagnostic snapshot on ABORT."""

import sys
import time
from datetime import datetime
from threading import Lock

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry

from px4_wall_ceiling_control.msg import SensorHealth, WallPerchStatus


class WallPerchStateMonitor:
    def __init__(self):
        self.previous_state = None
        self.previous_name = None
        self.lock = Lock()
        self.latest = {}
        self.bell = bool(rospy.get_param("~bell", True))
        self.status_topic = rospy.get_param("~status_topic", "/wall_perch/status")
        self.health_topic = rospy.get_param("~health_topic", "/sensor/health")
        self.state_topic = rospy.get_param("~state_topic", "/mock_mavros/state")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/mock_mavros/local_position/odom"
        )
        rospy.Subscriber(
            self.health_topic, SensorHealth, self.on_health, queue_size=20
        )
        rospy.Subscriber(
            self.state_topic, State, self.on_flight_state, queue_size=20
        )
        rospy.Subscriber(
            self.odom_topic, Odometry, self.on_odom, queue_size=20
        )
        rospy.Subscriber(
            self.status_topic,
            WallPerchStatus,
            self.on_status,
            queue_size=20,
        )

    def _remember(self, name, message):
        with self.lock:
            self.latest[name] = (message, time.monotonic())

    def on_health(self, message):
        self._remember("health", message)

    def on_flight_state(self, message):
        self._remember("state", message)

    def on_odom(self, message):
        self._remember("odom", message)

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
            latest = dict(self.latest) if status.state == WallPerchStatus.STATE_ABORT else None

        transition = "{} -> {}".format(previous_name, status.state_name)
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = (
            "[侧吸状态 {}] {} | 原因: {} | 前向: {:.3f} m | 顶部: {:.3f} m"
            " | 故障次数: {}"
        ).format(
            timestamp,
            transition,
            status.reason or "无",
            status.front_distance_filtered_m,
            status.top_distance_filtered_m,
            status.fault_count,
        )
        if status.state == WallPerchStatus.STATE_ABORT:
            line = "[警告] " + line

        output = "\a" if ring and sys.stdout.isatty() else ""
        output += line + "\n"
        if latest is not None:
            now = time.monotonic()
            output += "[ABORT 快照] 最近接收的消息；接收时间差不代表传感器样本年龄\n"
            output += self._format_message(self.status_topic, (status, now), now) + "\n"
            for name, topic in (
                ("health", self.health_topic),
                ("state", self.state_topic),
                ("odom", self.odom_topic),
            ):
                output += self._format_message(topic, latest.get(name), now) + "\n"
            output += "[ABORT 快照结束]\n"
        sys.stdout.write(output)
        sys.stdout.flush()


if __name__ == "__main__":
    rospy.init_node("wall_perch_state_monitor")
    WallPerchStateMonitor()
    rospy.spin()
