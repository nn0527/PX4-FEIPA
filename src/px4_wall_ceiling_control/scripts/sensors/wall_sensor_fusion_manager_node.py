#!/usr/bin/env python3
"""Manage optical-flow/range EKF parameters without blocking flight outputs."""
import fcntl
import hashlib
import json
import queue
import threading
import time

import rospy
from mavros_msgs.msg import State
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from px4_wall_ceiling_control.domain.wall_sensor_fusion import (
    FusionPolicy,
)
from px4_wall_ceiling_control.msg import WallPerchStatus, WallSensorFusionStatus


from px4_wall_ceiling_control.adapters.fusion_mavlink import MavrosParameterTransport
from px4_wall_ceiling_control.domain.fusion_worker import MemoryRestoreJournal, SerialParameterWorker


class WallSensorFusionManagerNode:
    def __init__(self):
        self.lock = threading.RLock()
        namespace = str(rospy.get_param("~mavros_namespace", "/mavros")).rstrip("/")
        vehicle_id = str(rospy.get_param("~vehicle_id", "")).strip()
        if not vehicle_id:
            target = rospy.get_param(namespace + "/target_system_id", None)
            fcu_url = str(rospy.get_param(namespace + "/fcu_url", "")).strip()
            if target is None or not fcu_url:
                raise ValueError("set ~vehicle_id or MAVROS target_system_id and fcu_url; vehicle identity is required")
            vehicle_id = "{}|sys={}|fcu={}".format(namespace, int(target), fcu_url)
        identity_hash = hashlib.sha256(vehicle_id.encode("utf-8")).hexdigest()[:24]
        # Prevent concurrent managers, without durable records or disk sync.
        # /tmp is volatile on the Pi; this lock does not store recovery state.
        self.process_lock = open("/tmp/wall_sensor_fusion_" + identity_hash + ".lock", "a")
        fcntl.flock(self.process_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        settings = {key: float(rospy.get_param("~" + key, default)) for key, default in (
            ("restore_tilt_deg", 80.), ("application_settle_s", 1.2),
            ("odom_timeout_s", .3), ("status_timeout_s", .6), ("state_timeout_s", 1.5),
            ("future_tolerance_s", .1), ("request_timeout_s", 5.),
            ("retry_initial_s", 1.), ("retry_max_s", 8.), ("restart_quarantine_s", 10.))}
        self.journal = MemoryRestoreJournal()
        self.policy = FusionPolicy(self.journal, vehicle_id,
                                  enabled=bool(rospy.get_param("~enabled", False)), **settings)
        self.publisher = rospy.Publisher(
            rospy.get_param("~status_topic", "/wall_perch/sensor_fusion_status"),
            WallSensorFusionStatus, queue_size=5, latch=True)
        self.transport = MavrosParameterTransport(namespace)
        self.worker = SerialParameterWorker(self.transport.execute,
                                            lambda: rospy.Time.now().to_sec(),
                                            authorize=self._authorize)
        self.diagnostics_publisher = rospy.Publisher(
            "/wall_perch/sensor_fusion_operation", String, queue_size=10)
        self.state_names = {getattr(WallPerchStatus, "STATE_" + name): name for name in (
            "IDLE", "FRONT_WALL_DETECT", "STABILIZE_HOVER", "SLOW_APPROACH",
            "FLIP_TO_WALL", "WALL_CAPTURE", "WALL_HOLD", "WALL_PIN",
            "DETACH_ROTATE", "RECOVER", "EXIT", "ABORT")}
        self.odom_subscriber = rospy.Subscriber(
            rospy.get_param("~odom_topic", namespace + "/local_position/odom"),
            Odometry, self._on_odom, queue_size=1, tcp_nodelay=True)
        self.wall_subscriber = rospy.Subscriber(
            rospy.get_param("~wall_status_topic", "/wall_perch/status"),
            WallPerchStatus, self._on_wall, queue_size=1, tcp_nodelay=True)
        self.state_subscriber = rospy.Subscriber(
            rospy.get_param("~state_topic", namespace + "/state"),
            State, self._on_state, queue_size=1, tcp_nodelay=True)
        self.kill_subscriber = rospy.Subscriber(
            "/attachment/kill_switch", String, self._on_kill, queue_size=10, tcp_nodelay=True)
        self.timer = rospy.Timer(rospy.Duration(.1), self._on_timer)
        rospy.on_shutdown(self.worker.close)
        rospy.loginfo("wall EKF parameter manager enabled=%s vehicle=%s recovery=memory_only; status is not actual fusion evidence",
                      self.policy.enabled, vehicle_id)

    @staticmethod
    def _times(message):
        return message.header.stamp.to_sec(), rospy.Time.now().to_sec(), time.monotonic()

    def _on_odom(self, message):
        q = message.pose.pose.orientation
        with self.lock:
            self.policy.observe_odom((q.x, q.y, q.z, q.w), *self._times(message))

    def _on_wall(self, message):
        with self.lock:
            # Numeric state is authoritative; a malformed textual label must not
            # create a false HOLD/recovery edge.
            self.policy.observe_wall(self.state_names.get(message.state, "UNKNOWN"),
                                     *self._times(message))

    def _on_state(self, message):
        with self.lock:
            self.policy.observe_fcu(message.connected, message.armed, *self._times(message))

    def _on_kill(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if (not isinstance(status, dict) or status.get("valid") is not True
                or type(status.get("active")) is not bool):
            return
        with self.lock:
            self.policy.observe_kill(status["active"], time.monotonic())

    def _authorize(self, operation):
        with self.lock:
            return self.policy.authorize_operation(
                operation, rospy.Time.now().to_sec(), time.monotonic())

    def _on_timer(self, _event):
        now, monotonic = rospy.Time.now().to_sec(), time.monotonic()
        with self.lock:
            while True:
                try:
                    progress = self.worker.progress.get_nowait()
                except queue.Empty:
                    break
                self.policy.mark_operation_stage(*progress)
            while True:
                try:
                    result = self.worker.results.get_nowait()
                except queue.Empty:
                    break
                self.policy.complete(*result)
            operation = self.policy.tick(now, monotonic)
            if operation is not None:
                self.policy.mark_operation_stage(operation, 'queued', now, monotonic)
                self.worker.submit(operation)
            data = self.policy.status(now, monotonic)
        message = WallSensorFusionStatus()
        message.header.stamp = rospy.Time.from_sec(now)
        for key, value in data.items():
            setattr(message, key, rospy.Time.from_sec(value)
                    if key in ("request_stamp", "confirmed_stamp", "ready_after") else value)
        self.publisher.publish(message)
        while True:
            try:
                diagnostic = self.worker.diagnostics.get_nowait()
            except queue.Empty:
                break
            payload = json.dumps(diagnostic, sort_keys=True)
            self.diagnostics_publisher.publish(String(data=payload))
            rospy.loginfo('wall parameter operation: %s', payload)
        if data["phase"] == "ERROR":
            rospy.logwarn_throttle(2., "wall EKF parameter management: %s", data["reason"])


def main():
    rospy.init_node("wall_sensor_fusion_manager_node")
    try:
        WallSensorFusionManagerNode()
    except (ValueError, OSError) as exc:
        rospy.logfatal("wall EKF parameter manager startup failed: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
