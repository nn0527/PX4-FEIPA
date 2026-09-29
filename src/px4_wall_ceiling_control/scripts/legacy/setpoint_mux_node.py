#!/usr/bin/env python3
"""MAVROS setpoint mux for the legacy generic distance-control chain."""

import copy
import math

import rospy
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from std_msgs.msg import UInt8

from px4_wall_ceiling_control.freshness import clamp, is_fresh
from px4_wall_ceiling_control.msg import ControlSetpoint, SupervisorState


class SetpointMuxNode:
    VELOCITY_ONLY_MASK = (
        PositionTarget.IGNORE_PX
        | PositionTarget.IGNORE_PY
        | PositionTarget.IGNORE_PZ
        | PositionTarget.IGNORE_AFX
        | PositionTarget.IGNORE_AFY
        | PositionTarget.IGNORE_AFZ
        | PositionTarget.IGNORE_YAW
        | PositionTarget.IGNORE_YAW_RATE
    )

    def __init__(self):
        self.output_enabled = bool(rospy.get_param("~output_enabled", False))
        self.ceiling_output_enabled = bool(
            rospy.get_param("~ceiling_output_enabled", False)
        )
        self.wall_output_enabled = bool(
            rospy.get_param("~wall_output_enabled", False)
        )
        self.allow_attitude = bool(
            rospy.get_param("~allow_attitude_setpoints", False)
        )
        self.enforce_velocity_only = bool(
            rospy.get_param("~enforce_velocity_only", True)
        )
        self.supervisor_timeout_s = float(
            rospy.get_param("~supervisor_timeout_s", 0.3)
        )
        self.candidate_timeout_s = float(
            rospy.get_param("~candidate_timeout_s", 0.2)
        )
        self.max_velocity_xy_mps = float(
            rospy.get_param("~max_velocity_xy_mps", 0.6)
        )
        self.max_velocity_z_mps = float(
            rospy.get_param("~max_velocity_z_mps", 0.4)
        )
        self.min_thrust = float(rospy.get_param("~min_thrust", 0.1))
        self.max_thrust = float(rospy.get_param("~max_thrust", 0.8))
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))

        self.supervisor = None
        self.supervisor_at = None
        self.ceiling_candidate = None
        self.ceiling_at = None
        self.wall_candidate = None
        self.wall_at = None

        self.local_publisher = rospy.Publisher(
            rospy.get_param(
                "~local_setpoint_topic", "/mavros/setpoint_raw/local"
            ),
            PositionTarget,
            queue_size=10,
        )
        self.attitude_publisher = rospy.Publisher(
            rospy.get_param(
                "~attitude_setpoint_topic", "/mavros/setpoint_raw/attitude"
            ),
            AttitudeTarget,
            queue_size=10,
        )
        self.selected_source_publisher = rospy.Publisher(
            "/control/selected_source", UInt8, queue_size=10
        )

        rospy.Subscriber(
            rospy.get_param("~supervisor_topic", "/supervisor/state"),
            SupervisorState,
            self._on_supervisor,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~ceiling_candidate_topic", "/control/ceiling_candidate"
            ),
            ControlSetpoint,
            self._on_ceiling,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~wall_candidate_topic", "/control/wall_candidate"),
            ControlSetpoint,
            self._on_wall,
            queue_size=10,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 2.0)), self._on_timer
        )

        if not self.output_enabled:
            rospy.logwarn("setpoint output is disabled by the master safety switch")

    def _on_supervisor(self, message):
        self.supervisor = message
        self.supervisor_at = rospy.Time.now()

    def _on_ceiling(self, message):
        self.ceiling_candidate = message
        self.ceiling_at = rospy.Time.now()

    def _on_wall(self, message):
        self.wall_candidate = message
        self.wall_at = rospy.Time.now()

    def _neutral_velocity_target(self, now):
        target = PositionTarget()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = self.VELOCITY_ONLY_MASK
        return target

    def _selected_candidate(self):
        if self.supervisor.active_mode == SupervisorState.MODE_CEILING:
            return (
                self.ceiling_candidate,
                self.ceiling_at,
                ControlSetpoint.SOURCE_CEILING,
            )
        if self.supervisor.active_mode == SupervisorState.MODE_WALL:
            return self.wall_candidate, self.wall_at, ControlSetpoint.SOURCE_WALL
        return None, None, ControlSetpoint.SOURCE_NONE

    def _selected_mode_enabled(self):
        if self.supervisor.active_mode == SupervisorState.MODE_CEILING:
            return self.ceiling_output_enabled
        if self.supervisor.active_mode == SupervisorState.MODE_WALL:
            return self.wall_output_enabled
        return False

    def _sanitize_local(self, target, now):
        result = copy.deepcopy(target)
        values = (result.velocity.x, result.velocity.y, result.velocity.z)
        if not all(math.isfinite(value) for value in values):
            return None

        xy_norm = math.hypot(result.velocity.x, result.velocity.y)
        if xy_norm > self.max_velocity_xy_mps > 0.0:
            scale = self.max_velocity_xy_mps / xy_norm
            result.velocity.x *= scale
            result.velocity.y *= scale
        result.velocity.z = clamp(
            result.velocity.z, -self.max_velocity_z_mps, self.max_velocity_z_mps
        )
        if self.enforce_velocity_only:
            result.type_mask = self.VELOCITY_ONLY_MASK
            result.yaw = 0.0
            result.yaw_rate = 0.0
        result.header.stamp = now
        return result

    def _sanitize_attitude(self, target, now):
        result = copy.deepcopy(target)
        quaternion = result.orientation
        values = (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
            result.thrust,
        )
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm < 1.0e-6:
            return None
        quaternion.x /= norm
        quaternion.y /= norm
        quaternion.z /= norm
        quaternion.w /= norm
        result.thrust = clamp(result.thrust, self.min_thrust, self.max_thrust)
        result.header.stamp = now
        return result

    def _on_timer(self, _event):
        now = rospy.Time.now()
        self.selected_source_publisher.publish(UInt8(data=ControlSetpoint.SOURCE_NONE))
        if not self.output_enabled:
            return
        if not is_fresh(self.supervisor_at, self.supervisor_timeout_s, now):
            rospy.logerr_throttle(1.0, "setpoint inhibited: supervisor timeout")
            return
        if not self.supervisor.setpoint_stream_allowed:
            return
        if not self._selected_mode_enabled():
            rospy.logwarn_throttle(1.0, "selected mode output switch is disabled")
            return

        if not self.supervisor.motion_allowed:
            self.local_publisher.publish(self._neutral_velocity_target(now))
            return

        candidate, received_at, expected_source = self._selected_candidate()
        if (
            candidate is None
            or not is_fresh(received_at, self.candidate_timeout_s, now)
            or not candidate.valid
            or candidate.source != expected_source
        ):
            rospy.logerr_throttle(
                1.0, "active candidate invalid/stale; sending neutral velocity"
            )
            self.local_publisher.publish(self._neutral_velocity_target(now))
            return

        if candidate.kind == ControlSetpoint.KIND_LOCAL:
            target = self._sanitize_local(candidate.local, now)
            if target is None:
                rospy.logerr_throttle(1.0, "rejected non-finite local setpoint")
                self.local_publisher.publish(self._neutral_velocity_target(now))
                return
            self.local_publisher.publish(target)
        elif candidate.kind == ControlSetpoint.KIND_ATTITUDE and self.allow_attitude:
            target = self._sanitize_attitude(candidate.attitude, now)
            if target is None:
                rospy.logerr_throttle(1.0, "rejected invalid attitude setpoint")
                self.local_publisher.publish(self._neutral_velocity_target(now))
                return
            self.attitude_publisher.publish(target)
        else:
            rospy.logerr_throttle(1.0, "rejected unsupported setpoint kind")
            self.local_publisher.publish(self._neutral_velocity_target(now))
            return

        self.selected_source_publisher.publish(UInt8(data=expected_source))


if __name__ == "__main__":
    rospy.init_node("setpoint_mux_node")
    SetpointMuxNode()
    rospy.spin()
