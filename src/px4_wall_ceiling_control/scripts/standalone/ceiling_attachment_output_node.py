#!/usr/bin/env python3
"""Translate a standalone ceiling decision into MAVROS raw setpoints.

Arbitration is intentionally upstream: this node accepts only the selected
ceiling decision stream and is the sole downstream command adapter. Combined
ceiling/wall operation uses attachment_output_mux_node instead.
"""

import math

import rospy
from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8

from px4_wall_ceiling_control.adapters.mavros_setpoint import MavrosSetpointBuilder
from px4_wall_ceiling_control.adapters.odometry import odometry_is_finite
from px4_wall_ceiling_control.domain.safety import flight_state_ready
from px4_wall_ceiling_control.freshness import is_fresh
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus


class CeilingAttachmentOutputNode:
    OUTPUT_NONE = 0
    OUTPUT_LOCAL_VELOCITY = 1
    OUTPUT_ATTITUDE_THRUST = 2

    LOCAL_Z_VELOCITY_MASK = MavrosSetpointBuilder.LOCAL_VELOCITY_MASK
    ATTITUDE_AND_THRUST_MASK = MavrosSetpointBuilder.ATTITUDE_AND_THRUST_MASK

    def __init__(self):
        self.output_enabled = bool(rospy.get_param("~output_enabled", False))
        self.require_connected = bool(rospy.get_param("~require_connected", True))
        self.require_armed = bool(rospy.get_param("~require_armed", True))
        self.require_offboard = bool(rospy.get_param("~require_offboard", True))
        self.publish_neutral_when_selected = bool(
            rospy.get_param("~publish_neutral_when_selected", True)
        )
        self.status_timeout_s = float(rospy.get_param("~status_timeout_s", 0.20))
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 0.50))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.max_approach_speed_mps = abs(
            float(rospy.get_param("~max_approach_speed_mps", 0.30))
        )
        self.min_thrust = float(rospy.get_param("~min_thrust", 0.10))
        self.max_thrust = float(rospy.get_param("~max_thrust", 0.90))
        self.local_frame_id = rospy.get_param("~local_frame_id", "map")
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))

        self.status = None
        self.status_received_at = None
        self.flight_state = None
        self.flight_state_received_at = None
        self.odom = None
        self.odom_received_at = None
        self.setpoint_builder = MavrosSetpointBuilder(
            self.max_approach_speed_mps,
            self.min_thrust,
            self.max_thrust,
            self.local_frame_id,
        )

        status_topic = rospy.get_param(
            "~status_topic", "/ceiling_attachment/status"
        )
        state_topic = rospy.get_param("~state_topic", "/mavros/state")
        odom_topic = rospy.get_param(
            "~odom_topic", "/mavros/local_position/odom"
        )
        local_topic = rospy.get_param(
            "~local_setpoint_topic", "/mavros/setpoint_raw/local"
        )
        attitude_topic = rospy.get_param(
            "~attitude_setpoint_topic", "/mavros/setpoint_raw/attitude"
        )
        output_mode_topic = rospy.get_param(
            "~output_mode_topic", "/ceiling_attachment/output_mode"
        )

        self.local_publisher = rospy.Publisher(
            local_topic, PositionTarget, queue_size=10
        )
        self.attitude_publisher = rospy.Publisher(
            attitude_topic, AttitudeTarget, queue_size=10
        )
        self.output_mode_publisher = rospy.Publisher(
            output_mode_topic, UInt8, queue_size=10, latch=True
        )
        rospy.Subscriber(
            status_topic, CeilingAttachmentStatus, self._on_status, queue_size=10
        )
        rospy.Subscriber(state_topic, State, self._on_state, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 2.0)), self._on_timer
        )

        if not self.output_enabled:
            rospy.logwarn("ceiling attachment MAVROS output is disabled")

    def _on_status(self, message):
        self.status = message
        self.status_received_at = rospy.Time.now()

    def _on_state(self, message):
        self.flight_state = message
        self.flight_state_received_at = rospy.Time.now()

    def _on_odom(self, message):
        self.odom = message
        self.odom_received_at = rospy.Time.now()

    def _flight_ready(self, now):
        return flight_state_ready(
            self.flight_state,
            is_fresh(self.flight_state_received_at, self.state_timeout_s, now),
            self.require_connected,
            self.require_armed,
            self.require_offboard,
        )

    def _odom_fresh(self, now):
        return bool(
            self.odom is not None
            and is_fresh(self.odom_received_at, self.odom_timeout_s, now)
            and odometry_is_finite(self.odom)
        )

    def _local_velocity_target(self, now, velocity_z):
        return self.setpoint_builder.local_velocity(now, velocity_z)

    def _attitude_thrust_target(self, now, orientation, thrust_body_z):
        # Reference PX4 status uses negative body-z; MAVROS AttitudeTarget uses
        # a positive normalized thrust scalar.
        return self.setpoint_builder.attitude_thrust(
            now, orientation, -thrust_body_z
        )

    def _publish_mode(self, mode):
        self.output_mode_publisher.publish(UInt8(data=mode))

    def _on_timer(self, _event):
        now = rospy.Time.now()
        if not self.output_enabled:
            self._publish_mode(self.OUTPUT_NONE)
            return
        if not is_fresh(self.status_received_at, self.status_timeout_s, now):
            rospy.logerr_throttle(1.0, "ceiling output inhibited: decision timeout")
            self._publish_mode(self.OUTPUT_NONE)
            return
        if self.status is None or not self.status.enabled:
            self._publish_mode(self.OUTPUT_NONE)
            return
        if not self.status.candidate_valid:
            rospy.logerr_throttle(1.0, "ceiling output inhibited: invalid candidate")
            self._publish_mode(self.OUTPUT_NONE)
            return

        ready, reason = self._flight_ready(now)
        if not ready:
            rospy.logwarn_throttle(1.0, "ceiling output inhibited: %s", reason)
            self._publish_mode(self.OUTPUT_NONE)
            return
        if not self._odom_fresh(now):
            rospy.logwarn_throttle(1.0, "ceiling output inhibited: odometry stale")
            self._publish_mode(self.OUTPUT_NONE)
            return

        if self.status.state == CeilingAttachmentStatus.STATE_APPROACH:
            velocity_z = self.status.approach_velocity_z_enu_mps
            if not math.isfinite(velocity_z):
                rospy.logerr_throttle(1.0, "invalid approach velocity candidate")
                self._publish_mode(self.OUTPUT_NONE)
                return
            self.local_publisher.publish(
                self._local_velocity_target(now, velocity_z)
            )
            self._publish_mode(self.OUTPUT_LOCAL_VELOCITY)
            return

        if self.status.state in (
            CeilingAttachmentStatus.STATE_ATTACH_CONTROL,
            CeilingAttachmentStatus.STATE_SURFACE_HOLD,
            CeilingAttachmentStatus.STATE_DETACH,
            CeilingAttachmentStatus.STATE_FAULT,
        ):
            if not self.status.attitude_setpoint_valid:
                rospy.logerr_throttle(1.0, "missing ceiling attitude candidate")
                self._publish_mode(self.OUTPUT_NONE)
                return
            target = self._attitude_thrust_target(
                now,
                self.status.attitude_setpoint,
                self.status.thrust_body_z_normalized,
            )
            if target is None:
                rospy.logerr_throttle(
                    1.0, "invalid/stale attitude or thrust candidate"
                )
                self._publish_mode(self.OUTPUT_NONE)
                return
            self.attitude_publisher.publish(target)
            self._publish_mode(self.OUTPUT_ATTITUDE_THRUST)
            return

        if self.publish_neutral_when_selected and self.status.state in (
            CeilingAttachmentStatus.STATE_CEILING_ARMED,
            CeilingAttachmentStatus.STATE_RECOVERY_HOVER,
        ):
            self.local_publisher.publish(self._local_velocity_target(now, 0.0))
            self._publish_mode(self.OUTPUT_LOCAL_VELOCITY)
            return

        self._publish_mode(self.OUTPUT_NONE)


if __name__ == "__main__":
    rospy.init_node("ceiling_attachment_output_node")
    CeilingAttachmentOutputNode()
    rospy.spin()
