"""Legacy conservative controller shell for generic surface tracking."""

import math

import rospy
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

from px4_wall_ceiling_control.freshness import clamp, is_fresh, is_valid_range
from px4_wall_ceiling_control.msg import (
    BehaviorCommand,
    ControllerStatus,
    ControlSetpoint,
    SupervisorState,
)


class SurfaceController:
    """Produce a candidate velocity target; never publish to MAVROS directly."""

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

    def __init__(self, surface):
        if surface not in ("ceiling", "wall"):
            raise ValueError("surface must be 'ceiling' or 'wall'")

        self.surface = surface
        self.source = (
            ControlSetpoint.SOURCE_CEILING
            if surface == "ceiling"
            else ControlSetpoint.SOURCE_WALL
        )
        self.required_supervisor_state = (
            SupervisorState.STATE_CEILING_ACTIVE
            if surface == "ceiling"
            else SupervisorState.STATE_WALL_ACTIVE
        )

        self.controller_enabled = bool(
            rospy.get_param("~controller_enabled", False)
        )
        self.kp = float(rospy.get_param("~kp", 0.5))
        self.control_sign = float(rospy.get_param("~control_sign", 1.0))
        self.max_speed_mps = float(rospy.get_param("~max_speed_mps", 0.3))
        self.input_timeout_s = float(rospy.get_param("~input_timeout_s", 0.3))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.5))
        self.target_tolerance_m = float(
            rospy.get_param("~target_tolerance_m", 0.1)
        )
        self.local_frame_id = rospy.get_param("~local_frame_id", "map")
        self.body_frame_id = rospy.get_param("~body_frame_id", "base_link")
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))

        behavior_topic = rospy.get_param(
            "~behavior_topic", "/{}_fsm/command".format(surface)
        )
        range_topic = rospy.get_param(
            "~range_topic", "/sensor/{}/range".format(surface)
        )
        odom_topic = rospy.get_param(
            "~odom_topic", "/mavros/local_position/odom"
        )
        supervisor_topic = rospy.get_param("~supervisor_topic", "/supervisor/state")
        candidate_topic = rospy.get_param(
            "~candidate_topic", "/control/{}_candidate".format(surface)
        )
        status_topic = rospy.get_param(
            "~status_topic", "/control/{}/status".format(surface)
        )

        self.behavior = None
        self.behavior_received_at = None
        self.range_message = None
        self.range_received_at = None
        self.odom = None
        self.odom_received_at = None
        self.supervisor = None
        self.supervisor_received_at = None

        self.candidate_publisher = rospy.Publisher(
            candidate_topic, ControlSetpoint, queue_size=10
        )
        self.status_publisher = rospy.Publisher(
            status_topic, ControllerStatus, queue_size=10
        )
        rospy.Subscriber(
            behavior_topic, BehaviorCommand, self._on_behavior, queue_size=10
        )
        rospy.Subscriber(range_topic, Range, self._on_range, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber(
            supervisor_topic, SupervisorState, self._on_supervisor, queue_size=10
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 1.0)), self._on_timer
        )

        if not self.controller_enabled:
            rospy.logwarn(
                "%s controller is disabled; candidates remain invalid", surface
            )

    def _on_behavior(self, message):
        self.behavior = message
        self.behavior_received_at = rospy.Time.now()

    def _on_range(self, message):
        self.range_message = message
        self.range_received_at = rospy.Time.now()

    def _on_odom(self, message):
        self.odom = message
        self.odom_received_at = rospy.Time.now()

    def _on_supervisor(self, message):
        self.supervisor = message
        self.supervisor_received_at = rospy.Time.now()

    def _on_timer(self, _event):
        now = rospy.Time.now()
        candidate, status = self._build_candidate(now)
        self.candidate_publisher.publish(candidate)
        self.status_publisher.publish(status)

    def _build_candidate(self, now):
        candidate = ControlSetpoint()
        candidate.header.stamp = now
        candidate.source = self.source
        candidate.kind = ControlSetpoint.KIND_LOCAL
        candidate.valid = False
        candidate.reason = "controller disabled"

        status = ControllerStatus()
        status.header.stamp = now
        status.source = self.source
        status.healthy = False
        status.input_fresh = False
        status.target_reached = False
        status.detail = candidate.reason

        local = PositionTarget()
        local.header.stamp = now
        local.type_mask = self.VELOCITY_ONLY_MASK
        if self.surface == "ceiling":
            # MAVROS consumes ROS-side ENU data and converts it for PX4.
            local.header.frame_id = self.local_frame_id
            local.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        else:
            # Validate BODY_NED/ROS FLU conversion and sensor mounting before flight.
            local.header.frame_id = self.body_frame_id
            local.coordinate_frame = PositionTarget.FRAME_BODY_NED
        candidate.local = local

        inputs_fresh = (
            is_fresh(self.behavior_received_at, self.input_timeout_s, now)
            and is_fresh(self.range_received_at, self.input_timeout_s, now)
            and is_fresh(self.odom_received_at, self.odom_timeout_s, now)
            and is_fresh(self.supervisor_received_at, self.input_timeout_s, now)
        )
        valid_range = is_valid_range(self.range_message)
        status.input_fresh = inputs_fresh
        status.healthy = inputs_fresh and valid_range

        if not self.controller_enabled:
            status.detail = candidate.reason
            return candidate, status
        if not inputs_fresh:
            candidate.reason = "controller input timeout"
            status.detail = candidate.reason
            return candidate, status
        if not valid_range:
            candidate.reason = "invalid surface range"
            status.detail = candidate.reason
            return candidate, status
        if (
            self.supervisor.state != self.required_supervisor_state
            or not self.supervisor.motion_allowed
        ):
            candidate.reason = "surface is not the active motion source"
            status.detail = candidate.reason
            return candidate, status
        if not self.behavior.active or self.behavior.source != self.source:
            candidate.reason = "behavior command inactive or source mismatch"
            status.detail = candidate.reason
            return candidate, status

        error_m = self.range_message.range - self.behavior.target_distance_m
        requested_limit = self.behavior.max_speed_mps
        speed_limit = self.max_speed_mps
        if requested_limit > 0.0:
            speed_limit = min(speed_limit, requested_limit)
        speed_mps = clamp(
            self.control_sign * self.kp * error_m, -speed_limit, speed_limit
        )
        if not math.isfinite(speed_mps):
            candidate.reason = "non-finite controller output"
            status.detail = candidate.reason
            return candidate, status

        # TODO: compensate sensor slant range using attitude and calibrated extrinsics.
        if self.surface == "ceiling":
            candidate.local.velocity.z = speed_mps
        else:
            candidate.local.velocity.x = speed_mps

        candidate.valid = True
        candidate.reason = "placeholder P controller candidate"
        status.target_reached = abs(error_m) <= self.target_tolerance_m
        status.detail = "candidate valid"
        return candidate, status


def run_surface_controller(surface):
    rospy.init_node("{}_controller_node".format(surface))
    SurfaceController(surface)
    rospy.spin()
