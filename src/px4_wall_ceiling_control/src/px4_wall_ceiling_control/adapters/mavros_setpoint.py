"""Build bounded MAVROS setpoint messages from behavior-neutral values."""

import copy
import math

from mavros_msgs.msg import AttitudeTarget, PositionTarget

from px4_wall_ceiling_control.freshness import clamp


class MavrosSetpointBuilder:
    LOCAL_VELOCITY_MASK = (
        PositionTarget.IGNORE_PX
        | PositionTarget.IGNORE_PY
        | PositionTarget.IGNORE_PZ
        | PositionTarget.IGNORE_AFX
        | PositionTarget.IGNORE_AFY
        | PositionTarget.IGNORE_AFZ
        | PositionTarget.IGNORE_YAW
        | PositionTarget.IGNORE_YAW_RATE
    )
    ATTITUDE_AND_THRUST_MASK = (
        AttitudeTarget.IGNORE_ROLL_RATE
        | AttitudeTarget.IGNORE_PITCH_RATE
        | AttitudeTarget.IGNORE_YAW_RATE
    )

    def __init__(self, max_velocity_z_mps, min_thrust, max_thrust, frame_id="map"):
        self.max_velocity_z_mps = abs(float(max_velocity_z_mps))
        self.min_thrust = float(min_thrust)
        self.max_thrust = float(max_thrust)
        self.frame_id = frame_id

    def local_velocity(self, stamp, velocity_z_enu_mps=0.0):
        if not math.isfinite(velocity_z_enu_mps):
            return None
        target = PositionTarget()
        target.header.stamp = stamp
        target.header.frame_id = self.frame_id
        # Values are populated in ROS ENU; MAVROS converts them to PX4 NED.
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = self.LOCAL_VELOCITY_MASK
        target.velocity.x = 0.0
        target.velocity.y = 0.0
        target.velocity.z = clamp(
            velocity_z_enu_mps,
            -self.max_velocity_z_mps,
            self.max_velocity_z_mps,
        )
        return target

    def attitude_thrust(self, stamp, orientation, thrust):
        values = (orientation.x, orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1.0e-6 or not math.isfinite(thrust):
            return None
        target = AttitudeTarget()
        target.header.stamp = stamp
        target.type_mask = self.ATTITUDE_AND_THRUST_MASK
        target.orientation = copy.deepcopy(orientation)
        target.orientation.x /= norm
        target.orientation.y /= norm
        target.orientation.z /= norm
        target.orientation.w /= norm
        target.thrust = clamp(thrust, self.min_thrust, self.max_thrust)
        return target

