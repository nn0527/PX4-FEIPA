"""Frame-explicit conversion and validation for ``nav_msgs/Odometry``."""

import math

from px4_wall_ceiling_control.domain.geometry import rotate_vector


def quaternion_tuple(odometry):
    orientation = odometry.pose.pose.orientation
    return (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )


def odometry_is_finite(odometry):
    if odometry is None:
        return False
    position = odometry.pose.pose.position
    linear = odometry.twist.twist.linear
    angular = odometry.twist.twist.angular
    quaternion = quaternion_tuple(odometry)
    values = (
        position.x,
        position.y,
        position.z,
        *quaternion,
        linear.x,
        linear.y,
        linear.z,
        angular.x,
        angular.y,
        angular.z,
    )
    return bool(
        all(math.isfinite(value) for value in values)
        and math.sqrt(sum(value * value for value in quaternion)) > 1.0e-6
    )


def linear_velocity_parent_frame(odometry, twist_in_child_frame=True):
    """Return linear velocity in the odometry parent/world frame.

    ``nav_msgs/Odometry`` defines twist in ``child_frame_id``. MAVROS commonly
    publishes a body child frame, so the pose orientation rotates this vector
    into the ENU parent frame. A compatibility flag supports sources that
    already publish world-frame twist despite the message convention.
    """

    linear = odometry.twist.twist.linear
    vector = (linear.x, linear.y, linear.z)
    if not twist_in_child_frame:
        return vector
    return rotate_vector(quaternion_tuple(odometry), vector)


def vertical_speed_enu_mps(odometry, twist_in_child_frame=True):
    return linear_velocity_parent_frame(odometry, twist_in_child_frame)[2]

