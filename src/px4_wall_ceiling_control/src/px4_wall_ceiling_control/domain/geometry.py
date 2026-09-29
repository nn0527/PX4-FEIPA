"""Quaternion math used by the attachment decisions.

Quaternions are represented as ``(x, y, z, w)`` tuples so this module can be
tested without importing ROS message packages.
"""

import math

from px4_wall_ceiling_control.freshness import clamp


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_normalize(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in quaternion)


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_angle(a, b):
    """Smallest full attitude difference in radians."""

    a = quaternion_normalize(a)
    b = quaternion_normalize(b)
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(clamp(dot, 0.0, 1.0))


def follow_rotation_about_body_z(reference, measured):
    """Hold the reference thrust axis while following rotation about it.

    Both quaternions rotate body FRD into the local frame. Body Z is the
    opposite of multicopter thrust, so a Z twist leaves wall pressure aligned.
    Returns None for the singular case of a half turn about a transverse axis.
    """

    reference = quaternion_normalize(reference)
    measured = quaternion_normalize(measured)
    relative = quaternion_multiply(quaternion_conjugate(reference), measured)
    twist_norm = math.hypot(relative[2], relative[3])
    if twist_norm < 1.0e-6:
        return None
    twist = (0.0, 0.0, relative[2] / twist_norm, relative[3] / twist_norm)
    return quaternion_normalize(quaternion_multiply(reference, twist))


def quaternion_slerp(q0, q1, amount):
    q0 = quaternion_normalize(q0)
    q1 = quaternion_normalize(q1)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    dot = clamp(dot, -1.0, 1.0)
    amount = clamp(amount, 0.0, 1.0)
    if dot > 0.9995:
        return quaternion_normalize(
            tuple(a + amount * (b - a) for a, b in zip(q0, q1))
        )
    angle = math.acos(dot)
    sine = math.sin(angle)
    scale0 = math.sin((1.0 - amount) * angle) / sine
    scale1 = math.sin(amount * angle) / sine
    return tuple(scale0 * a + scale1 * b for a, b in zip(q0, q1))


def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    roll = math.atan2(
        2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)
    )
    pitch = math.asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(
        2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
    )
    return roll, pitch, yaw


def hover_and_pitched_quaternions(yaw, pitch):
    q_hover = (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
    q_pitch = (0.0, math.sin(pitch * 0.5), 0.0, math.cos(pitch * 0.5))
    return q_hover, quaternion_normalize(quaternion_multiply(q_hover, q_pitch))


def smoothstep5(value):
    value = clamp(value, 0.0, 1.0)
    return value ** 3 * (10.0 - 15.0 * value + 6.0 * value * value)


def rotate_vector(quaternion, vector):
    """Rotate a vector by a child-to-parent orientation quaternion."""

    x, y, z, w = quaternion_normalize(quaternion)
    vx, vy, vz = vector
    # Equivalent to q * [v, 0] * conjugate(q), expanded to avoid allocations.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )
