"""Bounded RC attitude trim for the single MAVROS attitude publisher."""

import math

from px4_wall_ceiling_control.domain.geometry import (
    quaternion_multiply,
    quaternion_normalize,
)
from px4_wall_ceiling_control.freshness import clamp


def stick_fraction(channels, index, low, center, high, deadzone, reverse=False):
    """Return a calibrated stick value in [-1, 1], or None for bad input."""

    if index < 0 or index >= len(channels) or not low < center < high:
        return None
    pwm = channels[index]
    if not math.isfinite(pwm) or pwm < low - 100 or pwm > high + 100:
        return None
    span = high - center if pwm >= center else center - low
    value = clamp((pwm - center) / span, -1.0, 1.0)
    if reverse:
        value = -value
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def trim_attitude(quaternion, roll_rad, pitch_rad):
    """Apply small body-axis trims after the behavior's base orientation."""

    roll = (math.sin(roll_rad / 2.0), 0.0, 0.0, math.cos(roll_rad / 2.0))
    pitch = (0.0, math.sin(pitch_rad / 2.0), 0.0, math.cos(pitch_rad / 2.0))
    return quaternion_normalize(
        quaternion_multiply(quaternion_multiply(quaternion, roll), pitch)
    )


def slew(previous, desired, max_step):
    return previous + clamp(desired - previous, -max_step, max_step)
