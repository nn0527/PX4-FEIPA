"""Ceiling-specific motion envelope; positions/velocities use world ENU."""
import math
from dataclasses import dataclass, fields

from px4_wall_ceiling_control.domain.horizontal_velocity import HorizontalVelocityController
from px4_wall_ceiling_control.domain.recovery_attitude_limits import (
    attitude_from_axis_yaw, cone_axis, limit_recovery_attitude,
)


@dataclass(frozen=True)
class CeilingMotionSettings:
    odom_vertical_jump_m: float = .25
    horizontal_position_deadband_m: float = .10
    horizontal_position_kp: float = .30
    rc_manual_position_weight: float = .25
    rc_manual_blend_s: float = .50
    rc_manual_feedback_timeout_s: float = .20
    rc_manual_zero_deg: float = .05
    horizontal_max_speed_mps: float = .20
    horizontal_velocity_kp: float = 1.5
    horizontal_velocity_ki: float = .30
    horizontal_integral_limit_mps2: float = .20
    horizontal_max_acceleration_mps2: float = .40
    horizontal_target_acceleration_mps2: float = .10
    horizontal_drift_warn_m: float = .30
    horizontal_drift_pause_m: float = 1.0
    horizontal_drift_resume_m: float = .60
    attitude_max_tilt_deg: float = 10.
    contact_max_tilt_deg: float = 5.
    yaw_max_offset_deg: float = 10.
    attitude_tilt_slew_deg_s: float = 15.
    attitude_yaw_slew_deg_s: float = 15.
    actual_tilt_pause_deg: float = 15.
    actual_tilt_resume_deg: float = 12.
    actual_body_rate_pause_deg_s: float = 30.
    actual_body_rate_resume_deg_s: float = 20.
    motion_limit_confirm_s: float = .20
    motion_resume_hold_s: float = .50
    motion_feedback_timeout_s: float = 2.
    motion_pause_timeout_s: float = 8.
    motion_flow_timeout_s: float = .60
    motion_flow_future_tolerance_s: float = .10
    motion_flow_quality_min: float = 1.
    motion_estimator_timeout_s: float = 1.5
    motion_reacquire_time_s: float = .30
    motion_equilibrium_window_s: float = .50
    motion_equilibrium_speed_mps: float = .20
    motion_equilibrium_max_gap_s: float = .35
    motion_equilibrium_max_spread_deg: float = 1.5
    motion_fcu_target_timeout_s: float = .35
    motion_capture_timeout_s: float = 2.
    vertical_speed_kp: float = .30
    detach_brake_speed_mps: float = .10
    detach_velocity_ki: float = .10
    detach_integral_limit: float = .12
    detach_thrust_transition_s: float = .30
    detach_thrust_handover_slew_per_s: float = .20
    vertical_target_acceleration_mps2: float = .15
    detach_min_down_speed_mps: float = .05
    recovery_height_kp: float = .50
    recovery_height_tolerance_m: float = .10
    recovery_handover_height_tolerance_m: float = .15
    recovery_vertical_speed_mps: float = .15
    recovery_horizontal_speed_mps: float = .15
    recovery_max_tilt_deg: float = 10.
    recovery_max_rate_deg_s: float = 20.
    recovery_stable_time_s: float = .50
    recovery_timeout_s: float = 15.
    contact_release_distance_m: float = .10
    contact_release_hold_s: float = .20
    odom_position_jump_m: float = .75
    odom_yaw_jump_deg: float = 20.

    @classmethod
    def read(cls, getter):
        return cls(**{f.name: float(getter('~' + f.name, f.default)) for f in fields(cls)})

    def __post_init__(self):
        if any(not math.isfinite(getattr(self, f.name)) or getattr(self, f.name) <= 0
               for f in fields(self) if f.name != 'motion_flow_future_tolerance_s'):
            raise ValueError('ceiling motion parameters must be finite and positive')
        if (not 0 < self.rc_manual_position_weight <= 1.
                or not 0 <= self.motion_flow_future_tolerance_s <= .10
                or not 1 <= self.motion_flow_quality_min <= 255
                or not self.horizontal_position_deadband_m < self.horizontal_drift_warn_m < self.horizontal_drift_pause_m
                or not self.horizontal_position_deadband_m < self.horizontal_drift_resume_m < self.horizontal_drift_pause_m
                or not self.contact_max_tilt_deg <= self.attitude_max_tilt_deg < self.actual_tilt_pause_deg < 45
                or not self.actual_tilt_resume_deg < self.actual_tilt_pause_deg
                or not self.actual_body_rate_resume_deg_s < self.actual_body_rate_pause_deg_s
                or self.motion_equilibrium_speed_mps > .30
                or self.motion_equilibrium_max_gap_s > 1.
                or self.motion_equilibrium_max_spread_deg > 5.
                or self.horizontal_max_speed_mps > .30
                or self.recovery_max_tilt_deg > self.actual_tilt_pause_deg
                or self.recovery_stable_time_s >= self.recovery_timeout_s):
            raise ValueError('invalid ceiling motion envelope/hysteresis')


class CeilingMotionController:
    def __init__(self, settings):
        self.settings = settings
        self.horizontal = HorizontalVelocityController(
            settings.horizontal_velocity_kp, settings.horizontal_max_acceleration_mps2,
            settings.horizontal_target_acceleration_mps2, settings.horizontal_velocity_ki,
            settings.horizontal_integral_limit_mps2)
        self.reset()

    def reset(self):
        self.horizontal.reset()
        self.xy = None
        self.yaw = 0.
        self.axis = (0., 0., 1.)
        self.vertical_support = None
        self.vz_target = 0.
        self.vertical_integral = 0.
        self.detach_transition_active = False
        self.detach_transition_elapsed = 0.
        self.detach_transition_command = None

    def begin_detach_transition(self, entry, baseline):
        self.detach_transition_active = True
        self.detach_transition_elapsed = 0.
        self.detach_transition_entry = self.detach_transition_command = entry
        self.detach_transition_baseline = baseline
        self.detach_transition_rate = max(self.settings.detach_thrust_handover_slew_per_s,
            abs(entry-baseline)/self.settings.detach_thrust_transition_s)

    def detach_transition_thrust(self, requested, dt, advance=True, brake=False):
        """Quick entry-to-baseline ramp, then slew into PI without a second step."""
        if not self.detach_transition_active:
            return requested
        dt = min(.05, max(0., dt)) if advance else 0.
        duration = self.settings.detach_thrust_transition_s
        if brake:
            # An already excessive descent must not wait for a pressure reduction.
            self.detach_transition_elapsed = duration
            self.detach_transition_command = max(self.detach_transition_command, requested)
        elif self.detach_transition_elapsed < duration-1e-9:
            self.detach_transition_elapsed = min(duration, self.detach_transition_elapsed+dt)
            fraction = self.detach_transition_elapsed/duration
            self.detach_transition_command = self.detach_transition_entry + fraction*(
                self.detach_transition_baseline-self.detach_transition_entry)
            return self.detach_transition_command
        step = self.detach_transition_rate*dt
        error = requested-self.detach_transition_command
        self.detach_transition_command += max(-step, min(step, error))
        if advance and abs(requested-self.detach_transition_command) <= 1e-9:
            self.detach_transition_active = False
        return self.detach_transition_command

    def descent_velocity(self, height_error, speed_limit, height_kp):
        """Mirror ascent's distance/proportional/braking envelope; ENU positive up."""
        if height_error <= 0.:
            return min(.10, -height_kp*height_error)
        return -min(speed_limit, height_kp*height_error,
                    math.sqrt(2.*self.settings.vertical_target_acceleration_mps2*height_error))

    def recovery_velocity(self, height_error, speed_limit):
        """Brake inside the accepted height band; return to its nearest edge outside."""
        tolerance = self.settings.recovery_height_tolerance_m
        excess = max(0., abs(height_error)-tolerance)
        error = math.copysign(excess, height_error)
        return self.descent_velocity(error, speed_limit, self.settings.recovery_height_kp)

    def vertical_pi(self, measured, axis_z, minimum, maximum, integral_dt=0.):
        """Sample-gated PI with conditional anti-windup at the final thrust limits."""
        error = self.vz_target-measured
        dt = integral_dt if 0. < integral_dt <= .20 else 0.
        candidate = max(-self.settings.detach_integral_limit,
                        min(self.settings.detach_integral_limit,
                            self.vertical_integral+self.settings.detach_velocity_ki*error*dt))
        raw = (self.vertical_support+self.settings.vertical_speed_kp*error+candidate)/axis_z
        # Stop accumulating into saturation; opposite errors can unwind it.
        if not ((raw < minimum and error < 0.) or (raw > maximum and error > 0.)):
            self.vertical_integral = candidate
        raw = (self.vertical_support+self.settings.vertical_speed_kp*error+self.vertical_integral)/axis_z
        return max(minimum, min(maximum, raw))

    def descent_braking_thrust(self, requested, measured, axis_z, minimum, maximum):
        """Enforce upward braking support without clipping measured ENU velocity."""
        excess = -measured-self.settings.detach_brake_speed_mps
        if excess <= 0.:
            return requested
        # A negative descent integral must not cancel the overspeed brake.
        floor = (self.vertical_support+self.settings.vertical_speed_kp*excess)/axis_z
        return max(minimum, min(maximum, max(requested, floor)))

    def capture(self, xy, yaw, axis, thrust):
        self.reset()
        self.xy, self.yaw, self.axis = tuple(xy), yaw, tuple(axis)
        self.vertical_support = thrust * axis[2]

    def drift(self, xy):
        return math.hypot(*(a-b for a, b in zip(self.xy, xy))) if self.xy is not None else math.inf

    def attitude(self, xy, velocity, dt, integrate=False, integral_dt=0., applied_axis=None, position_weight=1.):
        s = self.settings
        error = tuple(a-b for a, b in zip(self.xy, xy))
        distance = math.hypot(*error)
        speed = min(s.horizontal_max_speed_mps,
                    s.horizontal_position_kp * max(0., distance-s.horizontal_position_deadband_m))
        speed *= max(0., min(1., position_weight))
        desired = tuple(v*speed/distance for v in error) if distance > 0 else (0., 0.)
        return self.horizontal.attitude(desired, velocity, self.yaw, dt, self.axis,
            integrate=integrate, integral_dt=integral_dt,
            max_tilt=math.radians(s.attitude_max_tilt_deg), applied_axis=applied_axis)

    def vertical_target(self, desired, dt):
        step = self.settings.vertical_target_acceleration_mps2 * max(0., min(.05, dt))
        self.vz_target += max(-step, min(step, desired-self.vz_target))
        return self.vz_target


def limit_ceiling_attitude(desired, previous, dt, yaw_reference, settings, contact=False, follow_hold_yaw=False):
    """Limit the FINAL target, after RC trim; a hard cone overrides slew on entry."""
    from px4_wall_ceiling_control.domain.geometry import rotate_vector, euler_from_quaternion
    yaw = euler_from_quaternion(desired)[2]
    offset = math.atan2(math.sin(yaw-yaw_reference), math.cos(yaw-yaw_reference))
    offset = max(-math.radians(settings.yaw_max_offset_deg),
                 min(math.radians(settings.yaw_max_offset_deg), offset))
    tilt = math.radians(settings.contact_max_tilt_deg if contact else settings.attitude_max_tilt_deg)
    target = attitude_from_axis_yaw(cone_axis(rotate_vector(desired, (0., 0., 1.)), tilt), yaw_reference+offset)
    if follow_hold_yaw and previous is not None:
        # HOLD follows measured wheel rotation; keep the tilt slew/cone intact.
        previous = attitude_from_axis_yaw(rotate_vector(previous, (0., 0., 1.)), yaw_reference+offset)
    return limit_recovery_attitude(target, previous, min(.05, max(0., dt)), tilt,
        math.radians(settings.attitude_tilt_slew_deg_s), math.radians(settings.attitude_yaw_slew_deg_s))[0]
