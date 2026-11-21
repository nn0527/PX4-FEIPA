"""Incline contact/return state machine. No ROS or motor allocation here.

Angles are radians internally; q rotates body FLU into world ENU. All timers
consume wall time, while motion steps are bounded and evidence is source-timed.
"""
from dataclasses import dataclass
import math
import statistics

from .geometry import (clamp, euler_from_quaternion, follow_rotation_about_body_z,
                       hover_and_pitched_quaternions,
                       quaternion_angle, quaternion_multiply, quaternion_normalize,
                       quaternion_slerp, rotate_vector, smoothstep5)
from .detach_braking import BrakingTrajectory, ReturnAxis, allowed_rate, braking_goal
from .departure_height import DepartureHeightController
from .incline_return_support import InclineReturnSupport
from .simple_recovery import ENTRY_TILT_DEG, STABLE_TIME_S, attitude_within_margins


DETACH_YAW_LIMIT_DEG = 30


@dataclass(frozen=True)
class InclineSettings:
    # Optional calibrated feasibility model. Zero friction disables the model;
    # invalid/insufficient range geometry always falls back to contact search.
    friction_coefficient: float = 0.
    contact_pitch_bias_deg: float = 0.
    normal_margin_ratio: float = .05
    estimate_min_travel_m: float = .08
    estimate_max_residual_m: float = .005
    enabled: bool = False
    min_entry_height_m: float = .10
    front_ready_distance_m: float = 1.80
    flip_trigger_distance_m: float = .18
    top_contact_distance_m: float = .070
    front_ready_hold_s: float = .40
    approach_timeout_s: float = 20.
    horizontal_approach_speed_mps: float = .10
    flip_max_tilt_deg: float = 65.
    flip_pitch_rate_deg_s: float = 10.
    flip_pitch_accel_deg_s2: float = 10.
    flip_max_tracking_error_deg: float = 10.
    flip_abort_roll_error_deg: float = 10.
    flip_abort_pitch_error_deg: float = 13.
    flip_abort_height_drop_m: float = .30
    flip_abort_confirm_s: float = .20
    flip_duration_timeout_s: float = 8.
    flip_still_window_s: float = .50
    flip_still_pitch_span_deg: float = 1.
    flip_still_min_pitch_error_deg: float = 8.
    flip_thrust_ramp_per_s: float = .03
    # Independent vertical support; the slow contact-search ramp is unchanged.
    flip_support_height_kp: float = .30
    flip_support_velocity_kp: float = .30
    flip_support_correction_max: float = .08
    flip_support_slew_per_s: float = .20
    # Collective feedforward starts only at the front-range flip transition.
    flip_entry_boost: float = .04
    flip_entry_boost_rise_s: float = .40
    flip_thrust_max: float = 0.  # Required calibration, never inherit wall cap.
    top_contact_hold_s: float = .20
    contact_settle_timeout_s: float = 6.
    contact_attitude_tolerance_deg: float = 10.
    contact_max_rate_deg_s: float = 10.
    hold_entry_blend_s: float = .50
    hold_contact_loss_margin_m: float = .015
    hold_contact_loss_hold_s: float = .30
    detach_max_rate_deg_s: float = 30.
    detach_max_accel_deg_s2: float = 30.
    detach_max_tracking_error_deg: float = 11.
    detach_duration_timeout_s: float = 10.
    detach_brake_target_deg: float = 13.
    detach_brake_decel_deg_s2: float = 30.
    detach_brake_delay_s: float = .15
    detach_brake_margin_deg: float = 5.
    detach_recovered_tilt_deg: float = 7.
    detach_recovered_rate_deg_s: float = 10.
    detach_recovered_hold_s: float = .30
    return_thrust_slew_per_s: float = .20
    return_thrust_drop_slew_per_s: float = .05
    recovery_hold_s: float = 1.
    recover_timeout_s: float = 15.
    recovery_max_speed_mps: float = .20
    recovery_max_height_error_m: float = .10
    height_kp: float = .30
    velocity_kp: float = .30

    @classmethod
    def read(cls, getter):
        default = cls()
        return cls(**{key: getter(key, getattr(default, key)) for key in cls.__dataclass_fields__})

    def error(self, global_cap):
        if type(self.enabled) is not bool:
            return 'incline enabled must be boolean'
        for key in self.__dataclass_fields__:
            if key == 'enabled':
                continue
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                return 'invalid incline parameter: ' + key
            if key not in ('flip_thrust_max', 'friction_coefficient', 'contact_pitch_bias_deg',
                           'flip_entry_boost') and value <= 0:
                return 'incline parameter must be positive: ' + key
        if self.friction_coefficient < 0 or abs(self.contact_pitch_bias_deg) > 10.:
            return 'invalid incline friction or installation bias'
        if not .1 < self.flip_thrust_max <= global_cap:
            return '斜墙flip_thrust_max未标定或超过统一输出上限'
        if not self.enabled:
            return '斜墙模式未启用'
        if not self.top_contact_distance_m < self.flip_trigger_distance_m < self.front_ready_distance_m:
            return 'invalid incline distance ordering'
        if not 60. <= self.flip_max_tilt_deg <= 65.:
            return 'incline tilt cap must cover 60 degrees and not exceed 65'
        if not (max(self.detach_recovered_tilt_deg, self.detach_brake_margin_deg)
                < self.detach_brake_target_deg < self.flip_max_tilt_deg < 90.):
            return 'invalid incline braking angles'
        if self.top_contact_hold_s >= self.contact_settle_timeout_s:
            return 'contact timeout must exceed confirmation window'
        if self.flip_pitch_rate_deg_s > 10. or self.flip_max_tracking_error_deg > 10.:
            return 'incline flip rate/tracking exceeds initial envelope'
        if self.flip_still_min_pitch_error_deg >= self.flip_abort_pitch_error_deg:
            return 'incline still-pitch gate must be below pitch abort threshold'
        if (self.flip_thrust_ramp_per_s > .03 or self.detach_max_tracking_error_deg > 11.
                or self.detach_max_rate_deg_s > 30. or self.detach_recovered_tilt_deg > 7.):
            return 'incline motion limits exceed initial envelope'
        if self.flip_duration_timeout_s <= self.flip_max_tilt_deg / self.flip_pitch_rate_deg_s:
            return 'incline flip timeout too short for configured rate'
        if (self.flip_support_height_kp > .5 or self.flip_support_velocity_kp > .5
                or self.flip_support_correction_max > .12
                or not self.flip_thrust_ramp_per_s <= self.flip_support_slew_per_s <= .5):
            return 'incline flip support exceeds bounded envelope'
        if (not 0. <= self.flip_entry_boost <= self.flip_support_correction_max
                or self.flip_entry_boost_rise_s >= self.flip_duration_timeout_s
                or 1.875*self.flip_entry_boost/self.flip_entry_boost_rise_s
                    > self.flip_support_slew_per_s+1e-9):
            return 'incline entry boost exceeds support amount/rate budget'
        if self.detach_recovered_hold_s >= self.detach_duration_timeout_s or self.recovery_hold_s >= self.recover_timeout_s:
            return 'incline recovery timeout too short'
        if not self.return_thrust_drop_slew_per_s <= self.return_thrust_slew_per_s <= .5:
            return 'invalid incline return thrust slew limits'
        return ''


@dataclass
class InclineFeedback:
    now: float
    q: tuple
    body_rate: tuple
    height: float
    vz: float
    horizontal_speed: float
    pose_stamp: float
    x: float = math.nan
    y: float = math.nan
    front: float = math.nan
    top: float = math.nan
    top_stamp: float = 0.
    front_valid: bool = False
    top_valid: bool = False
    pose_valid: bool = False
    control_valid: bool = False
    applied_q: tuple = None
    applied_thrust: float = math.nan
    output_valid: bool = False
    output_stamp: float = 0.


class InclineAttachment:
    ACTIVE = ('APPROACH', 'SLOW_FLIP', 'CONTACT_SETTLE', 'HOLD', 'RETURN', 'RECOVER', 'EXIT')

    def __init__(self, settings, global_cap):
        self.s = settings
        self.error = settings.error(global_cap)
        self.cap = min(global_cap, settings.flip_thrust_max) if not self.error else .1
        self.phase = 'WAIT'
        self.reason = self.error or '等待斜墙启动'
        self.fault = ''
        self.q = None
        self.thrust = math.nan
        self.last_at = self.entered = None
        self.stable_since = self.stable_stamp = None
        self.loss_since = None
        self.contact_seen = False
        self.planner = None
        self.reference_height = math.nan
        self.base = math.nan
        self.return_started = None
        self.return_support = None
        # Measured yaw at detach-command receipt, not the commanded attitude.
        self.detach_yaw_reference = None
        self.detach_yaw_reference_stamp = None
        self.detach_yaw_request_at = None
        self.recovery_started = None
        self.return_tracking_limited = False
        self.recover_stable = False
        self.return_history = []
        self.estimate_samples = []
        self.estimated_angle = None
        self.contact_samples = []
        self.contact_angle = None
        self.required_thrust = None
        self.flip_entry_height = math.nan
        self.flip_abort_since = {}
        self.flip_abort_stamp = None
        self.flip_pitch_history = []
        self.flip_top_stamp = 0.
        self.contact_top_stamp = 0.
        self.contact_top_received_at = None
        self.flip_entry_thrust = self.flip_pressure_thrust = math.nan
        self.flip_support_requested = self.flip_support_correction = math.nan
        self.flip_boost_elapsed = self.flip_boost_value = 0.

    @staticmethod
    def tilt(q):
        return math.acos(clamp(rotate_vector(q, (0., 0., 1.))[2], -1., 1.))

    def _enter(self, phase, now, reason):
        self.phase, self.entered, self.reason = phase, now, reason
        if phase == 'CONTACT_SETTLE':
            self.contact_top_received_at = now
        if phase != 'EXIT':
            self.stable_since = self.stable_stamp = None

    def _stable(self, condition, stamp, now, duration, max_gap=.20):
        if not condition or stamp <= 0. or stamp > now:
            self.stable_since = self.stable_stamp = None
            return False
        if self.stable_stamp is None:
            self.stable_since = self.stable_stamp = stamp
        elif stamp < self.stable_stamp:
            self.stable_since = self.stable_stamp = stamp
        elif stamp > self.stable_stamp:
            if stamp - self.stable_stamp > max_gap:
                self.stable_since = stamp
            self.stable_stamp = stamp
        return self.stable_stamp - self.stable_since >= duration - 1e-8

    def _required(self, angle):
        # T/mg >= cos(alpha) + sin(alpha)/mu + normal margin.
        # Assumes linear calibrated thrust and body axis normal to the plane.
        if self.s.friction_coefficient <= 0.:
            return None
        return self.base * (math.cos(angle) + math.sin(angle) /
            self.s.friction_coefficient + self.s.normal_margin_ratio)

    def _estimate(self, f):
        if not (f.top_valid and all(math.isfinite(v) for v in
                (f.x, f.y, f.height, f.top)) and self.tilt(f.q) <= math.radians(3.)):
            self.estimate_samples = []
            self.estimated_angle = None
            return
        yaw = euler_from_quaternion(self.hover)[2]
        travel = f.x*math.cos(yaw) + f.y*math.sin(yaw)
        point = (f.pose_stamp, travel, f.height + f.top)
        if self.estimate_samples and point[0] <= self.estimate_samples[-1][0]:
            return
        self.estimate_samples.append(point)
        self.estimate_samples = self.estimate_samples[-200:]
        samples = self.estimate_samples
        if len(samples) < 5 or max(v[1] for v in samples)-min(v[1] for v in samples) < self.s.estimate_min_travel_m:
            return
        x = statistics.mean(v[1] for v in samples)
        z = statistics.mean(v[2] for v in samples)
        slope = sum((v[1]-x)*(v[2]-z) for v in samples)/sum((v[1]-x)**2 for v in samples)
        residual = max(abs(v[2]-z-slope*(v[1]-x)) for v in samples)
        angle = math.atan(abs(slope))
        self.estimated_angle = angle if residual <= self.s.estimate_max_residual_m and angle <= math.radians(self.s.flip_max_tilt_deg) else None

    def start(self, f, seed_q, seed_thrust):
        if self.error or self.phase != 'WAIT' or not (f.pose_valid and f.control_valid):
            return False
        if not (math.isfinite(seed_thrust) and .1 <= seed_thrust < self.cap
                and all(math.isfinite(v) for v in seed_q) and sum(v*v for v in seed_q) > .5):
            self.reason = '斜墙接管推力必须低于标定上限，姿态必须有效'
            return False
        self.q = quaternion_normalize(seed_q)
        yaw = euler_from_quaternion(self.q)[2]
        self.hover = hover_and_pitched_quaternions(yaw, 0.)[0]
        self.thrust = seed_thrust
        self.base = seed_thrust * rotate_vector(self.q, (0., 0., 1.))[2]
        if not .1 <= self.base < self.cap:
            self.reason = '斜墙接管竖直支撑基准无效'
            return False
        self.reference_height = f.height
        self.last_at = f.now
        self._enter('APPROACH', f.now, '斜墙低速接近')
        self.support = DepartureHeightController(max_thrust=self.cap,
            thrust_slew=self.s.return_thrust_slew_per_s, compensation_start_deg=80.)
        self.support.begin(f.height, self.base, self.thrust, f.now)
        return True

    def abort(self, f, reason):
        if self.phase in ('WAIT', 'ABORT_DONE', 'LAND'):
            return
        if not self.fault:
            self.fault = reason
        self._land(f, self.fault)

    def _land(self, f, reason):
        self.fault = self.fault or reason
        self._enter('LAND', f.now, '斜墙ABORT，自动降落：' + self.fault)

    def latch_detach_yaw(self, f):
        """Capture once at command receipt; repeated commands cannot reanchor."""
        if self.phase != 'HOLD':
            return False
        if self.detach_yaw_reference is not None:
            return True
        if (not f.pose_valid or f.q is None
                or not all(math.isfinite(v) for v in f.q) or f.pose_stamp <= 0.):
            self._land(f, '脱离指令时实测姿态无效，无法锁定偏航基准')
            return False
        self.detach_yaw_reference = euler_from_quaternion(f.q)[2]
        self.detach_yaw_reference_stamp = f.pose_stamp
        self.detach_yaw_request_at = f.now
        return True

    def _return(self, f):
        if not (f.pose_valid and f.control_valid and f.output_valid):
            self._land(f, '回平必要反馈或控制权失效')
            return
        # Direct domain callers deliver detach in this step. The ROS adapter
        # has already latched the earlier callback sample and must preserve it.
        if self.detach_yaw_reference is None and not self.latch_detach_yaw(f):
            return
        self.q, self.thrust = f.applied_q, f.applied_thrust
        if not .1 <= self.thrust <= self.cap + 1e-6:
            self._land(f, '最终推力超出斜墙支撑上限')
            return
        if self.thrust+1e-6 < self.base:
            self._land(f, '保持实际推力低于已记录悬停基准，禁止不连续回平')
            return
        self.return_started = f.now
        self.return_history = [(f.output_stamp, self.q)]
        self.axis = ReturnAxis(self.hover, self.q)
        self.return_seed = self.axis.angle(self.q)
        # Two actual publication samples are required before planning velocity.
        self.planner = None
        self.reference_height = f.height
        self.support.begin(f.height, self.base, self.thrust, f.now)
        self.return_support = InclineReturnSupport(self.support, self.thrust,
            rotate_vector(f.q, (0., 0., 1.))[2], self.s.return_thrust_slew_per_s,
            self.s.return_thrust_drop_slew_per_s)
        self._enter('RETURN', f.now, self.reason if self.fault else '斜墙支撑脱离回平')

    def _update_return_support(self, f):
        if self.return_support is None:
            self._land(f, '斜墙回平支撑未初始化')
            return
        self.thrust = self.return_support.update(f.now, f.height, f.vz,
            rotate_vector(f.q, (0., 0., 1.))[2], f.applied_thrust,
            self.s.height_kp, self.s.velocity_kp)

    def _flip_motion_fault(self, f):
        """Immediate pitch guard; confirm roll tracking and entry-height loss.

        Compare against the published attitude, and count source time so a
        repeated feedback sample cannot confirm a persistent deviation.
        """
        if self.flip_abort_stamp is not None:
            gap = f.pose_stamp-self.flip_abort_stamp
            if gap < 0. or gap > .20+1e-8:
                self.flip_abort_since.clear()
        self.flip_abort_stamp = f.pose_stamp
        measured = euler_from_quaternion(f.q)
        sent = euler_from_quaternion(f.applied_q)
        errors = [abs(math.degrees(math.atan2(math.sin(a-b), math.cos(a-b))))
                  for a, b in zip(measured[:2], sent[:2])]
        checks = (
            ('横滚', errors[0], self.s.flip_abort_roll_error_deg, 'deg'),
            ('掉高', self.flip_entry_height-f.height, self.s.flip_abort_height_drop_m, 'm'),
        )
        if errors[1] > self.s.flip_abort_pitch_error_deg+1e-8:
            return ('斜墙翻转俯仰超限：%.3fdeg > %.3fdeg，立即ABORT' %
                    (errors[1], self.s.flip_abort_pitch_error_deg))
        for name, value, limit, unit in checks:
            if value <= limit:
                self.flip_abort_since.pop(name, None)
                continue
            since = self.flip_abort_since.setdefault(name, f.pose_stamp)
            if f.pose_stamp-since >= self.s.flip_abort_confirm_s-1e-8:
                return ('斜墙翻转%s超限：%.3f%s > %.3f%s，持续%.2fs' %
                        (name, value, unit, limit, unit, f.pose_stamp-since))
        return ''

    def _flip_pitch_still(self, f):
        """Require a full, contiguous source-time window; replays add no time."""
        stamp = f.pose_stamp
        if not math.isfinite(stamp) or stamp <= 0. or stamp > f.now+1e-8:
            self.flip_pitch_history = []
            return False
        pitch = euler_from_quaternion(f.q)[1]
        history = self.flip_pitch_history
        if history:
            gap = stamp-history[-1][0]
            if gap < 0. or gap > .20+1e-8:
                history.clear()
        if not history or stamp > history[-1][0]:
            history.append((stamp, pitch))
        cutoff = stamp-self.s.flip_still_window_s
        while len(history) > 1 and history[1][0] <= cutoff+1e-8:
            history.pop(0)
        if stamp-history[0][0] < self.s.flip_still_window_s-1e-8:
            return False
        # Unwrap before comparing the full excursion, not just the endpoints.
        angles = [history[0][1]]
        for _, angle in history[1:]:
            previous = angles[-1]
            angles.append(previous+math.atan2(math.sin(angle-previous), math.cos(angle-previous)))
        return math.degrees(max(angles)-min(angles)) < self.s.flip_still_pitch_span_deg-1e-8

    def _flip_search_complete(self, f):
        # Keep history continuously, but only use it after the >8deg gate.
        still = self._flip_pitch_still(f)
        measured = euler_from_quaternion(f.q)[1]
        sent = euler_from_quaternion(f.applied_q)[1]
        error = abs(math.degrees(math.atan2(math.sin(measured-sent), math.cos(measured-sent))))
        return error > self.s.flip_still_min_pitch_error_deg+1e-8 and still

    def _hold_from_top(self, f):
        self.contact_seen = True
        self.contact_q, self.contact_sent = f.q, f.applied_q
        self.contact_samples = []
        self.contact_angle = max(0., self.tilt(f.q)-math.radians(self.s.contact_pitch_bias_deg))
        self.required_thrust = self._required(self.contact_angle)
        self.q, self.thrust = self.contact_q, f.applied_thrust
        self._enter('HOLD', f.now, f'实际俯仰0.5s内变化小于1度且顶部单帧小于{self.s.top_contact_distance_m * 100:g}cm：锁定HOLD')

    def _flip_support(self, f, dt):
        """Entry collective boost plus takeover-height PD and tilt feedforward.

        No integral and no accumulation from delayed output echoes. The final
        publisher independently slews from its actual last publication.
        """
        # Integrate bounded control time: a stalled timer cannot skip the
        # smooth entry ramp. Boost and feedback share ONE correction budget.
        self.flip_boost_elapsed = min(self.s.flip_entry_boost_rise_s,
                                      self.flip_boost_elapsed+dt)
        self.flip_boost_value = self.s.flip_entry_boost*smoothstep5(
            self.flip_boost_elapsed/self.s.flip_entry_boost_rise_s)
        fraction = max(rotate_vector(f.q, (0., 0., 1.))[2], .001)
        correction = (self.flip_boost_value + self.base/fraction - self.flip_entry_thrust
            + self.s.flip_support_height_kp*(self.reference_height-f.height)
            - self.s.flip_support_velocity_kp*f.vz)
        self.flip_support_correction = clamp(correction,
            -self.s.flip_support_correction_max, self.s.flip_support_correction_max)
        self.flip_support_requested = min(self.cap, max(self.flip_pressure_thrust,
            self.flip_entry_thrust+self.flip_support_correction))
        step = self.s.flip_support_slew_per_s*dt
        self.thrust = clamp(self.flip_support_requested,
            max(.1, self.thrust-step), min(self.cap, self.thrust+step))

    def step(self, f, requested=True, detach=False, cancel=False):
        if self.phase not in self.ACTIVE:
            return
        delta = f.now - self.last_at
        if delta < 0.:
            self._land(f, '控制时钟倒退')
            return
        self.last_at = f.now
        dt = min(.05, delta)
        if (not f.pose_valid or not f.control_valid
                or not all(math.isfinite(v) for v in (f.height, f.vz))):
            self._land(f, '关键姿态/速度反馈或控制权失效')
            return
        if self.phase in ('HOLD', 'RETURN') and self.detach_yaw_reference is not None:
            yaw = euler_from_quaternion(f.q)[2]
            delta_yaw = yaw-self.detach_yaw_reference
            yaw_error = abs(math.atan2(math.sin(delta_yaw), math.cos(delta_yaw)))
            # Tolerate arithmetic roundoff at equality.
            if yaw_error > math.radians(DETACH_YAW_LIMIT_DEG) + 1e-9:
                self._land(f, '回平偏航偏离脱离指令基准超过%d度：基准%.3f度，实测%.3f度，偏差%.3f度' % (
                    DETACH_YAW_LIMIT_DEG, math.degrees(self.detach_yaw_reference),
                    math.degrees(yaw), math.degrees(yaw_error)))
                return
        if f.output_valid and (not math.isfinite(f.applied_thrust)
                or not .1-1e-6 <= f.applied_thrust <= self.cap+1e-6):
            # Check every phase before seeding/updating the shared support
            # controller. Invalid feedback must hand over, not raise in Timer.
            self._land(f, '最终推力反馈超出斜墙支撑范围')
            return
        if self.tilt(f.q) > math.radians(self.s.flip_max_tilt_deg) + 1e-6:
            self.abort(f, '实测倾角超过上限')
        if cancel or not requested:
            self.abort(f, '操作取消或斜墙意图释放')
        if self.phase == 'LAND':
            return
        if self.phase != 'APPROACH' and not f.output_valid:
            self._land(f, '最终发布反馈失效，禁止盲目回平或增推')
            return
        age = f.now - self.entered
        if self.phase == 'APPROACH':
            self._estimate(f)
            required = self._required(self.estimated_angle) if self.estimated_angle is not None else None
            if required is not None and required > self.cap:
                self.abort(f, '粗估坡度所需推力超过标定上限')
                return
            if age >= self.s.approach_timeout_s:
                self.abort(f, '接近超时')
            elif (f.front_valid and math.isfinite(f.front) and f.front > 0.
                    and f.front <= self.s.flip_trigger_distance_m + 1e-6 and f.output_valid):
                self.q, self.thrust = f.applied_q, f.applied_thrust
                self.flip_entry_thrust = self.flip_pressure_thrust = self.thrust
                self.flip_support_requested = self.thrust
                self.flip_support_correction = 0.
                self.flip_boost_elapsed = self.flip_boost_value = 0.
                self.flip_seed = self.q
                self.flip_angle = self.flip_rate = 0.
                self.flip_entry_height = f.height
                self.flip_abort_since.clear()
                self.flip_abort_stamp = f.pose_stamp
                self.flip_pitch_history = [(f.pose_stamp, euler_from_quaternion(f.q)[1])]
                self.flip_top_stamp = f.top_stamp
                self._enter('SLOW_FLIP', f.now, '斜墙慢速翻转并增推')
        elif self.phase == 'SLOW_FLIP':
            motion_fault = self._flip_motion_fault(f)
            if motion_fault:
                self.abort(f, motion_fault)
                return
            hard = (self.tilt(f.q) > math.radians(self.s.flip_max_tilt_deg) + 1e-6
                    or f.applied_thrust > self.cap + 1e-6)
            stop_search = self._flip_search_complete(f)
            new_top = (f.top_valid and math.isfinite(f.top_stamp)
                       and self.flip_top_stamp < f.top_stamp <= f.now)
            if new_top:
                self.flip_top_stamp = f.top_stamp
            if (stop_search and new_top and math.isfinite(f.top)
                    and 0. < f.top < self.s.top_contact_distance_m and not hard):
                self._hold_from_top(f)
                return
            limit = math.radians(self.s.flip_max_tilt_deg)
            if (hard or self.tilt(f.q) >= limit - 1e-6 or self.tilt(self.q) >= limit - 1e-6
                    or f.applied_thrust >= self.cap - 1e-6 or age >= self.s.flip_duration_timeout_s):
                self.abort(f, '翻转角度/推力上限或8秒预算耗尽，尚未接触')
                return
            if stop_search:
                # Freeze the last published target, not a new advancing target
                # or a jump back to measured attitude. HOLD still needs range.
                self.contact_q = f.q
                self.contact_sent = self.q = f.applied_q
                self.thrust = f.applied_thrust
                self.contact_top_stamp = f.top_stamp
                self._enter('CONTACT_SETTLE', f.now,
                    '俯仰偏差超过8度且0.5s内变化小于1度：冻结目标，'
                    f'等待顶部小于{self.s.top_contact_distance_m * 100:g}cm')
                return
            # Full-attitude tracking error does not pause SLOW_FLIP.
            # Motion faults, contact, tilt/thrust caps and timeout still apply.
            self._flip_support(f, dt)
            # A coarse estimate is advisory, never a contact declaration.
            # Slow the search near it; continue searching until contact/timeout.
            search_rate = self.s.flip_pitch_rate_deg_s
            if self.estimated_angle is not None and self.tilt(self.q) >= self.estimated_angle-math.radians(3.):
                search_rate *= .5
            rate = min(math.radians(search_rate),
                       self.flip_rate + math.radians(self.s.flip_pitch_accel_deg_s2)*dt)
            angle = self.flip_angle + rate*dt
            pitch = hover_and_pitched_quaternions(0., angle)[1]
            proposed = quaternion_normalize(quaternion_multiply(self.flip_seed, pitch))
            if self.tilt(proposed) > limit:
                amount = clamp((limit-self.tilt(self.q))/max(quaternion_angle(self.q, proposed), 1e-9), 0., 1.)
                proposed = quaternion_slerp(self.q, proposed, amount)
            self.flip_angle, self.flip_rate, self.q = angle, rate, proposed
            self.flip_pressure_thrust = min(self.cap,
                self.flip_pressure_thrust + self.s.flip_thrust_ramp_per_s*dt)
        elif self.phase == 'CONTACT_SETTLE':
            motion_fault = self._flip_motion_fault(f)
            if motion_fault:
                self.abort(f, motion_fault)
                return
            # Stop advancing the target and collective while awaiting contact.
            self.q = self.contact_sent
            if f.now - self.contact_top_received_at >= self.s.contact_settle_timeout_s:
                self.abort(f, f'斜面接触确认期间连续{self.s.contact_settle_timeout_s:g}秒未收到新的有效顶部雷达数据')
                return
            if (f.top_valid and math.isfinite(f.top) and f.top > 0.
                    and max(self.contact_top_stamp, self.entered) < f.top_stamp <= f.now):
                self.contact_top_stamp = f.top_stamp
                self.contact_top_received_at = f.now
                if f.top < self.s.top_contact_distance_m:
                    self._hold_from_top(f)
        elif self.phase == 'HOLD':
            # Contact is already confirmed. Range loss, distance, body rate and
            # optional friction estimates do not revoke HOLD. Common feedback
            # validity, output bounds and pilot/kill guards still apply.
            # Keep the contact-normal reference frozen, but follow measured
            # twist about that body Z axis like ordinary wall HOLD.
            locked_z = rotate_vector(self.contact_q, (0., 0., 1.))
            measured_z = rotate_vector(f.q, (0., 0., 1.))
            axis_error = math.acos(clamp(sum(a*b for a, b in zip(locked_z, measured_z)), -1., 1.))
            if axis_error > math.radians(self.s.contact_attitude_tolerance_deg) + 1e-9:
                self.abort(f, 'HOLD机体Z轴偏离接触锁定方向超过%g度' % self.s.contact_attitude_tolerance_deg)
            else:
                followed = follow_rotation_about_body_z(self.contact_q, f.q)
                self.q = followed if followed is not None else self.contact_q
                if detach:
                    self._return(f)
        elif self.phase == 'RETURN':
            if f.now-self.return_started >= self.s.detach_duration_timeout_s:
                self._land(f, '斜墙回平超时')
                return
            if self.planner is None:
                stamp = f.output_stamp
                previous = self.return_history[-1]
                if stamp > previous[0] and stamp-previous[0] <= .20:
                    angle = self.axis.angle(f.applied_q)
                    rate = (angle-self.axis.angle(previous[1]))/(stamp-previous[0])
                    # Conservative near-panel return envelope, incline only.
                    maximum = math.radians(min(self.s.detach_max_rate_deg_s, 10.)
                        if f.top_valid and f.top <= self.s.flip_trigger_distance_m
                        else self.s.detach_max_rate_deg_s)
                    if abs(rate) > maximum + 1e-6:
                        self._land(f, '回平入口实际输出角速度超限')
                        return
                    self.planner = BrakingTrajectory(angle, rate, 0.,
                        max_angle=max(angle, self.return_seed, 1e-5), max_rate=maximum,
                        max_accel=math.radians(self.s.detach_max_accel_deg_s2), max_duration=self.s.detach_duration_timeout_s)
                elif stamp > previous[0]:
                    self.return_history = [(stamp, f.applied_q)]
                self._update_return_support(f)
                return
            remaining = self.axis.angle(f.q)
            closing = self.axis.closing_rate(f.q, f.body_rate)
            goal = 0.
            if self.tilt(f.q) <= math.radians(self.s.detach_brake_target_deg):
                allowed = allowed_rate(remaining, math.radians(self.s.detach_brake_decel_deg_s2),
                    self.s.detach_brake_delay_s, math.radians(self.s.detach_brake_margin_deg), self.planner.max_rate)
                goal = braking_goal(remaining, closing, allowed, 4., 1.,
                    math.radians(self.s.detach_max_tracking_error_deg), self.planner.max_angle)
            recovered = (self.tilt(f.q) <= math.radians(self.s.detach_recovered_tilt_deg) + 1e-12
                and math.sqrt(sum(v*v for v in f.body_rate)) <= math.radians(self.s.detach_recovered_rate_deg_s) + 1e-12)
            if recovered:
                goal = 0.
            # Match wall return: permit error-reducing motion even when the
            # current full-attitude error (including yaw) exceeds the limit.
            previous_state = (self.planner.angle, self.planner.rate, self.planner.accel)
            try:
                target = self.axis.quaternion(self.planner.step(dt, goal))
            except (ValueError, RuntimeError) as error:
                self._land(f, '回平轨迹失败：' + str(error))
                return
            old_error = quaternion_angle(self.q, f.q)
            new_error = quaternion_angle(target, f.q)
            self.return_tracking_limited = bool(
                new_error > math.radians(self.s.detach_max_tracking_error_deg)+1e-6
                and new_error > old_error+1e-8)
            if self.return_tracking_limited:
                self.planner.angle, self.planner.rate, self.planner.accel = previous_state
                self.planner.segment = None
                self.planner.done = False
            else:
                self.q = target
            if self.planner.done:
                pitch = euler_from_quaternion(f.q)[1]
                if pitch < 0.:
                    self._land(f, 'detach最终俯仰角为负，请求PX4 AUTO.LAND')
                    return
                if self.tilt(f.q) < math.radians(ENTRY_TILT_DEG)-1e-12:
                    self.recovery_started = f.now
                    self.support.reanchor(f.height)
                    self.reference_height = f.height
                    self._enter('RECOVER', f.now, '斜墙回平完成，倾角小于10度，恢复悬停')
        # The ROS adapter uses the side-wall approach controller verbatim.
        # This support loop owns only incline return/recovery, never approach.
        if self.phase in ('RETURN', 'RECOVER', 'EXIT'):
            self._update_return_support(f)
        if self.phase in ('RECOVER', 'EXIT'):
            self.q = self.hover
            # Same captured-recovery contract as wall: task height and
            # velocity convergence do not gate handover. The mode manager
            # independently checks FCU estimator, RC, fusion and ownership.
            stable = attitude_within_margins(self.tilt(f.q),
                math.sqrt(sum(v*v for v in f.body_rate)))
            self.recover_stable = self._stable(stable, f.pose_stamp, f.now, STABLE_TIME_S, max_gap=.50)
            if self.recover_stable:
                if self.phase != 'EXIT':
                    self._enter('EXIT', f.now, '斜墙姿态稳定，等待交回POSCTL')
            elif self.phase == 'EXIT':
                self._enter('RECOVER', f.now, '撤回交接：重新确认姿态稳定')
