"""Incline contact/return state machine. No ROS or motor allocation here.

Angles are radians internally; q rotates body FLU into world ENU. All timers
consume wall time, while motion steps are bounded and evidence is source-timed.
"""
from dataclasses import dataclass
import math
import statistics

from .geometry import (clamp, euler_from_quaternion, hover_and_pitched_quaternions,
                       quaternion_angle, quaternion_multiply, quaternion_normalize,
                       quaternion_slerp, rotate_vector, smoothstep5)
from .detach_braking import BrakingTrajectory, ReturnAxis, allowed_rate, braking_goal
from .departure_height import DepartureHeightController


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
    front_ready_distance_m: float = 2.00
    flip_trigger_distance_m: float = .18
    top_contact_distance_m: float = .065
    front_ready_hold_s: float = .40
    approach_timeout_s: float = 20.
    horizontal_approach_speed_mps: float = .10
    flip_max_tilt_deg: float = 65.
    flip_pitch_rate_deg_s: float = 10.
    flip_pitch_accel_deg_s2: float = 10.
    flip_max_tracking_error_deg: float = 10.
    flip_duration_timeout_s: float = 8.
    flip_thrust_ramp_per_s: float = .03
    flip_thrust_max: float = 0.  # Required calibration, never inherit wall cap.
    top_contact_hold_s: float = .20
    contact_settle_timeout_s: float = 1.
    contact_attitude_tolerance_deg: float = 5.
    contact_max_rate_deg_s: float = 10.
    hold_entry_blend_s: float = .50
    hold_contact_loss_margin_m: float = .015
    hold_contact_loss_hold_s: float = .30
    detach_max_rate_deg_s: float = 30.
    detach_max_accel_deg_s2: float = 30.
    detach_max_tracking_error_deg: float = 10.
    detach_duration_timeout_s: float = 10.
    detach_brake_target_deg: float = 13.
    detach_brake_decel_deg_s2: float = 30.
    detach_brake_delay_s: float = .15
    detach_brake_margin_deg: float = 5.
    detach_recovered_tilt_deg: float = 5.
    detach_recovered_rate_deg_s: float = 10.
    detach_recovered_hold_s: float = .30
    return_thrust_slew_per_s: float = .20
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
            if key not in ('flip_thrust_max', 'friction_coefficient', 'contact_pitch_bias_deg') and value <= 0:
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
        if not (self.detach_recovered_tilt_deg <= self.detach_brake_margin_deg
                < self.detach_brake_target_deg < self.flip_max_tilt_deg < 90.):
            return 'invalid incline braking angles'
        if self.top_contact_hold_s >= self.contact_settle_timeout_s:
            return 'contact timeout must exceed confirmation window'
        if self.flip_pitch_rate_deg_s > 10. or self.flip_max_tracking_error_deg > 10.:
            return 'incline flip rate/tracking exceeds initial envelope'
        if (self.flip_thrust_ramp_per_s > .03 or self.detach_max_tracking_error_deg > 10.
                or self.detach_max_rate_deg_s > 30. or self.detach_recovered_tilt_deg > 5.):
            return 'incline motion limits exceed initial envelope'
        if self.flip_duration_timeout_s <= self.flip_max_tilt_deg / self.flip_pitch_rate_deg_s:
            return 'incline flip timeout too short for configured rate'
        if self.detach_recovered_hold_s >= self.detach_duration_timeout_s or self.recovery_hold_s >= self.recover_timeout_s:
            return 'incline recovery timeout too short'
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
        self.recovery_started = None
        self.return_history = []
        self.estimate_samples = []
        self.estimated_angle = None
        self.contact_samples = []
        self.contact_angle = None
        self.required_thrust = None

    @staticmethod
    def tilt(q):
        return math.acos(clamp(rotate_vector(q, (0., 0., 1.))[2], -1., 1.))

    def _enter(self, phase, now, reason):
        self.phase, self.entered, self.reason = phase, now, reason
        self.stable_since = self.stable_stamp = None

    def _stable(self, condition, stamp, now, duration):
        if not condition or stamp <= 0. or stamp > now:
            self.stable_since = self.stable_stamp = None
            return False
        if self.stable_stamp is None:
            self.stable_since = self.stable_stamp = stamp
        elif stamp > self.stable_stamp:
            if stamp - self.stable_stamp > .20:
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

    def _return(self, f):
        if not (f.pose_valid and f.control_valid and f.output_valid):
            self._land(f, '回平必要反馈或控制权失效')
            return
        self.q, self.thrust = f.applied_q, f.applied_thrust
        if not .1 <= self.thrust <= self.cap + 1e-6:
            self._land(f, '最终推力超出斜墙支撑上限')
            return
        self.return_started = f.now
        self.return_history = [(f.output_stamp, self.q)]
        self.axis = ReturnAxis(self.hover, self.q)
        self.return_seed = self.axis.angle(self.q)
        # Two actual publication samples are required before planning velocity.
        self.planner = None
        self.reference_height = f.height
        self.support.begin(f.height, self.base, self.thrust, f.now)
        self._enter('RETURN', f.now, self.reason if self.fault else '斜墙受控脱离回平')

    def step(self, f, requested=True, detach=False, cancel=False):
        if self.phase not in self.ACTIVE:
            return
        delta = f.now - self.last_at
        if delta < 0.:
            self._land(f, '控制时钟倒退')
            return
        self.last_at = f.now
        dt = min(.05, delta)
        if not f.pose_valid or not f.control_valid:
            self._land(f, '关键姿态/速度反馈或控制权失效')
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
                self.flip_seed = self.q
                self.flip_angle = self.flip_rate = 0.
                self._enter('SLOW_FLIP', f.now, '斜墙慢速翻转并增推')
        elif self.phase == 'SLOW_FLIP':
            hard = (self.tilt(f.q) > math.radians(self.s.flip_max_tilt_deg) + 1e-6
                    or f.applied_thrust > self.cap + 1e-6)
            if f.top_valid and f.top <= self.s.top_contact_distance_m and not hard:
                self.contact_seen = True
                self.contact_q, self.contact_sent = f.q, f.applied_q
                self.contact_samples = []
                self.q, self.thrust = f.applied_q, f.applied_thrust
                self._enter('CONTACT_SETTLE', f.now, '顶部6.5cm压实：停止翻转与增推')
                return
            limit = math.radians(self.s.flip_max_tilt_deg)
            if (hard or self.tilt(f.q) >= limit - 1e-6 or self.tilt(self.q) >= limit - 1e-6
                    or f.applied_thrust >= self.cap - 1e-6 or age >= self.s.flip_duration_timeout_s):
                self.abort(f, '翻转角度/推力上限或8秒预算耗尽，尚未接触')
                return
            tracking = math.radians(self.s.flip_max_tracking_error_deg)
            if quaternion_angle(self.q, f.q) > tracking:
                return  # Wall clock still advances; neither motion nor thrust catches up.
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
            if quaternion_angle(proposed, f.q) <= tracking:
                self.flip_angle, self.flip_rate, self.q = angle, rate, proposed
                self.thrust = min(self.cap, f.applied_thrust + self.s.flip_thrust_ramp_per_s*dt)
        elif self.phase == 'CONTACT_SETTLE':
            self.q = quaternion_slerp(self.contact_sent, self.contact_q,
                smoothstep5(age/self.s.hold_entry_blend_s))
            if not f.top_valid:
                # Range interruption is diagnostic only. Pause contact
                # confirmation and its timeout; never confirm from stale range.
                self.entered += delta
                self.contact_samples = []
                self.stable_since = self.stable_stamp = None
                return
            stable = (f.top_valid and f.top <= self.s.top_contact_distance_m
                and quaternion_angle(f.q, self.contact_q) <= math.radians(self.s.contact_attitude_tolerance_deg)
                and math.sqrt(sum(v*v for v in f.body_rate)) <= math.radians(self.s.contact_max_rate_deg_s))
            stamp = min(f.pose_stamp, f.top_stamp)
            if not stable:
                self.contact_samples = []
            elif not self.contact_samples or stamp > self.contact_samples[-1][0]:
                self.contact_samples.append((stamp, self.tilt(f.q)))
            if age >= self.s.hold_entry_blend_s and self._stable(stable, min(f.pose_stamp, f.top_stamp), f.now, self.s.top_contact_hold_s):
                self.contact_angle = max(0., statistics.median(v[1] for v in self.contact_samples) - math.radians(self.s.contact_pitch_bias_deg))
                self.required_thrust = self._required(self.contact_angle)
                if self.required_thrust is not None and self.thrust < self.required_thrust:
                    self.abort(f, '实际接触坡度所需支撑推力不足')
                    return
                self.q = self.contact_q
                self._enter('HOLD', f.now, '斜墙HOLD：锁定贴合姿态及总推力基准')
            elif age >= self.s.contact_settle_timeout_s:
                self.abort(f, '接触稳定确认超时')
        elif self.phase == 'HOLD':
            # Contact is already confirmed. Range loss, distance, body rate and
            # optional friction estimates do not revoke HOLD. Common feedback
            # validity, output bounds and pilot/kill guards still apply.
            if quaternion_angle(f.q, self.contact_q) > math.radians(self.s.contact_attitude_tolerance_deg):
                self.abort(f, 'HOLD姿态偏离接触锁定姿态超过5度')
            elif detach:
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
                return
            remaining = self.axis.angle(f.q)
            closing = self.axis.closing_rate(f.q, f.body_rate)
            goal = 0.
            if self.tilt(f.q) <= math.radians(self.s.detach_brake_target_deg):
                allowed = allowed_rate(remaining, math.radians(self.s.detach_brake_decel_deg_s2),
                    self.s.detach_brake_delay_s, math.radians(self.s.detach_brake_margin_deg), self.planner.max_rate)
                goal = braking_goal(remaining, closing, allowed, 4., 1.,
                    math.radians(self.s.detach_max_tracking_error_deg), self.planner.max_angle)
            recovered = (self.tilt(f.q) <= math.radians(self.s.detach_recovered_tilt_deg)
                and math.sqrt(sum(v*v for v in f.body_rate)) <= math.radians(self.s.detach_recovered_rate_deg_s))
            if recovered:
                goal = 0.
            if quaternion_angle(self.q, f.q) <= math.radians(self.s.detach_max_tracking_error_deg):
                try:
                    self.q = self.axis.quaternion(self.planner.step(dt, goal))
                except (ValueError, RuntimeError) as error:
                    self._land(f, '回平轨迹失败：' + str(error))
                    return
            self.thrust = self.support.update_range(f.height, f.vz, rotate_vector(f.q, (0., 0., 1.))[2],
                f.now, f.applied_thrust, self.s.height_kp, self.s.velocity_kp)
            if self._stable(recovered and self.planner.done, f.pose_stamp, f.now, self.s.detach_recovered_hold_s):
                self.recovery_started = f.now
                self._enter('RECOVER', f.now, '斜墙回平完成，恢复悬停')
        if self.phase in ('APPROACH', 'RECOVER', 'EXIT'):
            actual = f.applied_thrust if f.output_valid else self.thrust
            self.thrust = self.support.update_range(f.height, f.vz, rotate_vector(f.q, (0., 0., 1.))[2],
                f.now, actual, self.s.height_kp, self.s.velocity_kp)
        if self.phase in ('RECOVER', 'EXIT'):
            self.q = self.hover
            stable = (self.tilt(f.q) <= math.radians(self.s.detach_recovered_tilt_deg)
                and math.sqrt(sum(v*v for v in f.body_rate)) <= math.radians(self.s.detach_recovered_rate_deg_s)
                and max(abs(f.vz), f.horizontal_speed) <= self.s.recovery_max_speed_mps
                and abs(f.height-self.reference_height) <= self.s.recovery_max_height_error_m)
            if f.now-self.recovery_started >= self.s.recover_timeout_s:
                self._land(f, '恢复悬停超时')
            elif self._stable(stable, f.pose_stamp, f.now, self.s.recovery_hold_s):
                if self.phase != 'EXIT':
                    self._enter('EXIT', f.now, '斜墙故障回平完成，等待交回POSCTL' if self.fault else '斜墙脱离完成，等待交回POSCTL')
            elif self.phase == 'EXIT' and not stable:
                self._enter('RECOVER', f.now, '撤回交接：重新确认稳定')
