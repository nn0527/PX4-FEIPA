"""Source-timed floor range feedback. No ROS, EKF height or flow-quality gate.

OpticalFlowRad has no independent usable distance timestamp on this vehicle
(time_delta_distance_us == 0). A sample means a new *message* carrying distance,
not a proven new hardware ranging event. All times use the same ROS source clock.
"""
import math
from collections import deque
from dataclasses import dataclass

from .geometry import quaternion_normalize, quaternion_slerp, rotate_vector


@dataclass(frozen=True)
class RangeHeightConfig:
    entry_tilt_deg: float = 10.
    operating_tilt_deg: float = 60.
    timeout_s: float = .60
    pose_interp_max_span_s: float = .10
    pose_wait_s: float = .20
    velocity_window_s: float = .8
    height_only_kp: float = .15
    control_velocity_kp: float = .18
    control_blend_s: float = .30
    loss_support_s: float = 2.
    control_timeout_s: float = 15.
    handover_wait_s: float = 6.
    height_tolerance_m: float = .10
    speed_tolerance_mps: float = .20
    stable_time_s: float = 1.
    stable_min_samples: int = 4
    # No invented hardware range. Set these from the installed device limits.
    min_distance_m: float = 0.
    max_distance_m: float = 0.
    sensor_axis_body: tuple = (0., 0., -1.)
    # Vector from sensor origin to controlled reference point, in FLU body.
    reference_offset_body_m: tuple = (0., 0., 0.)
    geometry_confirmed: bool = False
    future_tolerance_s: float = .10
    velocity_min_dt_s: float = .05
    height_filter_tau_s: float = .10
    velocity_filter_tau_s: float = .25
    fast_velocity_filter_tau_s: float = .10
    fast_velocity_max_gap_s: float = .60
    jump_m: float = .25
    max_vertical_speed_mps: float = 2.
    output_sync_timeout_s: float = .30

    def __post_init__(self):
        positive = (self.entry_tilt_deg, self.operating_tilt_deg, self.timeout_s,
                    self.pose_interp_max_span_s, self.pose_wait_s,
                    self.velocity_window_s, self.height_only_kp, self.control_velocity_kp,
                    self.control_blend_s, self.loss_support_s, self.control_timeout_s,
                    self.handover_wait_s, self.height_tolerance_m, self.speed_tolerance_mps,
                    self.stable_time_s, self.velocity_min_dt_s, self.height_filter_tau_s,
                    self.velocity_filter_tau_s, self.fast_velocity_filter_tau_s,
                    self.fast_velocity_max_gap_s, self.jump_m,
                    self.max_vertical_speed_mps, self.output_sync_timeout_s)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError('range height gains/times/limits must be finite and positive')
        if not (0 < self.entry_tilt_deg <= self.operating_tilt_deg < 90
                and self.pose_interp_max_span_s < self.timeout_s
                and self.pose_wait_s < self.timeout_s
                and self.velocity_min_dt_s < self.velocity_window_s
                and self.velocity_min_dt_s < self.fast_velocity_max_gap_s <= self.timeout_s
                and self.handover_wait_s <= self.control_timeout_s
                and self.stable_min_samples >= 2 and int(self.stable_min_samples) == self.stable_min_samples
                and 0 <= self.future_tolerance_s <= .10):
            raise ValueError('inconsistent range height settings')
        if (len(self.sensor_axis_body) != 3 or len(self.reference_offset_body_m) != 3
                or not all(math.isfinite(v) for v in (*self.sensor_axis_body, *self.reference_offset_body_m))
                or abs(sum(v*v for v in self.sensor_axis_body)-1.) > .001):
            raise ValueError('range installation needs a unit body axis and finite offset')
        if not (math.isfinite(self.min_distance_m) and math.isfinite(self.max_distance_m)
                and 0 <= self.min_distance_m <= self.max_distance_m):
            raise ValueError('invalid device range limits')

    @property
    def installation_ready(self):
        return self.geometry_confirmed and self.max_distance_m > self.min_distance_m


@dataclass(frozen=True)
class RangeHeightSnapshot:
    episode: int
    target: float
    height: float
    velocity: float
    velocity_valid: bool
    valid: bool
    source: float
    received: float
    raw_distance: float
    projected_height: float
    sample_tilt_deg: float
    pose_match_dt_s: float
    sample_count: int
    age: float
    reason: str
    height_invalid_since: float
    velocity_source: float
    velocity_age: float
    velocity_invalid_since: float
    velocity_reason: str
    fast_velocity: float
    fast_velocity_valid: bool
    fast_velocity_source: float
    fast_velocity_age: float
    fast_velocity_reason: str
    pose_before_dt_s: float
    pose_after_dt_s: float
    pose_span_s: float
    pose_wait_s: float


class RangeHeightFeedback:
    def __init__(self, config=None):
        self.config = config or RangeHeightConfig()
        self.poses = deque(maxlen=512)
        self.reset()

    def reset(self):
        self.episode = 0
        self.started = None
        self.pending = deque(maxlen=128)
        self.last_message_source = None
        self.target = self.height = self.velocity = math.nan
        self.lock_source = self.lock_received = self.lock_time = None
        self.lock_raw = self.lock_height = self.lock_tilt = math.nan
        self.source = self.received = 0.
        self.raw_distance = self.projected_height = self.sample_tilt_deg = math.nan
        self.pose_match_dt_s = math.nan
        self.pose_before_dt_s = self.pose_after_dt_s = self.pose_span_s = math.nan
        self.pose_wait_s = math.nan
        self.velocity_valid = False
        self.velocity_source = None
        self.velocity_invalid_since = None
        self.velocity_reason = '测距速度窗口尚未就绪'
        self.samples = deque(maxlen=64)
        self.fast_samples = deque(maxlen=64)
        self.fast_velocity = math.nan
        self.fast_velocity_valid = False
        self.fast_velocity_source = None
        self.fast_velocity_reason = '快速速度尚未就绪'
        self.count = 0
        self.rejected = {}
        self.reason = '未开始脱离'
        self.height_invalid_since = None
        self.last_height = None
        self.last_tick = None
        self.velocity_ready_at = None
        self.target_revision = 0
        self.target_source = 'range_first_qualified_message'
        self.action_started = None

    @property
    def invalid_since(self):
        """Compatibility alias: invalid_since now means height invalidity only."""
        return self.height_invalid_since

    @invalid_since.setter
    def invalid_since(self, value):
        self.height_invalid_since = value


    def capture_recovery_target(self, now):
        """One explicit phase-boundary reanchor; never refresh while retreating."""
        s = self.read_snapshot(now)
        if not s.valid or not math.isfinite(s.projected_height) or s.projected_height <= 0:
            raise ValueError('恢复退墙等待新鲜有效的实际测距高度')
        self.target = s.projected_height
        self.target_revision += 1
        self.target_source = 'recover_entry_range_height'
        self.action_started = now
        return self.target

    @property
    def locked(self):
        return self.lock_source is not None

    def begin(self, episode, now, config=None):
        self.reset()  # Keep the pre-entry attitude buffer, discard old ranges.
        if config is not None:
            self.config = config
        self.episode, self.started = episode, now
        self.reason = '等待采样倾角和首条合格测距'

    def _reject(self, reason):
        self.rejected[reason] = self.rejected.get(reason, 0)+1
        self.reason = reason

    def observe_pose(self, source, received, quaternion):
        if (not all(math.isfinite(v) for v in (source, received, *quaternion))
                or source <= 0 or abs(sum(v*v for v in quaternion)-1.) > .01
                or (self.poses and source <= self.poses[-1][0])):
            return
        self.poses.append((source, received, quaternion_normalize(quaternion)))
        self.process(received)

    def observe_range(self, source, received, distance):
        if self.started is None:
            return
        c = self.config
        if (not all(math.isfinite(v) for v in (source, received)) or source <= self.started
                or received < self.started or not -c.future_tolerance_s <= received-source <= c.timeout_s):
            self._reject('测距源时间过期、未来或不属于本轮')
            return
        if self.last_message_source is not None and source <= self.last_message_source:
            self._reject('测距时间重复或倒退')
            return
        self.last_message_source = source
        if not c.installation_ready:
            self._reject('待核对测距量程、下视安装轴及上游几何修正')
            return
        if not math.isfinite(distance) or distance <= 0 or not c.min_distance_m <= distance <= c.max_distance_m:
            self._reject('测距无效或超出设备量程')
            return
        if len(self.pending) == self.pending.maxlen:
            # Never overwrite the first pending candidate with a later one.
            self._reject('等待首帧姿态时测距队列已满')
            return
        self.pending.append((source, received, distance))
        self.process(received)

    def _pose_at(self, source, now, allow_nearest=False):
        c = self.config
        poses = [p for p in self.poses if 0 <= now-p[1] <= c.timeout_s
                 and -c.future_tolerance_s <= now-p[0] <= c.timeout_s]
        before = next((p for p in reversed(poses) if p[0] <= source), None)
        after = next((p for p in poses if p[0] >= source), None)
        if before and after:
            span = after[0]-before[0]
            if span <= c.pose_interp_max_span_s+1.e-9:
                left, right = source-before[0], after[0]-source
                q = before[2] if span == 0 else quaternion_slerp(before[2], after[2], left/span)
                return q, left, right, span
        # A nearby source-timed pose is enough for a demo. Requiring two
        # bracketing messages made ordinary asynchronous arrivals reject data.
        if allow_nearest and poses:
            nearest = min(poses, key=lambda p: abs(p[0]-source))
            distance = abs(nearest[0]-source)
            if distance <= c.pose_interp_max_span_s+1.e-9:
                return nearest[2], max(0., source-nearest[0]), max(0., nearest[0]-source), distance
        return None

    def _invalidate_velocity(self, now, reason, reset_history=False, invalidate_fast=True):
        if reset_history:
            self.samples.clear()
            self.fast_samples.clear()
        self.velocity_valid = False
        self.velocity = math.nan
        self.velocity_ready_at = None
        self.velocity_source = None
        if self.velocity_invalid_since is None:
            self.velocity_invalid_since = now
        self.velocity_reason = reason
        if invalidate_fast:
            self.fast_velocity_valid = False
            self.fast_velocity = math.nan
            self.fast_velocity_source = None
            self.fast_velocity_reason = reason

    def _invalidate_height(self, now, reason, reset_history=True):
        if self.height_invalid_since is None:
            self.height_invalid_since = now
        self._invalidate_velocity(now, reason, reset_history=reset_history)
        self.reason = reason

    def process(self, now):
        if self.started is None:
            return
        c = self.config
        if self.last_tick is not None and now < self.last_tick:
            self._invalidate_height(now, '测距控制时钟倒退')
            self.pending.clear()
            return
        self.last_tick = now
        while self.pending:
            source, received, distance = self.pending[0]
            if now-source > c.timeout_s or now-received > c.timeout_s:
                self.pending.popleft()
                self._reject('等待姿态期间测距过期')
                continue
            matched = self._pose_at(source, now, allow_nearest=now-received >= c.pose_wait_s-1.e-9)
            if matched is None:
                if now-received <= c.pose_wait_s:
                    break
                self.pending.popleft()
                self._reject('采样时刻姿态缺失或匹配超时')
                continue
            self.pending.popleft()
            q, before_dt, after_dt, pose_span = matched
            body_up = rotate_vector(q, (0., 0., 1.))[2]
            tilt = math.degrees(math.acos(max(-1., min(1., body_up))))
            projection = -rotate_vector(q, c.sensor_axis_body)[2]
            sensor_tilt = math.degrees(math.acos(max(-1., min(1., projection))))
            limit = c.operating_tilt_deg if self.locked else c.entry_tilt_deg
            if max(tilt, sensor_tilt) > limit+1.e-9 or projection <= 0:
                self._reject('采样倾角超出测距工作门' if self.locked else '采样倾角尚未进入首帧门')
                continue
            height = distance*projection + rotate_vector(q, c.reference_offset_body_m)[2]
            if not math.isfinite(height) or height <= 0:
                self._reject('测距几何高度无效')
                continue
            if self.locked and self.last_height is not None:
                gap = source-self.source
                if gap <= 0:
                    self._reject('测距时间重复或倒退')
                    continue
                if abs(height-self.last_height) > c.jump_m+c.max_vertical_speed_mps*gap:
                    self._reject('测距高度异常跳点')
                    continue
            first = not self.locked
            if first:
                self.target = height  # Deterministic geometry, never a delayed average.
                self.lock_source, self.lock_received, self.lock_time = source, received, now
                self.action_started = now
                self.lock_raw, self.lock_height, self.lock_tilt = distance, height, tilt
            gap = source-self.source
            reset_history = bool(not first and (gap > c.timeout_s or self.height_invalid_since is not None))
            if reset_history:
                self.samples.clear()
                self.fast_samples.clear()
                self._invalidate_velocity(now, '测距高度中断后速度重新建窗', reset_history=True)
            self.height = (height if first or not math.isfinite(self.height) else self.height+
                           (1.-math.exp(-gap/c.height_filter_tau_s))*(height-self.height))
            self.last_height = height
            self.source, self.received = source, received
            self.raw_distance, self.projected_height = distance, height
            self.sample_tilt_deg = tilt
            self.pose_before_dt_s, self.pose_after_dt_s = before_dt, after_dt
            self.pose_span_s, self.pose_match_dt_s = pose_span, pose_span
            self.pose_wait_s = max(0., now-received)
            self.count += 1

            # Accumulate short frame intervals instead of repeatedly discarding
            # velocity when the sensor publishes faster than velocity_min_dt_s.
            fast_pair = next((p for p in reversed(self.fast_samples)
                              if source-p[0] >= c.velocity_min_dt_s-1.e-9), None)
            if fast_pair is not None:
                fast_source, fast_height = fast_pair
                fast_dt = source-fast_source
                if c.velocity_min_dt_s-1.e-9 <= fast_dt <= c.fast_velocity_max_gap_s:
                    raw_fast = (height-fast_height)/fast_dt
                    previous_fast = self.fast_velocity
                    filtered_fast = (raw_fast if not math.isfinite(previous_fast) else previous_fast+
                        (1.-math.exp(-fast_dt/c.fast_velocity_filter_tau_s))*(raw_fast-previous_fast))
                    if abs(raw_fast) <= c.max_vertical_speed_mps and abs(filtered_fast) <= c.max_vertical_speed_mps:
                        self.fast_velocity = filtered_fast
                        self.fast_velocity_valid = True
                        self.fast_velocity_source = source
                        self.fast_velocity_reason = '快速相邻高度速度有效'
                    else:
                        self.fast_velocity = math.nan
                        self.fast_velocity_valid = False
                        self.fast_velocity_source = None
                        self.fast_velocity_reason = '快速速度超出数值可信范围'
                else:
                    self.fast_velocity = math.nan
                    self.fast_velocity_valid = False
                    self.fast_velocity_source = None
                    self.fast_velocity_reason = '快速速度帧间隔不连续'
            self.fast_samples.append((source, height))

            self.samples.append((source, height))
            while len(self.samples) > 2 and self.samples[0][0] < source-c.velocity_window_s:
                self.samples.popleft()
            span = self.samples[-1][0]-self.samples[0][0]
            if len(self.samples) >= 2 and span >= c.velocity_min_dt_s:
                mean_t = sum(t-self.samples[0][0] for t, _ in self.samples)/len(self.samples)
                mean_h = sum(h for _, h in self.samples)/len(self.samples)
                numerator = sum((t-self.samples[0][0]-mean_t)*(h-mean_h) for t, h in self.samples)
                denominator = sum((t-self.samples[0][0]-mean_t)**2 for t, _ in self.samples)
                slope = numerator/denominator
                if abs(slope) > c.max_vertical_speed_mps:
                    self._reject('测距斜率超出可信速度范围')
                    self._invalidate_velocity(now, '测距斜率超出可信速度范围', invalidate_fast=False)
                else:
                    self.velocity = (slope if not self.velocity_valid else self.velocity+
                                     (1.-math.exp(-gap/c.velocity_filter_tau_s))*(slope-self.velocity))
                    if not self.velocity_valid:
                        self.velocity_ready_at = now
                    self.velocity_valid = True
                    self.velocity_source = source
                    self.velocity_invalid_since = None
                    self.velocity_reason = '慢速回归速度有效'
            elif not self.velocity_valid:
                if self.velocity_invalid_since is None:
                    self.velocity_invalid_since = now
                self.velocity_reason = '测距速度窗口尚未就绪'
            self.height_invalid_since = None
            self.reason = '测距高度/速度有效' if self.velocity_valid else '测距高度有效；'+self.velocity_reason
        if self.locked and (now-self.source > c.timeout_s or now-self.received > c.timeout_s):
            expired_at = min(self.source, self.received)+c.timeout_s
            self._invalidate_height(expired_at, '测距高度反馈过期')

    def snapshot(self, now):
        self.process(now)
        return self.read_snapshot(now)

    def read_snapshot(self, now):
        """Read feedback without processing pending frames or changing clocks."""
        age = max(now-self.source, now-self.received) if self.locked else math.inf
        valid = bool(self.locked and self.height_invalid_since is None
                     and -self.config.future_tolerance_s <= now-self.source <= self.config.timeout_s
                     and 0 <= now-self.received <= self.config.timeout_s)
        velocity_age = (now-self.velocity_source if self.velocity_source is not None else math.inf)
        velocity_valid = bool(valid and self.velocity_valid and self.velocity_source is not None
                              and -self.config.future_tolerance_s <= velocity_age <= self.config.timeout_s)
        fast_age = (now-self.fast_velocity_source if self.fast_velocity_source is not None else math.inf)
        fast_valid = bool(valid and self.fast_velocity_valid and self.fast_velocity_source is not None
                          and -self.config.future_tolerance_s <= fast_age <= self.config.timeout_s)
        return RangeHeightSnapshot(
            episode=self.episode, target=self.target, height=self.height,
            velocity=self.velocity if velocity_valid else math.nan,
            velocity_valid=velocity_valid, valid=valid, source=self.source,
            received=self.received, raw_distance=self.raw_distance,
            projected_height=self.projected_height, sample_tilt_deg=self.sample_tilt_deg,
            pose_match_dt_s=self.pose_match_dt_s, sample_count=len(self.samples),
            age=age, reason=self.reason,
            height_invalid_since=(self.height_invalid_since if self.height_invalid_since is not None else math.nan),
            velocity_source=(self.velocity_source if self.velocity_source is not None else 0.),
            velocity_age=velocity_age,
            velocity_invalid_since=(self.velocity_invalid_since if self.velocity_invalid_since is not None else math.nan),
            velocity_reason=self.velocity_reason,
            fast_velocity=self.fast_velocity if fast_valid else math.nan,
            fast_velocity_valid=fast_valid,
            fast_velocity_source=(self.fast_velocity_source if self.fast_velocity_source is not None else 0.),
            fast_velocity_age=fast_age,
            fast_velocity_reason=self.fast_velocity_reason,
            pose_before_dt_s=self.pose_before_dt_s,
            pose_after_dt_s=self.pose_after_dt_s,
            pose_span_s=self.pose_span_s,
            pose_wait_s=self.pose_wait_s)
