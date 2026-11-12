"""Independent incline approach, copied from the side-wall approach behavior.

ROS sensor/output feedback is shared transport. Targets, equilibrium, PI history,
height reference and approach parameters belong to incline mode alone.
"""
import math
from contextlib import nullcontext
import rospy
from mavros_msgs.msg import AttitudeTarget, State
from px4_wall_ceiling_control.domain.geometry import euler_from_quaternion, rotate_vector
from px4_wall_ceiling_control.domain.horizontal_equilibrium import HorizontalEquilibrium
from px4_wall_ceiling_control.domain.horizontal_velocity import HorizontalVelocityController
from px4_wall_ceiling_control.domain.recovery_attitude_limits import attitude_from_axis_yaw
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C


class InclineApproachMixin:
    def _init_incline_approach(self):
        defaults = dict(velocity_kp=1.5, velocity_ki=.30, integral_limit_mps2=.20,
            max_acceleration_mps2=.30, target_acceleration_mps2=.10,
            equilibrium_window_s=.5, equilibrium_speed_mps=.20,
            height_to_speed_kp=.50, vertical_velocity_kp=.30,
            max_up_speed_mps=.10, max_down_speed_mps=.15,
            positive_compensation_max=.08, max_thrust=.60,
            max_down_velocity_mps=.80, max_rate_rps=3.,
            abort_front_distance_m=1.8, source_timeout_s=.35,
            max_tilt_deg=10., ignore_range_loss=True)
        self.incline_approach_params = p = {
            k: rospy.get_param('/incline_attachment/approach/' + k, v) for k, v in defaults.items()}
        for key, value in p.items():
            if key == 'ignore_range_loss':
                if type(value) is not bool:
                    raise ValueError('invalid incline approach boolean')
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid incline approach parameter: ' + key)
        if not .1 < p['max_thrust'] <= self.max_thrust or p['max_tilt_deg'] > 10.:
            raise ValueError('incline approach exceeds output envelope')
        self.incline_approach_controller = HorizontalVelocityController(
            p['velocity_kp'], p['max_acceleration_mps2'], p['target_acceleration_mps2'],
            p['velocity_ki'], p['integral_limit_mps2'])
        self.incline_approach_equilibrium = HorizontalEquilibrium(window=p['equilibrium_window_s'], max_tilt_deg=8.)
        self.incline_approach_yaw = None
        self.incline_approach_source = None
        self.incline_approach_reference = self.incline_approach_base = self.incline_approach_takeover = None
        self.incline_approach_prestream = self.incline_approach_candidate_axis = None
        self.incline_approach_sample = None
        self.incline_approach_active = False
        self.incline_approach_reason = '等待斜面独立接近平衡基准'
        self.incline_approach_integral_reason = '预发送不积分'
        self.incline_approach_target_speed = self.incline_approach_correction = math.nan
        self.incline_approach_last_mode = ''
        rospy.Subscriber('/mavros/setpoint_raw/target_attitude', AttitudeTarget,
            self._incline_approach_on_target, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self._incline_approach_on_state, queue_size=10)

    def _incline_approach_on_state(self, message):
        with getattr(self, 'feedback_lock', nullcontext()):
            previous = self.incline_approach_last_mode
            self.incline_approach_last_mode = message.mode.upper()
            if not message.armed:
                self.incline_approach_reference = self.incline_approach_base = self.incline_approach_takeover = None
            elif (message.connected and previous == 'POSCTL' and message.mode.upper() == 'OFFBOARD'
                    and getattr(self, 'incline_selected', False)):
                # Capture once at the OFFBOARD edge, not later when a range
                # gate opens. OFFBOARD echoes can never become the baseline.
                now = rospy.Time.now()
                if is_fresh_source(message.header.stamp, self.state_timeout_s, now):
                    self._incline_approach_begin(self._incline_feedback_at(now))

    def _incline_approach_on_target(self, message):
        flight = getattr(self, 'flight_state', None)
        if flight is not None and flight.mode.upper() == 'POSCTL':
            self.incline_approach_source = (message, rospy.Time.now())

    def _incline_approach_prepare(self, now):
        tracker = self.incline_approach_equilibrium
        flight = self.flight_state
        if tracker.frozen and flight is not None and (not flight.armed or (
                flight.mode.upper() == 'POSCTL' and self.manager_phase in ('IDLE', 'LOCKED')
                and not self._incline_requested(now))):
            tracker.clear()
            self.incline_approach_yaw = None
            self.incline_approach_controller.reset()
            self.incline_approach_sample = None
        if tracker.frozen:
            return
        phase_fresh = is_fresh_receipt(self.manager_phase_received_at, .5, now)
        if self._incline_requested(now) and phase_fresh and self.manager_phase in ('ENTERING', 'ACTIVE'):
            if self.horizontal_velocity_ready and self._odom_fresh(now) and tracker.freeze(now.to_sec()):
                self.incline_approach_yaw = euler_from_quaternion(self._current_quaternion())[2]
            else:
                tracker.clear()
            return
        q = self._current_quaternion()
        source = self.incline_approach_source
        eligible = bool(flight is not None and flight.connected and flight.armed and flight.mode.upper() == 'POSCTL'
            and is_fresh_receipt(self.flight_state_received_at, 1.5, now)
            and self.horizontal_velocity_ready and q is not None and self._odom_fresh(now)
            and math.hypot(*self.horizontal_measured) <= self.incline_approach_params['equilibrium_speed_mps']
            and source is not None and is_fresh_receipt(source[1], .35, now))
        target = (0., 0., 0., 1.)
        if eligible:
            angular = self.odom.twist.twist.angular
            target = tuple(getattr(source[0].orientation, k) for k in 'xyzw')
            eligible = bool(math.isfinite(angular.y) and abs(angular.y) <= math.radians(20.)
                and rotate_vector(q, (0., 0., 1.))[2] >= math.cos(math.radians(8.))
                and all(math.isfinite(v) for v in target) and sum(v*v for v in target) > .5
                and not source[0].type_mask & AttitudeTarget.IGNORE_ATTITUDE)
            if eligible:
                eligible = sum(a*b for a, b in zip(rotate_vector(q, (0., 0., 1.)),
                    rotate_vector(target, (0., 0., 1.)))) >= math.cos(math.radians(5.))
        tracker.observe(source[1].to_sec() if source else 0., target, eligible)

    def _incline_approach_begin(self, f):
        if (self.incline_approach_reference is not None and self.incline_approach_base is not None
                and self.incline_approach_takeover is not None):
            return True
        if not f.pose_valid:
            return False
        source = self.incline_approach_source
        now = rospy.Time.from_sec(f.now)
        if source is None:
            return False
        target, received = source
        if (not is_fresh_receipt(received, self.incline_approach_params['source_timeout_s'], now)
                or not is_fresh_source(target.header.stamp, self.incline_approach_params['source_timeout_s'], now)
                or target.type_mask & AttitudeTarget.IGNORE_THRUST
                or not math.isfinite(target.thrust) or not .1 <= target.thrust <= 1.):
            return False
        self.incline_approach_takeover = float(target.thrust)
        self.incline_approach_base = target.thrust * rotate_vector(f.q, (0., 0., 1.))[2]
        self.incline_approach_reference = f.height
        self.incline_approach_sample = None
        return True

    def _incline_approach_ranges_ready(self, now):
        return bool(self._sensor_health_fresh(now) and self.sensor_health.front_valid and self._front_fresh(now))

    def _incline_approach_motion_allowed(self, now):
        return self.incline_approach_params['ignore_range_loss'] or self._incline_approach_ranges_ready(now)

    def _incline_approach_failure(self, f, elapsed, require_control=True):
        p = self.incline_approach_params
        if f.front_valid and self._incline_approach_ranges_ready(rospy.Time.from_sec(f.now)) and f.front > p['abort_front_distance_m']:
            return '斜面接近前距超过%.3fm' % p['abort_front_distance_m']
        if not f.pose_valid or (require_control and not f.control_valid):
            return '关键姿态/速度反馈或控制权失效'
        if any(not math.isfinite(v) or abs(v) > p['max_rate_rps'] for v in f.body_rate):
            return '斜面接近角速度超限'
        if not math.isfinite(f.vz) or f.vz < -p['max_down_velocity_mps']:
            return '斜面接近下降速度超限'
        if elapsed >= self.incline_settings.approach_timeout_s:
            return '斜面自动接近超时'
        return ''

    def _incline_approach_integral_interval(self, now):
        c = self.incline_approach_controller
        self.incline_approach_applied_axis = None
        if not (self._incline_requested(now) and self._auto_takeover_confirmed(now)
                and self.incline_approach_equilibrium.frozen and self._incline_approach_motion_allowed(now)
                and self.horizontal_output_owner == C.SOURCE_WALL
                and is_fresh_receipt(self.horizontal_output_owner_at, .3, now)
                and is_fresh_receipt(self.horizontal_output_at, .3, now)):
            c.clear_integral(); self.incline_approach_sample = None
            self.incline_approach_integral_reason = '等待本次斜面输出权限'
            return 0.
        output = self.horizontal_output
        q = tuple(getattr(output.orientation, k) for k in 'xyzw')
        clock = max(now.to_sec(), rospy.Time.now().to_sec())
        if (not all(math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1.) > .01
                or output.type_mask & AttitudeTarget.IGNORE_ATTITUDE
                or not 0 <= clock-output.header.stamp.to_sec() <= .3):
            c.clear_integral(); self.incline_approach_sample = None
            return 0.
        axis = rotate_vector(q, (0., 0., 1.))
        self.incline_approach_applied_axis = axis
        candidate = self.incline_approach_candidate_axis
        if candidate is None or sum(a*b for a,b in zip(axis, candidate)) < math.cos(math.radians(1.)):
            self.incline_approach_sample = None
            c.integral_active = False
            self.incline_approach_integral_reason = '等待斜面候选姿态输出确认'
            return 0.
        stamp = self.odom.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0 or not -.1 <= clock-stamp <= self.odom_timeout_s:
            c.clear_integral(); self.incline_approach_sample = None
            return 0.
        previous = self.incline_approach_sample
        c.integral_active = False
        if previous is not None and stamp <= previous:
            return 0.
        self.incline_approach_sample = stamp
        if previous is None or stamp-previous > self.odom_timeout_s:
            return 0.
        return min(.10, stamp-previous)

    def _incline_approach_attitude(self, now, prestream=False):
        tracker = self.incline_approach_equilibrium
        if not tracker.ready:
            return None
        yaw = self.incline_approach_yaw
        if yaw is None:
            yaw = euler_from_quaternion(self._current_quaternion())[2]
        baseline = attitude_from_axis_yaw(tracker.axis, yaw)
        c = self.incline_approach_controller
        self.incline_approach_active = False
        if not self.horizontal_velocity_ready:
            c.reset(); self.incline_approach_sample = None
            self.incline_approach_reason = self.horizontal_velocity_reason
            return baseline
        forward = self.incline_settings.horizontal_approach_speed_mps if not prestream and self._incline_approach_motion_allowed(now) else 0.
        interval = 0. if prestream else self._incline_approach_integral_interval(now)
        if prestream:
            c.clear_integral()
        q = c.attitude((forward*math.cos(yaw), forward*math.sin(yaw)), self.horizontal_measured,
            yaw, self.control_dt, tracker.axis, integrate=interval > 0., integral_dt=interval,
            max_tilt=None if prestream else math.radians(self.incline_approach_params['max_tilt_deg']),
            applied_axis=None if prestream else self.incline_approach_applied_axis)
        if not prestream:
            self.incline_approach_candidate_axis = rotate_vector(q, (0., 0., 1.))
            self.incline_approach_active = True
            self.incline_approach_reason = '斜面水平速度接近：目标前向%.2fm/s、横向0' % forward
        return q

    def _incline_approach_control(self, now, f):
        p = self.incline_approach_params
        q = self._incline_approach_attitude(now)
        if self.incline_approach_reference is None or self.incline_approach_base is None:
            return q, math.nan
        target = max(-p['max_down_speed_mps'], min(p['max_up_speed_mps'],
            p['height_to_speed_kp']*(self.incline_approach_reference-f.height)))
        correction = min(p['positive_compensation_max'], p['vertical_velocity_kp']*(target-f.vz))
        self.incline_approach_target_speed, self.incline_approach_correction = target, correction
        fraction = max(0., rotate_vector(f.q, (0., 0., 1.))[2])
        thrust = max(.1, min(self.max_thrust, self.incline.cap, p['max_thrust'],
            max(0., self.incline_approach_base+correction)/max(fraction, 1.e-3)))
        return q, thrust

    def _fill_incline_approach_status(self, status):
        tracker, c = self.incline_approach_equilibrium, self.incline_approach_controller
        now = status.header.stamp
        self.incline_approach_prestream = None
        if self.incline.phase == 'WAIT' and self._incline_requested(now) and tracker.ready:
            self.incline_approach_prestream = self._incline_approach_attitude(now, prestream=True)
        status.horizontal_velocity_enabled = True
        status.horizontal_equilibrium_enabled = True
        status.horizontal_equilibrium_valid, status.horizontal_equilibrium_frozen = tracker.ready, tracker.frozen
        status.horizontal_equilibrium_reason = ('斜面独立平衡姿态已锁定' if tracker.frozen else
            '斜面独立平衡姿态已就绪' if tracker.ready else '等待斜面独立平衡姿态')
        yaw = self.incline_approach_yaw
        if yaw is None:
            yaw = euler_from_quaternion(self._current_quaternion() or (0.,0.,0.,1.))[2]
        baseline = attitude_from_axis_yaw(tracker.axis, yaw)
        for k,v in zip('xyzw', baseline): setattr(status.horizontal_equilibrium_attitude,k,v)
        status.horizontal_prestream_valid = self.incline_approach_prestream is not None
        if status.horizontal_prestream_valid:
            for k,v in zip('xyzw',self.incline_approach_prestream): setattr(status.horizontal_prestream_attitude,k,v)
        status.horizontal_velocity_ready = self.horizontal_velocity_ready
        status.horizontal_velocity_active = self.incline_approach_active
        status.horizontal_velocity_stable = self.horizontal_velocity_stable
        status.horizontal_velocity_reason = (self.horizontal_velocity_reason
            if self.incline.phase == 'WAIT' else self.incline_approach_reason)
        status.horizontal_velocity_x_enu_mps, status.horizontal_velocity_y_enu_mps = self.horizontal_measured
        status.horizontal_target_x_enu_mps, status.horizontal_target_y_enu_mps = c.target
        status.horizontal_accel_x_enu_mps2, status.horizontal_accel_y_enu_mps2 = c.acceleration
        forward = (math.cos(yaw), math.sin(yaw))
        status.horizontal_forward_target_mps = sum(a*b for a,b in zip(forward,c.target))
        status.horizontal_forward_measured_mps = sum(a*b for a,b in zip(forward,self.horizontal_measured))
        status.horizontal_integral_x_enu_mps2, status.horizontal_integral_y_enu_mps2 = c.integral
        status.horizontal_integral_active, status.horizontal_control_saturated = c.integral_active, c.saturated
        status.horizontal_integral_reason = self.incline_approach_integral_reason
        status.optical_flow_quality = self.horizontal_flow.quality if self.horizontal_flow is not None else 0
