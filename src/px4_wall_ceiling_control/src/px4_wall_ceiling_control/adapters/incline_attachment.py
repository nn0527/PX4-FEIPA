"""Incline profile on the existing wall ownership/output transport.

This is a separate state machine, not a second MAVROS publisher. Existing
wall and ceiling controllers are unchanged unless MODE_INCLINE is selected.
"""
import math
import uuid
from dataclasses import replace

import rospy
from geometry_msgs.msg import Quaternion
from px4_wall_ceiling_control.msg import OperatorCommand as O, WallPerchStatus as W
from px4_wall_ceiling_control.adapters.kill_switch import kill_clear
from px4_wall_ceiling_control.adapters.incline_approach import InclineApproachMixin
from px4_wall_ceiling_control.adapters.odometry import linear_velocity_parent_frame
from px4_wall_ceiling_control.domain.geometry import quaternion_conjugate, rotate_vector, euler_from_quaternion
from px4_wall_ceiling_control.domain.incline_attachment import InclineAttachment, InclineFeedback, InclineSettings
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source


class InclineAttachmentMixin(InclineApproachMixin):
    INCLINE_STATES = {
        'WAIT': W.STATE_IDLE, 'APPROACH': W.STATE_SLOW_APPROACH,
        'SLOW_FLIP': W.STATE_FLIP_TO_WALL, 'CONTACT_SETTLE': W.STATE_WALL_CAPTURE,
        'HOLD': W.STATE_WALL_HOLD, 'RETURN': W.STATE_DETACH_ROTATE,
        'RECOVER': W.STATE_RECOVER, 'EXIT': W.STATE_EXIT,
        'LAND': W.STATE_ABORT, 'ABORT_DONE': W.STATE_IDLE,
    }

    def _init_incline(self):
        self.incline_settings = InclineSettings.read(
            lambda key, default: rospy.get_param('/incline_attachment/' + key, default))
        self.incline = InclineAttachment(self.incline_settings, self.max_thrust)
        self.incline_selected = False
        self.incline_ready_since = self.incline_ready_stamp = None
        self.incline_feedback = None
        self._init_incline_approach()

    def _incline_on_operator(self, message):
        # Selection is latched during a maneuver; a different mode only cancels
        # the old action. It cannot turn an attached incline into a wall profile.
        if self.state == W.STATE_IDLE:
            selected = message.mode == O.MODE_INCLINE and message.enable_control
            if not selected and self.incline.phase in ('WAIT', 'ABORT_DONE'):
                self.incline_selected = False
                self.incline = InclineAttachment(self.incline_settings, self.max_thrust)
                self.incline_ready_since = self.incline_ready_stamp = None
            elif selected and not self.incline_selected:
                self.incline_selected = True
                self.incline = InclineAttachment(self.incline_settings, self.max_thrust)
                self.incline_approach_equilibrium.clear()
                self.incline_approach_controller.reset()
                self.incline_approach_yaw = self.incline_approach_sample = None
                self.incline_approach_reference = self.incline_approach_base = None
                self.incline_approach_candidate_axis = None

    def _incline_latch_detach_yaw(self, now):
        # Called under feedback_lock by the detach subscriber. Do not wait for
        # the next control tick, which may already contain a different yaw.
        f = self._incline_feedback_at(now)
        accepted = self.incline.latch_detach_yaw(f)
        if accepted:
            rospy.loginfo('incline detach yaw baseline: measured=%.3fdeg source=%.6f request=%.6f',
                math.degrees(self.incline.detach_yaw_reference),
                self.incline.detach_yaw_reference_stamp, self.incline.detach_yaw_request_at)
        return accepted

    def _incline_requested(self, now):
        return bool(self.decision_enabled and self.operator is not None
            and self.operator.enable_control and self.operator.mode == O.MODE_INCLINE
            and is_fresh_receipt(self.operator_received_at, self.operator_timeout_s, now))

    def _incline_feedback_at(self, now):
        q = self._current_quaternion()
        pose_valid = self._odom_fresh(now) and q is not None
        stamp = 0.
        height = vz = speed = math.nan
        rate = (math.nan,) * 3
        if pose_valid:
            try:
                stamp = self.odom.header.stamp.to_sec()
                velocity = linear_velocity_parent_frame(self.odom, self.odom_twist_in_child_frame)
                height, vz, speed = self.odom.pose.pose.position.z, velocity[2], math.hypot(*velocity[:2])
                omega = self.odom.twist.twist.angular
                rate = (omega.x, omega.y, omega.z)
                if not self.odom_twist_in_child_frame:
                    rate = rotate_vector(quaternion_conjugate(q), rate)
                # MAVROS feedback uses the same bounded source-clock lead as
                # side-wall control; receipt and source timeouts still apply.
                pose_valid = bool(is_fresh_source(self.odom.header.stamp, self.odom_timeout_s, now)
                    and all(math.isfinite(v) for v in (*q, *rate, height, vz, speed))
                    and abs(sum(v*v for v in q)-1.) <= .01)
            except (AttributeError, ValueError, TypeError):
                pose_valid = False
        output = self._valid_wall_output(now)
        flight = self.flight_state
        control = bool(kill_clear(self, now) and flight is not None and flight.connected and flight.armed
            and flight.mode.upper() == 'OFFBOARD'
            and is_fresh_receipt(self.flight_state_received_at, self.state_timeout_s, now)
            and is_fresh_source(flight.header.stamp, self.state_timeout_s, now))
        front_valid, top_valid = self._front_fresh(now), self._top_fresh(now)
        return InclineFeedback(now.to_sec(), q, rate, height, vz, speed, stamp,
            x=self.odom.pose.pose.position.x if pose_valid else math.nan,
            y=self.odom.pose.pose.position.y if pose_valid else math.nan,
            front=self.front_range.range if front_valid else math.nan,
            top=self.top_range.range if top_valid else math.nan,
            top_stamp=self.top_range.header.stamp.to_sec() if top_valid else 0.,
            front_valid=front_valid, top_valid=top_valid, pose_valid=pose_valid,
            control_valid=control, output_valid=output is not None,
            applied_q=tuple(getattr(output.orientation, key) for key in 'xyzw') if output else None,
            applied_thrust=output.thrust if output else math.nan,
            output_stamp=output.header.stamp.to_sec() if output else 0.)

    def _incline_front_ready(self, f):
        if not f.front_valid or f.front >= self.incline_settings.front_ready_distance_m:
            self.incline_ready_since = self.incline_ready_stamp = None
            return False
        stamp = self.front_range.header.stamp.to_sec()
        if self.incline_ready_stamp is None or stamp-self.incline_ready_stamp > .20:
            self.incline_ready_since = stamp
        if self.incline_ready_stamp is None or stamp > self.incline_ready_stamp:
            self.incline_ready_stamp = stamp
        return self.incline_ready_stamp-self.incline_ready_since >= self.incline_settings.front_ready_hold_s-1e-8

    def _incline_entry_height_ready(self):
        height = self._entry_height()
        return bool(height is not None and math.isfinite(height)
                    and height > self.incline_settings.min_entry_height_m)

    def _incline_cycle(self, now):
        engine = self.incline
        old = engine.phase
        requested = self._incline_requested(now)
        self._prepare_horizontal_velocity(now)
        self._incline_approach_prepare(now)
        f = self._incline_feedback_at(now)
        ready = self._incline_front_ready(f)
        failure = (self._incline_approach_failure(f,
            max(0., f.now-engine.entered) if engine.phase == 'APPROACH' else 0.,
            require_control=engine.phase == 'APPROACH')
            if engine.phase in ('WAIT', 'APPROACH') else '')
        if f.front_valid and f.front > self.incline_approach_params['abort_front_distance_m']:
            ready = False
        flight = self.flight_state
        live_mode = (flight is not None and is_fresh_receipt(self.flight_state_received_at, self.state_timeout_s, now)
                     and is_fresh_source(flight.header.stamp, self.state_timeout_s, now))
        if engine.phase in engine.ACTIVE and live_mode and (not flight.armed or flight.mode.upper() != 'OFFBOARD'):
            # Pilot/flight-controller takeover is terminal. Never re-enter the
            # old trajectory when OFFBOARD later reappears.
            engine.phase, engine.reason = 'ABORT_DONE', '斜墙外部模式接管，释放旧任务'
        elif engine.phase == 'WAIT':
            if requested and failure:
                engine.reason = '斜墙接近等待：' + failure
            elif requested:
                engine.reason = '等待斜面独立平衡基准、测距准入及本次OFFBOARD接管确认'
            if (requested and not failure and not engine.error and ready and f.pose_valid and f.control_valid
                    and self._incline_entry_height_ready()):
                if (self._auto_takeover_confirmed(now) and self.incline_approach_equilibrium.frozen
                        and self._incline_approach_begin(f)):
                    seed = self.incline_approach_prestream or self.q_hover
                    if engine.start(f, seed, self.incline_approach_takeover):
                        # Preserve the POSCTL -> OFFBOARD edge reference/base,
                        # including loss before the front-distance flip gate.
                        engine.reference_height = self.incline_approach_reference
                        engine.base = self.incline_approach_base
                        self.maneuver_id = uuid.uuid4().hex
                        self.control_sequence = 0
                        self.hold_achieved = False
        elif engine.phase in engine.ACTIVE:
            if engine.phase == 'APPROACH' and failure:
                engine.abort(f, failure)
            else:
                # Share wall range/velocity admission to contact transitions;
                # never authorize a flip using unhealthy or paused feedback.
                step_feedback = f
                if engine.phase == 'APPROACH' and (
                        not self._incline_approach_ranges_ready(now) or not self.horizontal_velocity_ready):
                    step_feedback = replace(f, front_valid=False)
                engine.step(step_feedback, requested, self.detach_requested, self.cancel_requested)
        elif engine.phase == 'LAND' and live_mode and (not flight.armed or flight.mode.upper() != 'OFFBOARD'):
            engine.phase, engine.reason = 'ABORT_DONE', '斜墙故障交接已结束，禁止自动重启'
        self.state = self.INCLINE_STATES[engine.phase]
        self.abort_land_requested = engine.phase == 'LAND'
        self.last_reason = engine.reason
        self.q_hover = getattr(engine, 'hover', self.q_hover)
        self.detach_requested = self.cancel_requested = False
        if engine.phase != old:
            self.state_entered_at = now
            self.incline_approach_controller.clear_integral()
            self.incline_approach_sample = None
            self._clear_horizontal_integral('斜墙阶段切换')
            if engine.phase == 'SLOW_FLIP':
                self._reset_horizontal_gate('斜墙翻转暂停水平闭环')
            if engine.phase == 'HOLD':
                self.hold_achieved = True
            rospy.loginfo('incline: %s -> %s: %s', old, engine.phase, engine.reason)
        self.incline_feedback = f
        status = self._incline_status(now, f, ready)
        self.status_publisher.publish(status)
        self.control_sequence += 1
        self.control_candidate_publisher.publish(
            self._build_control_candidate(status, self.maneuver_id, self.control_sequence))

    def _incline_status(self, now, f, ready):
        engine = self.incline
        status = W()
        status.header.stamp = now
        status.state = self.state
        # Retain transport state names for the existing console/manager gates.
        status.state_name = self.STATE_NAMES[self.state]
        status.incline_active = True
        status.incline_ready = not engine.error and self.horizontal_velocity_enabled and self.auto_approach
        status.incline_phase = engine.phase
        status.incline_fault_reason = engine.fault
        status.incline_thrust_limit = engine.cap
        status.enabled = status.keep_ownership = engine.phase in engine.ACTIVE or engine.phase == 'LAND'
        status.acquire_request = engine.phase == 'APPROACH' and f.control_valid
        status.odom_fresh = f.pose_valid
        status.flight_state_fresh = f.control_valid
        status.front_range_fresh, status.top_range_fresh = f.front_valid, f.top_valid
        status.front_ready = ready
        status.sensor_health_fresh = self._sensor_health_fresh(now)
        status.entry_height_ready = self._incline_entry_height_ready()
        status.height_above_takeoff_m = self._entry_height() or 0.
        status.altitude_reference_valid = self._entry_height() is not None
        status.min_entry_height_m = self.incline_settings.min_entry_height_m
        status.auto_approach_enabled = True
        status.approach_paused = bool(engine.phase == 'APPROACH'
            and not self._incline_approach_motion_allowed(now))
        status.hold_achieved = self.hold_achieved
        status.detach_ready = engine.phase == 'HOLD' and f.pose_valid and f.control_valid and f.output_valid
        status.fault_detected = bool(engine.fault)
        status.abort_land_requested = engine.phase == 'LAND'
        # Select the same independent FCU/RC/fusion handover gate as captured
        # wall recovery, rather than the legacy task-height/speed gate.
        status.recover_attitude_locked = engine.phase in ('RECOVER', 'EXIT')
        status.recover_stable = engine.recover_stable
        status.detach_final_pitch_deg = math.degrees(euler_from_quaternion(f.q)[1]) if f.pose_valid else math.nan
        status.detach_range_maneuver_id = self.maneuver_id
        status.detach_range_episode = 1
        status.fault_recovery_completed = engine.phase == 'EXIT' and bool(engine.fault)
        status.detach_completed = status.recovery_completed = engine.phase == 'EXIT' and not engine.fault
        status.front_distance_raw_m = status.front_distance_filtered_m = f.front
        status.top_distance_raw_m = status.top_distance_filtered_m = f.top
        status.reason = engine.error or engine.reason
        if engine.phase == 'RETURN' and engine.return_tracking_limited:
            status.reason += '; 回平新目标会继续扩大超限误差，暂停本步'
        q, thrust = engine.q, engine.thrust
        self.horizontal_velocity_active = False
        if engine.phase == 'APPROACH':
            # Copied wall behavior with incline-owned parameters and history.
            q, thrust = self._incline_approach_control(now, f)
            engine.q, engine.thrust = q, thrust
            status.vertical_control_active = True
            status.vertical_reference_valid = self.incline_approach_reference is not None
            status.vertical_reference_m = self.incline_approach_reference if self.incline_approach_reference is not None else math.nan
            status.vertical_reference_source = 'incline_offboard_takeover'
            status.vertical_target_enu_mps = self.incline_approach_target_speed
            status.vertical_thrust_correction = self.incline_approach_correction
            status.approach_takeover_thrust = self.incline_approach_takeover if self.incline_approach_takeover is not None else math.nan
            status.approach_base_vertical_thrust = self.incline_approach_base if self.incline_approach_base is not None else math.nan
        elif engine.phase == 'SLOW_FLIP':
            status.vertical_control_active = True
            status.vertical_reference_valid = math.isfinite(engine.reference_height)
            status.vertical_reference_m = engine.reference_height
            status.vertical_reference_source = 'incline_offboard_takeover_flip_support'
            status.vertical_target_enu_mps = 0.
            status.vertical_thrust_correction = engine.flip_support_correction
            status.flip_vertical_support_enabled = True
            status.flip_vertical_support_weight = 1.
            status.flip_entry_thrust = engine.flip_entry_thrust
            status.flip_base_vertical_thrust = engine.base
            status.flip_support_thrust = engine.flip_support_requested
            status.flip_pressure_thrust = engine.flip_pressure_thrust
        elif engine.phase in ('RECOVER', 'EXIT') and q is not None:
            old_speed = self.horizontal_approach_speed
            try:
                self.horizontal_approach_speed = self.incline_settings.horizontal_approach_speed_mps
                # RECOVER/EXIT only occur after actual and target return are
                # near level. No horizontal correction during contact/return.
                q = self._horizontal_attitude(now, q)
            finally:
                self.horizontal_approach_speed = old_speed
        if engine.phase in ('WAIT', 'APPROACH'):
            self._fill_incline_approach_status(status)
        else:
            self._fill_horizontal_status(status)
        status.attitude_setpoint_valid = q is not None
        if q is not None:
            status.attitude_setpoint = Quaternion(*q)
        status.thrust_normalized = thrust
        status.candidate_valid = bool(status.enabled and f.pose_valid and f.control_valid
            and q is not None and math.isfinite(thrust) and not engine.error)
        status.recovery_support_valid = status.candidate_valid
        status.recovery_thrust_limit = engine.cap
        return status
