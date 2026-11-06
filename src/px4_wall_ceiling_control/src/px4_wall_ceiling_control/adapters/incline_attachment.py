"""Incline profile on the existing wall ownership/output transport.

This is a separate state machine, not a second MAVROS publisher. Existing
wall and ceiling controllers are unchanged unless MODE_INCLINE is selected.
"""
import math
import uuid

import rospy
from geometry_msgs.msg import Quaternion
from px4_wall_ceiling_control.msg import OperatorCommand as O, WallPerchStatus as W
from px4_wall_ceiling_control.adapters.kill_switch import kill_clear
from px4_wall_ceiling_control.adapters.odometry import linear_velocity_parent_frame
from px4_wall_ceiling_control.domain.geometry import quaternion_conjugate, rotate_vector
from px4_wall_ceiling_control.domain.incline_attachment import InclineAttachment, InclineFeedback, InclineSettings
from px4_wall_ceiling_control.freshness import is_fresh_receipt, is_fresh_source


class InclineAttachmentMixin:
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
                pose_valid = bool(is_fresh_source(self.odom.header.stamp, self.odom_timeout_s, now, future_tolerance_s=0.)
                    and all(math.isfinite(v) for v in (*q, *rate, height, vz, speed))
                    and abs(sum(v*v for v in q)-1.) <= .01)
            except (AttributeError, ValueError, TypeError):
                pose_valid = False
        output = self._valid_wall_output(now)
        flight = self.flight_state
        control = bool(kill_clear(self, now) and flight is not None and flight.connected and flight.armed
            and flight.mode.upper() == 'OFFBOARD'
            and is_fresh_receipt(self.flight_state_received_at, self.state_timeout_s, now)
            and is_fresh_source(flight.header.stamp, self.state_timeout_s, now, future_tolerance_s=0.))
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
        self._prepare_horizontal_equilibrium(now)
        f = self._incline_feedback_at(now)
        ready = self._incline_front_ready(f)
        flight = self.flight_state
        live_mode = (flight is not None and is_fresh_receipt(self.flight_state_received_at, self.state_timeout_s, now)
                     and is_fresh_source(flight.header.stamp, self.state_timeout_s, now, future_tolerance_s=0.))
        if engine.phase in engine.ACTIVE and live_mode and (not flight.armed or flight.mode.upper() != 'OFFBOARD'):
            # Pilot/flight-controller takeover is terminal. Never re-enter the
            # old trajectory when OFFBOARD later reappears.
            engine.phase, engine.reason = 'ABORT_DONE', '斜墙外部模式接管，释放旧任务'
        elif engine.phase == 'WAIT':
            if (requested and not engine.error and ready and f.pose_valid and f.control_valid
                    and self._incline_entry_height_ready()):
                if (self._auto_takeover_confirmed(now) and self.horizontal_equilibrium.frozen
                        and self.approach_takeover_thrust is not None):
                    seed = self.horizontal_prestream_attitude or self.q_hover
                    if engine.start(f, seed, self.approach_takeover_thrust):
                        self.maneuver_id = uuid.uuid4().hex
                        self.control_sequence = 0
                        self.hold_achieved = False
                        self.approach_altitude_m = f.height
        elif engine.phase in engine.ACTIVE:
            engine.step(f, requested, self.detach_requested, self.cancel_requested)
        elif engine.phase == 'LAND' and live_mode and (not flight.armed or flight.mode.upper() != 'OFFBOARD'):
            engine.phase, engine.reason = 'ABORT_DONE', '斜墙故障交接已结束，禁止自动重启'
        self.state = self.INCLINE_STATES[engine.phase]
        self.abort_land_requested = engine.phase == 'LAND'
        self.last_reason = engine.reason
        self.q_hover = getattr(engine, 'hover', self.q_hover)
        self.detach_requested = self.cancel_requested = False
        if engine.phase != old:
            self.state_entered_at = now
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
        status.hold_achieved = self.hold_achieved
        status.detach_ready = engine.phase == 'HOLD' and f.pose_valid and f.control_valid and f.output_valid
        status.fault_detected = bool(engine.fault)
        status.abort_land_requested = engine.phase == 'LAND'
        status.detach_range_maneuver_id = self.maneuver_id
        status.detach_range_episode = 1
        status.fault_recovery_completed = engine.phase == 'EXIT' and bool(engine.fault)
        status.detach_completed = status.recovery_completed = engine.phase == 'EXIT' and not engine.fault
        status.front_distance_raw_m = status.front_distance_filtered_m = f.front
        status.top_distance_raw_m = status.top_distance_filtered_m = f.top
        status.reason = engine.error or engine.reason
        q, thrust = engine.q, engine.thrust
        self.horizontal_velocity_active = False
        if engine.phase in ('APPROACH', 'RECOVER', 'EXIT') and q is not None:
            old_speed = self.horizontal_approach_speed
            try:
                self.horizontal_approach_speed = self.incline_settings.horizontal_approach_speed_mps
                # RECOVER/EXIT only occur after actual and target return are
                # near level. No horizontal correction during contact/return.
                q = self._horizontal_attitude(now, q)
            finally:
                self.horizontal_approach_speed = old_speed
            if engine.phase == 'APPROACH':
                engine.q = q
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
