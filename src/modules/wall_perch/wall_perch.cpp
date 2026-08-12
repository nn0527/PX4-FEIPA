/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file wall_perch.cpp
 *
 * wall_perch — RC-triggered wall perching maneuver
 */

#include "wall_perch.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace matrix;

// ==========================================================================
//  Constructor / Destructor
// ==========================================================================

WallPerch::WallPerch() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
}

WallPerch::~WallPerch()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

// ==========================================================================
//  Init
// ==========================================================================

bool WallPerch::init()
{
	parameters_update(true);

	if (!_distance_sensor_subs.advertised()) {
		PX4_WARN("[wall_perch] distance_sensor topic not advertised yet");

	} else {
		PX4_INFO("[wall_perch] distance_sensor topic available");
	}

	PX4_INFO("[wall_perch] Started, 10 ms loop");
	ScheduleOnInterval(10_ms);
	_first_run = true;
	return true;
}

// ==========================================================================
//  Parameter update
// ==========================================================================

void WallPerch::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

// ==========================================================================
//  State name helper
// ==========================================================================

const char *WallPerch::state_name(State s) const
{
	switch (s) {
	case State::IDLE:              return "IDLE";

	case State::FRONT_WALL_DETECT: return "FRONT_WALL_DETECT";

	case State::STABILIZE_HOVER:   return "STABILIZE_HOVER";

	case State::SLOW_APPROACH:     return "SLOW_APPROACH";

	case State::FLIP_TO_WALL:      return "FLIP_TO_WALL";

	case State::WALL_CAPTURE:      return "WALL_CAPTURE";

	case State::WALL_HOLD:         return "WALL_HOLD";

	case State::WALL_PIN:          return "WALL_PIN";

	case State::DETACH_ROTATE:     return "DETACH_ROTATE";

	case State::RECOVER:           return "RECOVER";

	case State::EXIT:              return "EXIT";

	case State::ABORT:             return "ABORT";

	default:                       return "UNKNOWN";
	}
}

// ==========================================================================
//  Main Run loop
// ==========================================================================

void WallPerch::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	const hrt_abstime now = hrt_absolute_time();

	if (_first_run) {
		_first_run = false;
		mavlink_log_info(&_mavlink_log_pub, "[wall_perch] STARTED");
	}

	parameters_update(false);

	// --- Read sensor data ---
	// vehicle_status
	vehicle_status_s status{};

	if (_vehicle_status_sub.copy(&status)) {
		_armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		_nav_state = status.nav_state;
		_rotary_wing = status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_vehicle_status_ts = status.timestamp;
	}

	vehicle_control_mode_s control_mode{};

	if (_vehicle_control_mode_sub.copy(&control_mode)) {
		_vehicle_control_mode = control_mode;
		_vehicle_control_mode_ts = control_mode.timestamp;
	}

	// vehicle_attitude
	vehicle_attitude_s att{};

	if (_vehicle_attitude_sub.copy(&att)) {
		Quatf q(att.q);
		_q_current = q;
		_attitude_euler = Eulerf(q);
		_current_yaw = _attitude_euler.psi();
		_vehicle_attitude_ts = att.timestamp;
	}

	// vehicle_angular_velocity
	vehicle_angular_velocity_s ang_vel{};

	if (_vehicle_angular_velocity_sub.copy(&ang_vel)) {
		_angular_velocity(0) = ang_vel.xyz[0];
		_angular_velocity(1) = ang_vel.xyz[1];
		_angular_velocity(2) = ang_vel.xyz[2];
		_vehicle_angular_velocity_ts = ang_vel.timestamp;
	}

	// vehicle_local_position
	vehicle_local_position_s local_pos{};

	if (_vehicle_local_pos_sub.copy(&local_pos)) {
		_velocity(0) = local_pos.vx;
		_velocity(1) = local_pos.vy;
		_velocity(2) = local_pos.vz;
		_local_position_valid = local_pos.z_valid && PX4_ISFINITE(local_pos.z);
		_local_velocity_valid = local_pos.v_z_valid && PX4_ISFINITE(local_pos.vz);

		if (_local_position_valid) {
			_current_altitude = -local_pos.z; // NED z-up -> altitude
		}

		_vehicle_local_position_ts = local_pos.timestamp;
	}

	update_manual_switches();

	// distance_sensor (front and upward), selected by orientation across all instances
	update_distance_sensors();

	// hover_thrust_estimate (for dynamic thrust scaling)
	hover_thrust_estimate_s hte{};

	if (_hover_thrust_estimate_sub.copy(&hte) && hte.valid) {
		_hover_thrust_estimate = math::constrain(hte.hover_thrust, 0.1f, 0.9f);
	}

	// --- Compute dt ---
	static hrt_abstime last_run{0};
	float dt = 0.01f;

	if (last_run != 0) {
		dt = math::constrain((now - last_run) / 1e6f, 0.001f, 0.05f);
	}

	last_run = now;

	// --- Update state machine ---
	_last_thrust_norm = 0.f;
	_state_progress = 0.f;
	update_state_machine(dt);

	// --- Publish status ---
	publish_wall_perch_status();

	// --- Periodic log ---
	if (hrt_elapsed_time(&_last_status_log_time) > 5_s) {
		mavlink_log_info(&_mavlink_log_pub,
				 "[wall_perch] st=%s(%d) fdist=%.3f tdist=%.3f armed=%d",
				 state_name(_state), (int)_state,
				 (double)_front_wall_distance_m, (double)_top_wall_distance_m,
				 (int)_armed);
		_last_status_log_time = now;
	}

	perf_end(_loop_perf);
}

// ==========================================================================
//  Distance sensor helpers
// ==========================================================================

void WallPerch::update_distance_sensors()
{
	distance_sensor_s newest_front{};
	distance_sensor_s newest_top{};
	bool front_updated = false;
	bool top_updated = false;

	for (auto &distance_sub : _distance_sensor_subs) {
		distance_sensor_s msg{};

		if (!distance_sub.update(&msg)
		    || msg.timestamp == 0
		    || !PX4_ISFINITE(msg.current_distance)
		    || msg.current_distance < msg.min_distance
		    || msg.current_distance > msg.max_distance
		    || msg.signal_quality == 0) {
			continue;
		}

		if (msg.orientation == distance_sensor_s::ROTATION_FORWARD_FACING
		    && (!front_updated || msg.timestamp > newest_front.timestamp)) {
			newest_front = msg;
			front_updated = true;

		} else if (msg.orientation == distance_sensor_s::ROTATION_UPWARD_FACING
			   && (!top_updated || msg.timestamp > newest_top.timestamp)) {
			newest_top = msg;
			top_updated = true;
		}
	}

	if (front_updated) {
		_front_distance_ts = newest_front.timestamp;
		const float raw = newest_front.current_distance;

		if (!_lpf_front_initialized) {
			_front_distance_lpf = raw;
			_lpf_front_initialized = true;

		} else {
			_front_distance_lpf += 0.3f * (raw - _front_distance_lpf);
		}

		_front_wall_distance_m = _front_distance_lpf;
	}

	if (top_updated) {
		_top_distance_ts = newest_top.timestamp;
		const float raw = newest_top.current_distance;

		if (!_lpf_top_initialized) {
			_top_distance_lpf = raw;
			_lpf_top_initialized = true;

		} else {
			_top_distance_lpf += 0.3f * (raw - _top_distance_lpf);
		}

		_top_wall_distance_m = _top_distance_lpf;
	}
}

// ==========================================================================
//  User input helpers
// ==========================================================================

void WallPerch::update_manual_switches()
{
	manual_control_setpoint_s manual{};

	if (!_manual_control_setpoint_sub.copy(&manual)) {
		_start_switch_on = false;
		_detach_switch_on = false;
		return;
	}

	_aux1_raw = manual.aux1; _aux2_raw = manual.aux2;
	_aux3_raw = manual.aux3; _aux4_raw = manual.aux4;
	_manual_control_ts = manual.timestamp;
	_start_switch_on = selected_aux_value(_param_wp_aux_ch.get()) > 0.3f;
	_detach_switch_on = selected_aux_value(_param_wp_detach_aux_ch.get()) > 0.3f;
}

float WallPerch::selected_aux_value(int channel) const
{
	switch (channel) {
	case 1: return _aux1_raw;

	case 2: return _aux2_raw;

	case 3: return _aux3_raw;

	case 4: return _aux4_raw;

	default: return -1.f;
	}
}

bool WallPerch::user_start_requested()
{
	return _param_wp_enable.get() && _start_switch_on;
}

bool WallPerch::user_detach_requested()
{
	return _detach_switch_on;
}

// ==========================================================================
//  Generic hold condition
// ==========================================================================

bool WallPerch::hold_condition(bool condition, hrt_abstime &start_time, float hold_time_s)
{
	const hrt_abstime now = hrt_absolute_time();

	if (condition) {
		if (start_time == 0) {
			start_time = now;
		}

		if (hrt_elapsed_time(&start_time) >= (hrt_abstime)(hold_time_s * 1e6f)) {
			return true;
		}

	} else {
		start_time = 0;
	}

	return false;
}

// ==========================================================================
//  Distance condition checks (with hysteresis via hold_condition)
// ==========================================================================

bool WallPerch::front_ready()
{
	return hold_condition(
		       _front_wall_distance_m < _param_wp_frn_rd_dist.get(),
		       _front_ready_start,
		       _param_wp_frn_rd_hld.get()
	       );
}

bool WallPerch::flip_ready()
{
	return hold_condition(
		       _front_wall_distance_m <= _param_wp_flp_trd_dist.get(),
		       _flip_ready_start,
		       _param_wp_flp_trd_hld.get()
	       );
}

bool WallPerch::top_contact_ready()
{
	return hold_condition(
		       _top_wall_distance_m <= _param_wp_top_ct_dist.get(),
		       _top_contact_start,
		       _param_wp_top_ct_hld.get()
	       );
}

// ==========================================================================
//  Sensor validity
// ==========================================================================

bool WallPerch::sensors_valid()
{
	const hrt_abstime now = hrt_absolute_time();
	const float timeout_us = _param_wp_sens_timeout.get() * 1e6f;
	const bool front_fresh = _front_distance_ts != 0 && now >= _front_distance_ts
				 && (float)(now - _front_distance_ts) <= timeout_us;
	const bool top_fresh = _top_distance_ts != 0 && now >= _top_distance_ts
			       && (float)(now - _top_distance_ts) <= timeout_us;

	// FRONT is needed until the flip starts. UP is needed while rotating toward
	// and attached to the wall. Recovery deliberately does not depend on either
	// rangefinder, otherwise a sensor fault could prevent a safe detach.
	if (_state <= State::SLOW_APPROACH && !front_fresh) {
		return false;
	}

	if (_state >= State::FLIP_TO_WALL && _state <= State::WALL_PIN && !top_fresh) {
		return false;
	}

	if (_state == State::WALL_PIN && !front_fresh) {
		return false;
	}

	return true;
}

// ==========================================================================
//  Safety checks
// ==========================================================================

bool WallPerch::safety_ok()
{
	if (!_armed) { return false; }

	if (!sensors_valid()) { return false; }

	// Rate and velocity limits are relaxed once the flip has started:
	// the maneuver intentionally produces high angular rates and downward
	// velocity during/after the slerp flip.
	if (_state < State::FLIP_TO_WALL) {
		if (!rate_safe()) { return false; }

		if (!vz_safe()) { return false; }
	}

	return true;
}

bool WallPerch::control_mode_valid() const
{
	const hrt_abstime now = hrt_absolute_time();
	constexpr hrt_abstime input_timeout = 500_ms;
	const bool status_fresh = _vehicle_status_ts != 0 && now >= _vehicle_status_ts
				  && (now - _vehicle_status_ts) <= input_timeout;
	const bool mode_fresh = _vehicle_control_mode_ts != 0 && now >= _vehicle_control_mode_ts
				&& (now - _vehicle_control_mode_ts) <= input_timeout;

	return status_fresh && mode_fresh && _armed && _rotary_wing
	       && _nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL
	       && _vehicle_control_mode.flag_armed
	       && _vehicle_control_mode.flag_control_manual_enabled
	       && _vehicle_control_mode.flag_control_altitude_enabled
	       && _vehicle_control_mode.flag_control_climb_rate_enabled
	       && _vehicle_control_mode.flag_control_attitude_enabled
	       && _vehicle_control_mode.flag_control_rates_enabled
	       && _vehicle_control_mode.flag_control_allocation_enabled
	       && !_vehicle_control_mode.flag_control_auto_enabled
	       && !_vehicle_control_mode.flag_control_offboard_enabled;
}

bool WallPerch::start_conditions_valid() const
{
	const hrt_abstime now = hrt_absolute_time();
	constexpr hrt_abstime input_timeout = 500_ms;
	const float distance_timeout_us = _param_wp_sens_timeout.get() * 1e6f;
	const bool state_inputs_fresh = _vehicle_attitude_ts != 0 && now >= _vehicle_attitude_ts
					&& (now - _vehicle_attitude_ts) <= input_timeout
					&& _vehicle_angular_velocity_ts != 0 && now >= _vehicle_angular_velocity_ts
					&& (now - _vehicle_angular_velocity_ts) <= input_timeout
					&& _vehicle_local_position_ts != 0 && now >= _vehicle_local_position_ts
					&& (now - _vehicle_local_position_ts) <= input_timeout
					&& _manual_control_ts != 0 && now >= _manual_control_ts
					&& (now - _manual_control_ts) <= input_timeout;
	const bool both_ranges_fresh = _front_distance_ts != 0 && now >= _front_distance_ts
				       && (float)(now - _front_distance_ts) <= distance_timeout_us
				       && _top_distance_ts != 0 && now >= _top_distance_ts
				       && (float)(now - _top_distance_ts) <= distance_timeout_us;

	return control_mode_valid() && state_inputs_fresh && both_ranges_fresh
	       && _local_position_valid && _local_velocity_valid
	       && PX4_ISFINITE(_current_altitude) && PX4_ISFINITE(_velocity(2))
	       && _current_altitude > _param_wp_min_alt.get()
	       && rate_safe() && vz_safe();
}

bool WallPerch::rate_safe() const
{
	float max_rate = _param_wp_max_rate.get();
	return (fabsf(_angular_velocity(0)) <= max_rate &&
		fabsf(_angular_velocity(1)) <= max_rate &&
		fabsf(_angular_velocity(2)) <= max_rate);
}

bool WallPerch::vz_safe() const
{
	// vz is NED: positive = down
	return _velocity(2) <= _param_wp_max_vz_down.get();
}

bool WallPerch::vz_stable() const
{
	return fabsf(_velocity(2)) <= _param_wp_max_vz_down.get();
}

bool WallPerch::wall_owns_attitude() const
{
	return (_state >= State::SLOW_APPROACH && _state <= State::RECOVER)
	       || _state == State::ABORT;
}

float WallPerch::tilt_compensated_thrust(const Quatf &q_des, float minimum_thrust) const
{
	const float cos_tilt = fabsf(Dcmf(q_des)(2, 2));
	const float max_thrust = math::constrain(_param_wp_flip_thr_max.get(), 0.1f, 1.f);
	const float compensated = cos_tilt > 0.05f ? _hover_thrust / cos_tilt : max_thrust;
	return math::constrain(math::max(compensated, minimum_thrust), 0.f, max_thrust);
}

bool WallPerch::attitude_recovered() const
{
	float limit = math::radians(_param_wp_recover_rp.get());
	return (fabsf(_attitude_euler.phi()) < limit &&
		fabsf(_attitude_euler.theta()) < limit);
}

// ==========================================================================
//  Attitude computation
// ==========================================================================

Quatf WallPerch::compute_hover_attitude(float yaw)
{
	return Quatf(Eulerf(0.f, 0.f, yaw));
}

Quatf WallPerch::compute_wall_attitude(const Quatf &q_hover)
{
	// Nose-forward wall perching: negative pitch angle tilts nose toward wall
	// q_wall = q_hover * quat_from_euler(0, angle, 0)
	// Using 3-2-1 (psi-theta-phi) intrinsic: theta is pitch forward (negative = nose down)
	float angle_rad = math::radians(_param_wp_wall_angle.get());
	Quatf q_tilt(Eulerf(0.f, angle_rad, 0.f));
	return q_hover * q_tilt;
}

// ==========================================================================
//  Math utilities
// ==========================================================================

float WallPerch::smoothstep5(float tau)
{
	// 10*t^3 - 15*t^4 + 6*t^5 — zero first and second derivatives at boundaries
	float t = math::constrain(tau, 0.f, 1.f);
	float t2 = t * t;
	float t3 = t2 * t;
	return 10.f * t3 - 15.f * t3 * t + 6.f * t3 * t2;
}

Quatf WallPerch::slerp_quat(const Quatf &q0, const Quatf &q1, float s)
{
	// Spherical linear interpolation between q0 and q1 at parameter s in [0,1]
	// Based on: q = q0 * (q0^-1 * q1)^s
	Quatf q0_unit = q0.unit();
	Quatf q1_unit = q1.unit();

	// Compute the cosine of the angle between the two quaternions
	float cos_omega = q0_unit(0) * q1_unit(0) +
			  q0_unit(1) * q1_unit(1) +
			  q0_unit(2) * q1_unit(2) +
			  q0_unit(3) * q1_unit(3);

	// Take the shortest path
	Quatf q1_adj = q1_unit;

	if (cos_omega < 0.f) {
		q1_adj = q1_unit * -1.f;
		cos_omega = -cos_omega;
	}

	// Linear interpolation for small angles (avoid division by zero)
	const float k_slerp_epsilon = 1e-6f;

	if (cos_omega > 1.f - k_slerp_epsilon) {
		Quatf result = q0_unit * (1.f - s) + q1_adj * s;
		result.normalize();
		return result;
	}

	// Standard slerp
	float omega = std::acos(cos_omega);
	float sin_omega = std::sin(omega);
	float scale0 = std::sin((1.f - s) * omega) / sin_omega;
	float scale1 = std::sin(s * omega) / sin_omega;

	Quatf result = q0_unit * scale0 + q1_adj * scale1;
	result.normalize();
	return result;
}

float WallPerch::ramp(float from, float to, float duration)
{
	if (duration < 1e-6f) { return to; }

	float elapsed = (float)hrt_elapsed_time(&_state_entry_time) * 1e-6f;
	float tau = math::constrain(elapsed / duration, 0.f, 1.f);
	return from + (to - from) * tau;
}

// ==========================================================================
//  Output
// ==========================================================================

void WallPerch::publish_attitude_setpoint(const Quatf &q_des, float thrust_norm)
{
	const float thrust = math::constrain(thrust_norm, 0.f, 1.f);
	vehicle_attitude_setpoint_s sp{};
	sp.timestamp = hrt_absolute_time();
	q_des.copyTo(sp.q_d);

	sp.thrust_body[0] = 0.0f;
	sp.thrust_body[1] = 0.0f;
	sp.thrust_body[2] = -thrust; // thrust UP in body frame

	_att_sp_pub.publish(sp);
	_last_thrust_norm = thrust;
}

void WallPerch::publish_wall_perch_status()
{
	wall_perch_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.state = (uint8_t)_state;
	status.active = wall_owns_attitude();
	status.start_switch_on = _start_switch_on;
	status.detach_switch_on = _detach_switch_on;
	status.rearm_required = _rearm_required;
	status.control_mode_valid = control_mode_valid();
	status.direct_motor_control = _state == State::WALL_PIN;
	status.front_ready = _front_wall_distance_m < _param_wp_frn_rd_dist.get();
	status.flip_ready = _front_wall_distance_m <= _param_wp_flp_trd_dist.get();
	status.top_contact_ready = _top_wall_distance_m <= _param_wp_top_ct_dist.get();
	status.front_wall_distance_m = _front_wall_distance_m;
	status.top_wall_distance_m = _top_wall_distance_m;
	status.progress = _state_progress;
	status.thrust_norm = _last_thrust_norm;
	status.failsafe_triggered = _failsafe_triggered;
	_status_pub.publish(status);
}

bool WallPerch::pin_trigger_reached() const
{
	if (!_param_wp_pin_enable.get()) { return false; }

	// Nose-down pitch magnitude (theta) reaching the configured threshold.
	return fabsf(_attitude_euler.theta()) >= math::radians(_param_wp_pin_pitch.get());
}

void WallPerch::publish_actuator_motors(float thrust)
{
	actuator_motors_s motors{};
	motors.timestamp = hrt_absolute_time();
	motors.timestamp_sample = motors.timestamp;

	// Non-reversible motors: control[] in [0,1] is remapped to [-1,1] by the
	// mixer. 1.0 => full forward (max RPM).  See FunctionMotors::updateValues.
	motors.reversible_flags = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		motors.control[i] = NAN;
	}

	const float t = math::constrain(thrust, 0.f, 1.f);

	// Four fans -> Motor1..Motor4 (control indices 0..3).
	for (int i = 0; i < 4; ++i) {
		motors.control[i] = t;
	}

	_actuator_motors_pub.publish(motors);
	_last_thrust_norm = t;
}

// ==========================================================================
//  State entry
// ==========================================================================

void WallPerch::enter_state(State new_state)
{
	if (_state == new_state) { return; }

	PX4_INFO("[wall_perch] %s -> %s", state_name(_state), state_name(new_state));
	mavlink_log_info(&_mavlink_log_pub, "[wall_perch] %s -> %s",
			 state_name(_state), state_name(new_state));

	_state = new_state;
	_state_entry_time = hrt_absolute_time();

	switch (_state) {
	case State::FRONT_WALL_DETECT:
		// Snapshot the ALTCTL entry state. The entry altitude is diagnostic only:
		// recovery hands back at the actual altitude reached after detaching.
		_entry_altitude = _current_altitude;
		_entry_vertical_velocity = _velocity(2);
		_q_entry = _q_current;
		_yaw_hold = _current_yaw;
		_q_hover = compute_hover_attitude(_yaw_hold);
		_q_wall = compute_wall_attitude(_q_hover);
		_rearm_required = true;
		_failsafe_triggered = false;
		_hover_thrust_recorded = true;
		_hover_thrust = _hover_thrust_estimate;

		if (_hover_thrust < 0.1f || _hover_thrust > 0.9f) {
			_hover_thrust = _param_wp_thr_hover.get();
		}

		mavlink_log_info(&_mavlink_log_pub,
				 "[wall_perch] ALTCTL start alt=%.2f vz=%.2f hover=%.3f",
				 (double)_entry_altitude, (double)_entry_vertical_velocity, (double)_hover_thrust);
		break;

	case State::SLOW_APPROACH:
		_approach_start_time = hrt_absolute_time();
		break;

	case State::FLIP_TO_WALL:
		_flip_start_time = hrt_absolute_time();
		break;

	case State::DETACH_ROTATE:
		_detach_start_time = hrt_absolute_time();
		_q_detach_start = _q_current;
		break;

	case State::ABORT:
		_detach_start_time = hrt_absolute_time();
		_q_detach_start = _q_current;
		_failsafe_triggered = true;
		break;

	case State::WALL_PIN:
		_pin_start_time = hrt_absolute_time();
		mavlink_log_info(&_mavlink_log_pub,
				 "[wall_perch] WALL_PIN: Control Allocator yielded, motors at %.2f",
				 (double)_param_wp_pin_thr.get());
		break;

	case State::EXIT:
		_handoff_altitude = _current_altitude;
		// Clear all timers
		_front_ready_start = 0;
		_flip_ready_start = 0;
		_top_contact_start = 0;
		_hover_thrust_recorded = false;
		mavlink_log_info(&_mavlink_log_pub,
				 "[wall_perch] ALTCTL handoff at current alt=%.2f (entry %.2f)",
				 (double)_handoff_altitude, (double)_entry_altitude);
		break;

	default:
		break;
	}
}

// ==========================================================================
//  State machine
// ==========================================================================

void WallPerch::update_state_machine(float dt)
{
	(void)dt;

	const bool start_req = user_start_requested();
	const bool detach_req = user_detach_requested();
	const bool mode_valid = control_mode_valid();
	const bool safe = safety_ok();

	// --- State behaviors ---
	switch (_state) {

	// ================================================================
	// IDLE
	// ================================================================
	case State::IDLE: {
			// A completed maneuver is latched out until AUX1 has first been
			// observed low. AUX2 also blocks every new start while held high.
			if (_rearm_required && !_start_switch_on) {
				_rearm_required = false;
			}

			if (start_req && !detach_req && !_rearm_required && start_conditions_valid()) {
				enter_state(State::FRONT_WALL_DETECT);
			}

			break;
		}

	// ================================================================
	// FRONT_WALL_DETECT — read front distance, check ready
	// ================================================================
	case State::FRONT_WALL_DETECT: {
			if (detach_req) { enter_state(State::EXIT); break; }

			if (!safe || !mode_valid) {
				_failsafe_triggered = true;
				enter_state(State::EXIT);
				break;
			}

			if (front_ready()) {
				enter_state(State::STABILIZE_HOVER);
			}

			break;
		}

	// ================================================================
	// STABILIZE_HOVER — maintain level hover, record yaw
	// Do NOT publish attitude setpoint here: let mc_pos_control keep
	// the position hold so there is no thrust jump when wall_perch
	// later takes over in SLOW_APPROACH.
	// ================================================================
	case State::STABILIZE_HOVER: {
			if (detach_req) { enter_state(State::EXIT); break; }

			if (!safe || !mode_valid) {
				_failsafe_triggered = true;
				enter_state(State::EXIT);
				break;
			}

			// Wait for stabilize time + attitude settled
			_state_progress = math::constrain((float)hrt_elapsed_time(&_state_entry_time) * 1e-6f
							  / _param_wp_stab_time.get(), 0.f, 1.f);

			if (hrt_elapsed_time(&_state_entry_time) > (hrt_abstime)(_param_wp_stab_time.get() * 1e6f) &&
			    attitude_recovered() && rate_safe()) {
				enter_state(State::SLOW_APPROACH);
			}

			break;
		}

	// ================================================================
	// SLOW_APPROACH — tilt forward slightly, move toward wall
	// ================================================================
	case State::SLOW_APPROACH: {
			if (!_armed) { enter_state(State::EXIT); break; }

			if (detach_req) { enter_state(State::DETACH_ROTATE); break; }

			if (!safe || !mode_valid) {
				_failsafe_triggered = true;
				enter_state(State::DETACH_ROTATE);
				break;
			}

			// Timeout check
			_state_progress = math::constrain((float)hrt_elapsed_time(&_approach_start_time) * 1e-6f
							  / _param_wp_appr_timeout.get(), 0.f, 1.f);

			if (hrt_elapsed_time(&_approach_start_time) >
			    (hrt_abstime)(_param_wp_appr_timeout.get() * 1e6f)) {
				PX4_WARN("[wall_perch] Approach timeout");
				_failsafe_triggered = true;
				enter_state(State::DETACH_ROTATE); break;
			}

			// Small forward tilt
			// PX4 pitch is negative for nose-forward motion.  WP_APPR_PITCH is
			// configured as a positive magnitude in the parameter metadata.
			float approach_pitch = -math::radians(_param_wp_appr_pitch.get());
			Quatf q_approach = _q_hover * Quatf(Eulerf(0.f, approach_pitch, 0.f));
			publish_attitude_setpoint(q_approach,
						  tilt_compensated_thrust(q_approach, _param_wp_thr_approach.get()));

			if (flip_ready()) {
				enter_state(State::FLIP_TO_WALL);
			}

			break;
		}

	// ================================================================
	// FLIP_TO_WALL — slerp from hover to wall attitude
	// ================================================================
	case State::FLIP_TO_WALL: {
			if (!_armed) { enter_state(State::EXIT); break; }

			if (detach_req) { enter_state(State::DETACH_ROTATE); break; }

			if (!safe || !mode_valid) {
				_failsafe_triggered = true;
				enter_state(State::DETACH_ROTATE);
				break;
			}

			float duration = _param_wp_flip_time.get();
			float elapsed  = (float)hrt_elapsed_time(&_flip_start_time) * 1e-6f;
			float tau = math::constrain(elapsed / duration, 0.f, 1.f);
			float s = smoothstep5(tau);
			_state_progress = tau;

			Quatf q_des = slerp_quat(_q_hover, _q_wall, s);
			publish_attitude_setpoint(q_des, tilt_compensated_thrust(q_des, _hover_thrust));

			// Once the nose-down pitch reaches the threshold, bypass the mixer and
			// pin the aircraft to the wall at full thrust (no attitude recovery).
			if (pin_trigger_reached()) {
				enter_state(State::WALL_PIN);
				break;
			}

			// Transition to WALL_CAPTURE when slerp is done or top sensor already sees wall
			if (tau >= 1.f || top_contact_ready()) {
				enter_state(State::WALL_CAPTURE);
			}

			break;
		}

	// ================================================================
	// WALL_CAPTURE — confirm wall contact, ramp thrust
	// ================================================================
	case State::WALL_CAPTURE: {
			if (!_armed) { enter_state(State::EXIT); break; }

			if (detach_req) { enter_state(State::DETACH_ROTATE); break; }

			if (!safe || !mode_valid) {
				_failsafe_triggered = true;
				enter_state(State::DETACH_ROTATE);
				break;
			}

			_state_progress = math::constrain((float)hrt_elapsed_time(&_state_entry_time) * 1e-6f
							  / _param_wp_capture_time.get(), 0.f, 1.f);
			publish_attitude_setpoint(_q_wall, tilt_compensated_thrust(_q_wall, _hover_thrust));

			if (pin_trigger_reached()) {
				enter_state(State::WALL_PIN);
				break;
			}

			if (top_contact_ready()) {
				enter_state(State::WALL_HOLD);
			}

			// Timeout check — if no contact after capture_time * 3
			if (hrt_elapsed_time(&_state_entry_time) >
			    (hrt_abstime)(_param_wp_capture_time.get() * 3.f * 1e6f)) {
				PX4_WARN("[wall_perch] Wall capture timeout");
				_failsafe_triggered = true;
				enter_state(State::DETACH_ROTATE);
			}

			break;
		}

	// ================================================================
	// WALL_HOLD — maintain wall attitude and thrust
	// ================================================================
	case State::WALL_HOLD: {
			if (!_armed) { enter_state(State::EXIT); break; }

			if (detach_req) { enter_state(State::DETACH_ROTATE); break; }

			if (!safe || !mode_valid) {
				_failsafe_triggered = true;
				enter_state(State::DETACH_ROTATE);
				break;
			}

			publish_attitude_setpoint(_q_wall, tilt_compensated_thrust(_q_wall, _hover_thrust));

			if (pin_trigger_reached()) {
				enter_state(State::WALL_PIN);
				break;
			}

			// Check top contact still valid
			if (_top_wall_distance_m > _param_wp_top_ct_dist.get() * 3.f) {
				PX4_WARN("[wall_perch] Lost wall contact");
				_failsafe_triggered = true;
				enter_state(State::DETACH_ROTATE); break;
			}

			float hold_time = _param_wp_hold_time.get();

			if (hold_time > 0.f &&
			    hrt_elapsed_time(&_state_entry_time) > (hrt_abstime)(hold_time * 1e6f)) {
				enter_state(State::DETACH_ROTATE);
			}

			break;
		}

	// ================================================================
	// WALL_PIN — Control Allocator yields via WallPerchStatus and this module
	// directly commands all four motors. Altitude is recorded only.
	// ================================================================
	case State::WALL_PIN: {
			if (!_armed) {
				enter_state(State::EXIT);
				break;
			}

			if (detach_req || !mode_valid || !sensors_valid()) {
				if (!mode_valid || !sensors_valid()) {
					_failsafe_triggered = true;
				}

				// Publishing direct_motor_control=false at the end of this cycle
				// re-enables Control Allocator before the first recovery setpoint.
				enter_state(State::DETACH_ROTATE);
				break;
			}

			if (_param_wp_pin_hold.get() > 0.f &&
			    hrt_elapsed_time(&_pin_start_time) > (hrt_abstime)(_param_wp_pin_hold.get() * 1e6f)) {
				_failsafe_triggered = true;
				enter_state(State::DETACH_ROTATE); break;
			}

			publish_actuator_motors(_param_wp_pin_thr.get());

			break;
		}

	// ================================================================
	// DETACH_ROTATE — slerp back to hover, schedule thrust
	// ================================================================
	case State::DETACH_ROTATE: {
			if (!_armed) { enter_state(State::EXIT); break; }

			float duration = _param_wp_detach_time.get();
			float elapsed  = (float)hrt_elapsed_time(&_detach_start_time) * 1e-6f;
			float tau = math::constrain(elapsed / duration, 0.f, 1.f);
			float s = smoothstep5(tau);
			_state_progress = tau;

			Quatf q_des = slerp_quat(_q_detach_start, _q_hover, s);
			publish_attitude_setpoint(q_des, tilt_compensated_thrust(q_des, _hover_thrust));

			if (tau >= 1.f) {
				enter_state(State::RECOVER);
			}

			break;
		}

	// ================================================================
	// RECOVER — hover and wait for stability
	// ================================================================
	case State::RECOVER: {
			if (!_armed) { enter_state(State::EXIT); break; }

			_state_progress = math::constrain((float)hrt_elapsed_time(&_state_entry_time) * 1e-6f
							  / _param_wp_recover_time.get(), 0.f, 1.f);
			publish_attitude_setpoint(_q_hover, tilt_compensated_thrust(_q_hover, _hover_thrust));

			if (hrt_elapsed_time(&_state_entry_time) > (hrt_abstime)(_param_wp_recover_time.get() * 1e6f) &&
			    attitude_recovered() && rate_safe() && vz_stable()) {
				enter_state(State::EXIT);
			}

			break;
		}

	// ================================================================
	// EXIT — release control
	// ================================================================
	case State::EXIT: {
			mavlink_log_info(&_mavlink_log_pub, "[wall_perch] EXIT, releasing control");
			enter_state(State::IDLE);
			break;
		}

	// ================================================================
	// ABORT — emergency recovery, don't drop thrust
	// ================================================================
	case State::ABORT: {
			if (!_armed) { enter_state(State::EXIT); break; }

			const float elapsed = (float)hrt_elapsed_time(&_detach_start_time) * 1e-6f;
			const float tau = math::constrain(elapsed / _param_wp_detach_time.get(), 0.f, 1.f);
			const float s = smoothstep5(tau);
			const Quatf q_des = slerp_quat(_q_detach_start, _q_hover, s);
			_state_progress = tau;
			publish_attitude_setpoint(q_des, tilt_compensated_thrust(q_des, _hover_thrust));

			if (tau >= 1.f && attitude_recovered() && rate_safe() && vz_stable()) {
				enter_state(State::EXIT);
			}

			break;
		}

	} // end switch
}

// ==========================================================================
//  Module entry points
// ==========================================================================

int WallPerch::task_spawn(int argc, char *argv[])
{
	WallPerch *instance = new WallPerch();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int WallPerch::custom_command(int, char *[])
{
	return print_usage("unknown command");
}

int WallPerch::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s\n", reason); }

	PRINT_MODULE_USAGE_NAME("wall_perch", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int wall_perch_main(int argc, char *argv[])
{
	return WallPerch::main(argc, argv);
}
