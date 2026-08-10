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
		PX4_INFO("[wall_perch] distance_sensor instances available: %u",
			 (unsigned)_distance_sensor_subs.advertised_count());
	}

	PX4_INFO("[wall_perch] expects front(+X, orient=0) and suction(-Z, orient=24)");
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
	}

	// vehicle_attitude
	vehicle_attitude_s att{};
	if (_vehicle_attitude_sub.copy(&att)) {
		Quatf q(att.q);
		_q_current = q;
		_attitude_euler = Eulerf(q);
		_current_yaw = _attitude_euler.psi();
	}

	// vehicle_angular_velocity
	vehicle_angular_velocity_s ang_vel{};
	if (_vehicle_angular_velocity_sub.copy(&ang_vel)) {
		_angular_velocity(0) = ang_vel.xyz[0];
		_angular_velocity(1) = ang_vel.xyz[1];
		_angular_velocity(2) = ang_vel.xyz[2];
	}

	// vehicle_local_position
	vehicle_local_position_s local_pos{};
	if (_vehicle_local_pos_sub.copy(&local_pos)) {
		_velocity(0) = local_pos.vx;
		_velocity(1) = local_pos.vy;
		_velocity(2) = local_pos.vz;
		_current_altitude = -local_pos.z; // NED z-up -> altitude
		_local_position_ts = local_pos.timestamp;
		_local_position_data_valid = local_pos.z_valid && local_pos.v_z_valid
					     && PX4_ISFINITE(local_pos.z) && PX4_ISFINITE(local_pos.vz);
	}

	// distance_sensor: route all advertised instances by mounting orientation.
	update_distance_sensors();

	// hover_thrust_estimate (for dynamic thrust scaling)
	hover_thrust_estimate_s hte{};
	if (_hover_thrust_estimate_sub.copy(&hte) && hte.valid) {
		_hover_thrust_estimate = math::constrain(hte.hover_thrust, 0.1f, 0.9f);
		_hover_thrust_estimate_valid = true;
	}

	// --- Compute dt ---
	static hrt_abstime last_run{0};
	float dt = 0.01f;
	if (last_run != 0) {
		dt = math::constrain((now - last_run) / 1e6f, 0.001f, 0.05f);
	}
	last_run = now;

	// --- Update state machine ---
	update_state_machine(dt);

	// --- Publish status ---
	publish_wall_perch_status();

	// --- Periodic log ---
	if (hrt_elapsed_time(&_last_status_log_time) > 5_s) {
		mavlink_log_info(&_mavlink_log_pub,
			"[wall_perch] st=%s f=%.3f[%d,%d] z=%.3f[%d,%d] armed=%d",
			state_name(_state),
			(double)_front_wall_distance_m, (int)_front_sensor_instance, (int)front_sensor_valid(),
			(double)_top_wall_distance_m, (int)_top_sensor_instance, (int)top_sensor_valid(),
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
	static constexpr float kDistanceLpfAlpha = 0.3f;

	for (unsigned instance = 0; instance < _distance_sensor_subs.size(); ++instance) {
		distance_sensor_s msg{};

		if (!_distance_sensor_subs[instance].update(&msg)
		    || !PX4_ISFINITE(msg.current_distance)
		    || msg.current_distance < 0.f
		    || msg.timestamp == 0) {
			continue;
		}

		if (msg.orientation == distance_sensor_s::ROTATION_FORWARD_FACING) {
			if (!_lpf_front_initialized) {
				_front_distance_lpf = msg.current_distance;
				_lpf_front_initialized = true;

			} else {
				_front_distance_lpf += kDistanceLpfAlpha * (msg.current_distance - _front_distance_lpf);
			}

			_front_wall_distance_m = _front_distance_lpf;
			_front_distance_ts = msg.timestamp;
			_front_sensor_instance = static_cast<int8_t>(instance);

		} else if (msg.orientation == distance_sensor_s::ROTATION_UPWARD_FACING) {
			// Suction/contact rangefinder is mounted along body -Z. After the
			// 90-degree pitch it becomes the wall-normal contact measurement.
			if (!_lpf_top_initialized) {
				_top_distance_lpf = msg.current_distance;
				_lpf_top_initialized = true;

			} else {
				_top_distance_lpf += kDistanceLpfAlpha * (msg.current_distance - _top_distance_lpf);
			}

			_top_wall_distance_m = _top_distance_lpf;
			_top_distance_ts = msg.timestamp;
			_top_sensor_instance = static_cast<int8_t>(instance);
		}
	}
}

// ==========================================================================
//  User input helpers
// ==========================================================================

bool WallPerch::user_start_requested()
{
	if (!_param_wp_enable.get()) { return false; }

	manual_control_setpoint_s manual{};
	if (!_manual_control_setpoint_sub.copy(&manual)) { return false; }

	_aux1_raw = manual.aux1; _aux2_raw = manual.aux2;
	_aux3_raw = manual.aux3; _aux4_raw = manual.aux4;

	float val = 0.f;
	switch (_param_wp_aux_ch.get()) {
	case 1:  val = manual.aux1; break;
	case 2:  val = manual.aux2; break;
	case 3:  val = manual.aux3; break;
	case 4:  val = manual.aux4; break;
	default: val = manual.aux1; break;
	}
	return val > 0.3f;
}

bool WallPerch::user_detach_requested()
{
	manual_control_setpoint_s manual{};
	if (!_manual_control_setpoint_sub.copy(&manual)) { return false; }
	float val = 0.f;
	switch (_param_wp_detach_aux_ch.get()) {
	case 1:  val = manual.aux1; break;
	case 2:  val = manual.aux2; break;
	case 3:  val = manual.aux3; break;
	case 4:  val = manual.aux4; break;
	default: val = manual.aux2; break;
	}
	return val > 0.3f;
}

bool WallPerch::user_cancel_requested()
{
	manual_control_setpoint_s manual{};
	if (!_manual_control_setpoint_sub.copy(&manual)) { return false; }
	float val = 0.f;
	switch (_param_wp_cancel_aux_ch.get()) {
	case 1:  val = manual.aux1; break;
	case 2:  val = manual.aux2; break;
	case 3:  val = manual.aux3; break;
	case 4:  val = manual.aux4; break;
	default: val = manual.aux3; break;
	}
	return val > 0.3f;
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

bool WallPerch::top_distance_sufficient() const
{
	return top_sensor_valid() && _top_wall_distance_m <= _param_wp_top_ct_dist.get();
}

bool WallPerch::top_contact_ready()
{
	// The body -Z sensor is meaningful as a wall-normal contact signal only
	// after the measured attitude has reached the wall orientation and the state
	// machine has entered wall-distance control. A sustained close-distance
	// reading means the current motor command is sufficient and can be held.
	const bool wall_distance_control_active = _state == State::WALL_CAPTURE
						  || _state == State::WALL_HOLD
						  || _state == State::WALL_PIN;
	return hold_condition(
		wall_distance_control_active && top_distance_sufficient(),
		_top_contact_start,
		_param_wp_top_ct_hld.get()
	);
}

// ==========================================================================
//  Sensor validity
// ==========================================================================

bool WallPerch::sensors_valid()
{
	// The forward signal is required only until it has triggered the rotation.
	// From FLIP_TO_WALL onward it is rotating away from the wall, so only the
	// body -Z/contact signal remains geometrically meaningful.
	switch (_state) {
	case State::FLIP_TO_WALL:
	case State::WALL_CAPTURE:
	case State::WALL_HOLD:
	case State::WALL_PIN:
	case State::DETACH_ROTATE:
	case State::RECOVER:
		return top_sensor_valid();

	default:
		return front_sensor_valid() && top_sensor_valid();
	}
}

bool WallPerch::front_sensor_valid() const
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime timeout_us = (hrt_abstime)(_param_wp_sens_timeout.get() * 1e6f);
	return _front_distance_ts != 0 && _front_distance_ts <= now && now - _front_distance_ts <= timeout_us;
}

bool WallPerch::top_sensor_valid() const
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime timeout_us = (hrt_abstime)(_param_wp_sens_timeout.get() * 1e6f);
	return _top_distance_ts != 0 && _top_distance_ts <= now && now - _top_distance_ts <= timeout_us;
}

bool WallPerch::local_position_valid() const
{
	const hrt_abstime now = hrt_absolute_time();
	static constexpr hrt_abstime kLocalPositionTimeoutUs = 500000;
	return _local_position_data_valid && _local_position_ts != 0 && _local_position_ts <= now
	       && now - _local_position_ts <= kLocalPositionTimeoutUs;
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
		if (!local_position_valid()) { return false; }
		if (!rate_safe()) { return false; }
		if (!vz_safe()) { return false; }
	}
	return true;
}

bool WallPerch::rate_safe()
{
	float max_rate = _param_wp_max_rate.get();
	return (fabsf(_angular_velocity(0)) <= max_rate &&
		fabsf(_angular_velocity(1)) <= max_rate &&
		fabsf(_angular_velocity(2)) <= max_rate);
}

bool WallPerch::vz_safe()
{
	// vz is NED: positive = down
	return _velocity(2) <= _param_wp_max_vz_down.get();
}

bool WallPerch::attitude_recovered()
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

float WallPerch::compute_approach_thrust(float dt, float pitch_rad)
{
	// Positive altitude error means the aircraft is below the altitude latched
	// at switch-on. NED vz is positive downward, so both terms must increase
	// thrust. The small tilt compensation preserves the vertical component
	// while the negative approach pitch generates forward acceleration.
	_altitude_error = _altitude_hold_sp - _current_altitude;

	const float ki = _param_wp_alt_ki.get();
	const float max_adjustment = _param_wp_alt_max.get();

	if (ki > FLT_EPSILON) {
		const float integral_limit = max_adjustment / ki;
		_altitude_error_integral = math::constrain(
			_altitude_error_integral + _altitude_error * dt,
			-integral_limit, integral_limit);

	} else {
		_altitude_error_integral = 0.f;
	}

	float adjustment = _param_wp_alt_kp.get() * _altitude_error
			   + ki * _altitude_error_integral
			   + _param_wp_alt_kd.get() * _velocity(2);
	adjustment = math::constrain(adjustment, -max_adjustment, max_adjustment);

	const float vertical_fraction = math::max(cosf(fabsf(pitch_rad)), 0.7f);
	return math::constrain((_hover_thrust + adjustment) / vertical_fraction, 0.1f, 0.9f);
}

void WallPerch::update_wall_thrust_command(float dt)
{
	if (top_distance_sufficient()) {
		return;
	}

	// A distance above the target means that the present RPM is insufficient.
	// Increase monotonically; once the measured distance reaches the target,
	// stop at the achieved value rather than always continuing to WP_PIN_THR.
	const float max_thrust = math::max(_hover_thrust,
					   math::constrain(_param_wp_pin_thr.get(), 0.f, 1.f));
	const float ramp_time = math::max(_param_wp_capture_time.get(), 0.05f);
	const float thrust_rate = (max_thrust - _hover_thrust) / ramp_time;
	_pin_thrust_command = math::constrain(_pin_thrust_command + thrust_rate * dt,
					      _hover_thrust, max_thrust);
}

bool WallPerch::wall_thrust_at_max() const
{
	const float max_thrust = math::max(_hover_thrust,
					   math::constrain(_param_wp_pin_thr.get(), 0.f, 1.f));
	return _pin_thrust_command >= max_thrust - 0.001f;
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
	status.active = ((_state >= State::SLOW_APPROACH && _state < State::EXIT) || _state == State::ABORT);
	status.front_ready = front_sensor_valid() && _front_wall_distance_m <= _param_wp_frn_rd_dist.get();
	status.flip_ready = _flip_ready_confirmed;
	status.top_contact_ready = _top_contact_confirmed;
	status.front_sensor_valid = front_sensor_valid();
	status.top_sensor_valid = top_sensor_valid();
	status.local_position_valid = local_position_valid();
	status.altitude_hold_active = _state == State::SLOW_APPROACH;
	status.wall_attitude_ready = wall_attitude_ready();
	status.motor_boost_active = (_state == State::WALL_CAPTURE || _state == State::WALL_HOLD
				     || _state == State::WALL_PIN)
				    && _pin_thrust_command > _hover_thrust + 0.001f;
	status.front_sensor_instance = _front_sensor_instance;
	status.top_sensor_instance = _top_sensor_instance;
	status.front_wall_distance_m = _front_wall_distance_m;
	status.top_wall_distance_m = _top_wall_distance_m;
	status.altitude_setpoint_m = _altitude_hold_sp;
	status.altitude_m = _current_altitude;
	status.altitude_error_m = _altitude_error;
	status.progress = 0.f;
	status.thrust_norm = _last_thrust_norm;
	status.failsafe_triggered = status.active && !safety_ok();
	_status_pub.publish(status);
}

bool WallPerch::wall_attitude_ready() const
{
	// Measured pitch, not merely commanded pitch, must reach the wall angle
	// before body -Z is accepted as the contact signal.
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
	case State::SLOW_APPROACH:
		// Switch-on takeover is immediate: latch the current ALTCTL altitude and
		// heading here instead of entering a level-hover stabilization state.
		_yaw_hold = _current_yaw;
		_q_hover = compute_hover_attitude(_yaw_hold);
		_q_wall = compute_wall_attitude(_q_hover);
		_hover_thrust = _hover_thrust_estimate_valid ? _hover_thrust_estimate : _param_wp_thr_hover.get();
		_hover_thrust = math::constrain(_hover_thrust, 0.1f, 0.9f);
		_altitude_hold_sp = _current_altitude;
		_altitude_error = 0.f;
		_altitude_error_integral = 0.f;
		_flip_ready_start = 0;
		_flip_ready_confirmed = false;
		_top_contact_start = 0;
		_top_contact_confirmed = false;
		_approach_start_time = hrt_absolute_time();
		mavlink_log_info(&_mavlink_log_pub,
			"[wall_perch] takeover: alt=%.2f hover=%.3f",
			(double)_altitude_hold_sp, (double)_hover_thrust);
		break;

	case State::FLIP_TO_WALL:
		_flip_start_time = hrt_absolute_time();
		_q_flip_start = _q_current;
		break;

	case State::WALL_CAPTURE:
		_top_contact_start = 0;
		_top_contact_confirmed = false;
		_motor_insufficient_start = 0;
		_pin_thrust_command = _hover_thrust;
		break;

	case State::DETACH_ROTATE:
		_detach_start_time = hrt_absolute_time();
		break;

	case State::WALL_PIN:
		_pin_start_time = hrt_absolute_time();
		_motor_insufficient_start = 0;
		mavlink_log_info(&_mavlink_log_pub,
			"[wall_perch] motor thrust sufficient at %.3f; raw hold enabled",
			(double)_pin_thrust_command);
		break;

	case State::ABORT:
		PX4_WARN("[wall_perch] ABORT — recovering to hover");
		_top_contact_confirmed = false;
		_motor_insufficient_start = 0;
		break;

	case State::EXIT:
		// Clear all timers
		_front_ready_start = 0;
		_flip_ready_start = 0;
		_top_contact_start = 0;
		_motor_insufficient_start = 0;
		_flip_ready_confirmed = false;
		_top_contact_confirmed = false;
		_altitude_error = 0.f;
		_altitude_error_integral = 0.f;
		_last_thrust_norm = 0.f;
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
	const bool start_switch_on = user_start_requested();
	const bool start_req = start_switch_on && !_start_switch_was_on;
	_start_switch_was_on = start_switch_on;
	bool detach_req = user_detach_requested();
	bool cancel_req = user_cancel_requested();
	bool safe = safety_ok();

	// --- Global cancel / abort triggers ---
	if (cancel_req && _state > State::IDLE && _state < State::EXIT) {
		PX4_WARN("[wall_perch] User cancel requested");
		enter_state(State::ABORT);
	}

	// --- State behaviors ---
	switch (_state) {

	// ================================================================
	// IDLE
	// ================================================================
	case State::IDLE: {
		if (start_req && _armed && safe &&
		    _current_altitude > _param_wp_min_alt.get()) {
			enter_state(State::SLOW_APPROACH);
		}
		break;
	}

	// ================================================================
	// Legacy states — retained for stable status enum values only. They no
	// longer insert a hover/recovery delay after the switch is turned on.
	// ================================================================
	case State::FRONT_WALL_DETECT: {
		enter_state(State::SLOW_APPROACH);
		break;
	}

	case State::STABILIZE_HOVER: {
		enter_state(State::SLOW_APPROACH);
		break;
	}

	// ================================================================
	// SLOW_APPROACH — tilt forward slightly, move toward wall
	// ================================================================
	case State::SLOW_APPROACH: {
		if (cancel_req) { enter_state(State::ABORT); break; }
		if (!safe)      { enter_state(State::ABORT); break; }

		// Timeout check
		if (hrt_elapsed_time(&_approach_start_time) >
		    (hrt_abstime)(_param_wp_appr_timeout.get() * 1e6f)) {
			PX4_WARN("[wall_perch] Approach timeout");
			enter_state(State::ABORT); break;
		}

		// Nose-down pitch moves the aircraft forward. Reduce the pitch as the
		// front sensor approaches the flip threshold to limit overshoot.
		const float slow_distance = math::max(_param_wp_frn_rd_dist.get(),
						   _param_wp_flp_trd_dist.get() + 0.01f);
		const float distance_span = slow_distance - _param_wp_flp_trd_dist.get();
		const float approach_scale = math::constrain(
			(_front_wall_distance_m - _param_wp_flp_trd_dist.get()) / distance_span,
			0.25f, 1.f);
		const float approach_pitch = -math::radians(fabsf(_param_wp_appr_pitch.get())) * approach_scale;
		Quatf q_approach = _q_hover * Quatf(Eulerf(0.f, approach_pitch, 0.f));
		publish_attitude_setpoint(q_approach, compute_approach_thrust(dt, approach_pitch));

		_flip_ready_confirmed = flip_ready();

		if (_flip_ready_confirmed) {
			enter_state(State::FLIP_TO_WALL);
		}
		break;
	}

	// ================================================================
	// FLIP_TO_WALL — slerp from hover to wall attitude
	// ================================================================
	case State::FLIP_TO_WALL: {
		if (cancel_req) { enter_state(State::ABORT); break; }
		if (!safe)      { enter_state(State::ABORT); break; }

		float duration = _param_wp_flip_time.get();
		float elapsed  = (float)hrt_elapsed_time(&_flip_start_time) * 1e-6f;
		float tau = math::constrain(elapsed / duration, 0.f, 1.f);
		float s = smoothstep5(tau);

		// Continue from the measured approach attitude. Starting again from the
		// level reference would create the unwanted recovery "kick" at takeover.
		Quatf q_des = slerp_quat(_q_flip_start, _q_wall, s);
		publish_attitude_setpoint(q_des, _hover_thrust);

		// The distance-control ramp starts only after the commanded interpolation
		// and the measured wall attitude are both ready.
		if (tau >= 1.f && wall_attitude_ready()) {
			enter_state(State::WALL_CAPTURE);

		} else if (elapsed > duration + _param_wp_contact_tmo.get()) {
			PX4_WARN("[wall_perch] Wall attitude not reached");
			enter_state(State::ABORT);
		}
		break;
	}

	// ================================================================
	// WALL_CAPTURE — ramp wall-normal thrust while body -Z measures distance.
	// Reaching the target confirms that the current RPM is sufficient.
	// ================================================================
	case State::WALL_CAPTURE: {
		if (cancel_req) { enter_state(State::ABORT); break; }
		if (!safe)      { enter_state(State::ABORT); break; }

		update_wall_thrust_command(dt);
		publish_attitude_setpoint(_q_wall, _pin_thrust_command);
		_top_contact_confirmed = top_contact_ready();

		if (_top_contact_confirmed) {
			enter_state(_param_wp_pin_enable.get() ? State::WALL_PIN : State::WALL_HOLD);
			break;
		}

		// If even the configured maximum RPM cannot reach the requested distance,
		// recover rather than remaining at maximum indefinitely.
		if (hold_condition(wall_thrust_at_max() && !top_distance_sufficient(),
				   _motor_insufficient_start, _param_wp_contact_tmo.get())) {
			PX4_WARN("[wall_perch] Maximum wall thrust insufficient");
			enter_state(State::ABORT);
		}
		break;
	}

	// ================================================================
	// WALL_HOLD — maintain wall attitude and thrust
	// ================================================================
	case State::WALL_HOLD: {
		if (cancel_req) { enter_state(State::ABORT); break; }
		if (!safe)      { enter_state(State::ABORT); break; }

		// Pin bypass is disabled in this state, but the same distance feedback is
		// used: increase only while the measured distance says RPM is insufficient.
		update_wall_thrust_command(dt);
		publish_attitude_setpoint(_q_wall, _pin_thrust_command);
		_top_contact_confirmed = top_contact_ready();

		if (hold_condition(wall_thrust_at_max() && !top_distance_sufficient(),
				   _motor_insufficient_start, _param_wp_contact_tmo.get())) {
			PX4_WARN("[wall_perch] Maximum wall thrust insufficient");
			enter_state(State::ABORT);
			break;
		}

		// Detach on user request or hold timeout
		if (detach_req) {
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
	// WALL_PIN — contact-confirmed mixer bypass: command all four motors,
	// do NOT publish an attitude setpoint (no attitude recovery).
	// ControlAllocator reads wall_perch_status and suppresses its own output.
	// ================================================================
	case State::WALL_PIN: {
		if (!_armed || !top_sensor_valid()) {
			enter_state(State::ABORT);
			break;
		}

		// Continue using body -Z distance after pinning. If distance grows, the
		// previously sufficient RPM is no longer enough and is increased again.
		update_wall_thrust_command(dt);
		_top_contact_confirmed = top_contact_ready();
		publish_actuator_motors(_pin_thrust_command);

		if (hold_condition(wall_thrust_at_max() && !top_distance_sufficient(),
				   _motor_insufficient_start, _param_wp_contact_tmo.get())) {
			PX4_WARN("[wall_perch] Maximum wall thrust insufficient");
			enter_state(State::ABORT);
			break;
		}

		// Stay pinned until the user cancels (recovers) or the optional
		// hold timeout elapses.
		if (cancel_req) {
			enter_state(State::ABORT); break;
		}

		if (_param_wp_pin_hold.get() > 0.f &&
		    hrt_elapsed_time(&_pin_start_time) > (hrt_abstime)(_param_wp_pin_hold.get() * 1e6f)) {
			enter_state(State::ABORT); break;
		}
		break;
	}

	// ================================================================
	// DETACH_ROTATE — slerp back to hover, schedule thrust
	// ================================================================
	case State::DETACH_ROTATE: {
		if (cancel_req) { enter_state(State::ABORT); break; }
		if (!_armed || !sensors_valid()) { enter_state(State::ABORT); break; }

		float duration = _param_wp_detach_time.get();
		float elapsed  = (float)hrt_elapsed_time(&_detach_start_time) * 1e-6f;
		float tau = math::constrain(elapsed / duration, 0.f, 1.f);
		float s = smoothstep5(tau);

		Quatf q_des = slerp_quat(_q_wall, _q_hover, s);

		// Thrust schedule: interpolate from hold (1.5*hover) to recover (1.1*hover)
		float thrust = _hover_thrust * 1.5f +
			       s * (_hover_thrust * 1.1f - _hover_thrust * 1.5f);

		publish_attitude_setpoint(q_des, thrust);

		if (tau >= 1.f) {
			enter_state(State::RECOVER);
		}
		break;
	}

	// ================================================================
	// RECOVER — hover and wait for stability
	// ================================================================
	case State::RECOVER: {
		if (cancel_req) { enter_state(State::ABORT); break; }
		if (!_armed || !sensors_valid()) { enter_state(State::ABORT); break; }

		publish_attitude_setpoint(_q_hover, _hover_thrust * 1.1f); // WP_THR_RECOVER = 1.1 * hover

		if (hrt_elapsed_time(&_state_entry_time) > (hrt_abstime)(_param_wp_recover_time.get() * 1e6f) &&
		    attitude_recovered() && rate_safe() && vz_safe() &&
		    !top_contact_ready()) {
			enter_state(State::EXIT);
		}
		break;
	}

	// ================================================================
	// EXIT — release control
	// ================================================================
	case State::EXIT: {
		// Stop publishing attitude setpoint
		// Reset all timers
		_front_ready_start = 0;
		_flip_ready_start = 0;
		_top_contact_start = 0;

		mavlink_log_info(&_mavlink_log_pub, "[wall_perch] EXIT, releasing control");
		_state = State::IDLE;
		break;
	}

	// ================================================================
	// ABORT — emergency recovery, don't drop thrust
	// ================================================================
	case State::ABORT: {
		// Publish hover attitude with recovery thrust
		publish_attitude_setpoint(_q_hover, _hover_thrust * 1.1f); // WP_THR_RECOVER = 1.1 * hover

		if (attitude_recovered() && rate_safe() && vz_safe()) {
			// Hold a bit longer to ensure stability
			if (hrt_elapsed_time(&_state_entry_time) > (hrt_abstime)(_param_wp_recover_time.get() * 1e6f)) {
				enter_state(State::EXIT);
			} else {
				// keep recovering
			}
		}
		// NOTE: If never recovers, rely on pilot takeover or PX4 failsafe
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
