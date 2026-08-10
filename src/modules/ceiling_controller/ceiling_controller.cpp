#include "ceiling_controller.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace matrix;



//===================================================================
//  CeilingController — nav_and_controllers WQ, subscribes distance_sensor
//===================================================================

CeilingController::CeilingController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
}

CeilingController::~CeilingController()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool CeilingController::init()
{
	parameters_update(true);
	
	// 检查 distance_sensor topic 是否可用
	if (!_distance_sensor_subs.advertised()) {
		PX4_WARN("[CeilCtrl] distance_sensor topic not advertised yet");
	} else {
		PX4_INFO("[CeilCtrl] distance_sensor instances available: %u",
			 (unsigned)_distance_sensor_subs.advertised_count());
	}
	
	PX4_INFO("[CeilCtrl] Started, selecting body -Z distance_sensor");
	ScheduleOnInterval(10_ms);
	_first_run = true;
	return true;
}

void CeilingController::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

void CeilingController::read_switches(bool &ceiling_arm, bool &detach)
{
	manual_control_setpoint_s manual{};
	if (_manual_control_setpoint_sub.copy(&manual)) {
		_aux1_raw = manual.aux1;
		_aux2_raw = manual.aux2;
		ceiling_arm = manual.aux1 > 0.3f;
		detach     = manual.aux2 > 0.3f;
	}
}

float CeilingController::compute_distance_control(float dt)
{
	float d0 = _param_ceil_d0.get();
	float comp_tgt = _param_ceil_comp_tgt.get();
	float target_dist = d0 - comp_tgt;
	if (_state == ceiling_contact_status_s::DETACH_MODE) {
		float ramp_rate = _param_ceil_ramp_dist.get();
		_target_distance += ramp_rate * dt;
		if (_target_distance > d0) { _target_distance = d0; }
		target_dist = _target_distance;
	} else { _target_distance = target_dist; }

	float error = _ceiling_distance_lpf - target_dist;
	_dist_error_integral += error * dt;
	_dist_error_integral = math::constrain(_dist_error_integral, -1.f, 1.f);

	float derivative = 0.f;
	if (dt > 1e-4f) { derivative = (error - _dist_error_prev) / dt; }
	_dist_error_prev = error;

	float pid_out = _param_ceil_dist_kp.get() * error
		+ _param_ceil_dist_ki.get() * _dist_error_integral
		+ _param_ceil_dist_kd.get() * derivative;

	float baseline = -_hover_thrust;
	if (_attach_baseline_initialized
	    && (_state == ceiling_contact_status_s::ATTACH_CONTROL_MODE
		|| _state == ceiling_contact_status_s::SURFACE_MANUAL_MODE)) {
		baseline = _attach_baseline_thrust;
	}

	float thrust_z = baseline - pid_out;
	if (_state == ceiling_contact_status_s::ATTACH_CONTROL_MODE
	    || _state == ceiling_contact_status_s::SURFACE_MANUAL_MODE) {
		float thrust_floor = -math::min(_hover_thrust * 1.5f, 0.95f);
		thrust_z = math::min(thrust_z, thrust_floor);
	}
	float max_thrust = -math::max(_param_ceil_max_thrust.get(), 0.05f);
	thrust_z = math::constrain(thrust_z, max_thrust, -_hover_thrust);
	return thrust_z;
}

float CeilingController::compute_lock_thrust()
{
	// Lock body-Z thrust to CEIL_ATTACH_MULT x hover to stick to the ceiling.
	float lock = math::min(_recorded_hover_thrust * _param_ceil_attach_mult.get(), 0.95f);
	return -lock;
}

bool CeilingController::check_attach_confirmed()
{
	float compression = _param_ceil_d0.get() - _ceiling_distance_lpf;
	if (compression >= COMPRESSION_THR) {
		if (_attach_detect_start == 0) { _attach_detect_start = hrt_absolute_time(); }
		if (hrt_elapsed_time(&_attach_detect_start) >= ATTACH_TIME_THR) return true;
	} else { _attach_detect_start = 0; }
	return false;
}

bool CeilingController::check_distance_stable()
{
	float target = _param_ceil_d0.get() - _param_ceil_comp_tgt.get();
	if (fabsf(_ceiling_distance_lpf - target) <= 0.01f) {
		if (_dist_stable_start == 0) { _dist_stable_start = hrt_absolute_time(); }
		if (hrt_elapsed_time(&_dist_stable_start) >= (hrt_abstime)(_param_ceil_stable_t.get() * 1e6f)) return true;
	} else { _dist_stable_start = 0; }
	return false;
}

bool CeilingController::check_fault_conditions()
{
	float max_roll  = math::radians(_param_ceil_max_roll.get());
	float max_pitch = math::radians(_param_ceil_max_pitch.get());
	if (fabsf(_attitude_euler.phi()) > max_roll || fabsf(_attitude_euler.theta()) > max_pitch) return true;
	if (_state != ceiling_contact_status_s::NORMAL_FLIGHT && !_distance_subscription_active) return true;
	if (_state >= ceiling_contact_status_s::ATTACH_CONTROL_MODE
	    && _state <= ceiling_contact_status_s::SURFACE_MANUAL_MODE)
		if (_ceiling_distance_lpf > _param_ceil_d0.get() + 0.08f) return true;
	return false;
}

void CeilingController::enter_state(uint8_t new_state)
{
	if (_state == new_state) return;
	PX4_INFO("Ceiling state: %d -> %d", _state, new_state);
	_state = new_state; _state_entry_time = hrt_absolute_time();

	if (_state == ceiling_contact_status_s::CEILING_ARM_MODE) {
		_attach_detect_start = 0; _dist_stable_start = 0;
		_dist_error_integral = 0.f; _dist_error_prev = 0.f;
		_target_distance = _param_ceil_d0.get() - _param_ceil_comp_tgt.get();
	}
	if (_state == ceiling_contact_status_s::APPROACH_MODE) {
		_attach_detect_start = 0;
		_dist_error_integral = 0.f; _dist_error_prev = 0.f;
	}
	if (_state == ceiling_contact_status_s::ATTACH_CONTROL_MODE) {
		_dist_stable_start = 0;
		vehicle_thrust_setpoint_s vts{};
		if (_vehicle_thrust_setpoint_sub.copy(&vts)) {
			_attach_baseline_thrust = math::constrain(vts.xyz[2], -0.95f, -0.05f);
			_attach_baseline_initialized = true;
		} else { _attach_baseline_thrust = -_hover_thrust; _attach_baseline_initialized = true; }
	}
	if (_state == ceiling_contact_status_s::DETACH_MODE
	    || _state == ceiling_contact_status_s::NORMAL_FLIGHT) _attach_baseline_initialized = false;
	if (_state == ceiling_contact_status_s::DETACH_MODE) {
		_target_distance = _param_ceil_d0.get() - _param_ceil_comp_tgt.get();
		_dist_error_integral = 0.f; _dist_error_prev = 0.f;
	}
}

void CeilingController::update_state_machine(float dt)
{
	read_switches(_ceiling_arm_switch, _detach_switch);
	bool fault = check_fault_conditions();

	if (_detach_switch && _state >= ceiling_contact_status_s::ATTACH_CONTROL_MODE
	    && _state <= ceiling_contact_status_s::SURFACE_MANUAL_MODE) {
		enter_state(ceiling_contact_status_s::DETACH_MODE);
	}
	if (fault && _state >= ceiling_contact_status_s::ATTACH_CONTROL_MODE
	    && _state <= ceiling_contact_status_s::SURFACE_MANUAL_MODE) {
		_fault_count++;
		enter_state(ceiling_contact_status_s::DETACH_MODE);
	}

	switch (_state) {
	case ceiling_contact_status_s::NORMAL_FLIGHT:
		if (_ceiling_arm_switch) { enter_state(ceiling_contact_status_s::CEILING_ARM_MODE); }
		break;
	case ceiling_contact_status_s::CEILING_ARM_MODE:
		if (!_ceiling_arm_switch) { enter_state(ceiling_contact_status_s::NORMAL_FLIGHT); }
		else if (_ceiling_distance_lpf < _param_ceil_dist_thr.get()
			 && fabsf(_velocity(2)) < _param_ceil_vel_thr.get()) {
			enter_state(ceiling_contact_status_s::APPROACH_MODE);
		}
		break;
	case ceiling_contact_status_s::APPROACH_MODE:
		if (!_ceiling_arm_switch || !_distance_subscription_active) {
			enter_state(ceiling_contact_status_s::NORMAL_FLIGHT);
		} else if (hrt_elapsed_time(&_state_entry_time) > (hrt_abstime)(_param_ceil_appr_to.get() * 1000ULL)) {
			enter_state(ceiling_contact_status_s::NORMAL_FLIGHT);
		} 		else if (hrt_elapsed_time(&_state_entry_time) >= (hrt_abstime)(_param_ceil_appr_hover.get() * 1000ULL)
			 && _ceiling_distance_lpf <= _param_ceil_d0.get() + 0.02f) {
			enter_state(ceiling_contact_status_s::ATTACH_CONTROL_MODE);
		}
		break;
	case ceiling_contact_status_s::ATTACH_CONTROL_MODE:
		if (!_ceiling_arm_switch) { enter_state(ceiling_contact_status_s::DETACH_MODE); }
		else if (hrt_elapsed_time(&_state_entry_time) >= 500_ms
			 && _ceiling_distance_lpf <= _param_ceil_d0.get() + 0.02f) {
			enter_state(ceiling_contact_status_s::SURFACE_MANUAL_MODE);
		}
		break;
	case ceiling_contact_status_s::SURFACE_MANUAL_MODE:
		if (!_ceiling_arm_switch) { enter_state(ceiling_contact_status_s::DETACH_MODE); }
		break;
	case ceiling_contact_status_s::DETACH_MODE:
		// Constant-velocity descent is commanded in Run(). Leave DETACH (hand back to POSCTL)
		// when the aircraft has dropped far enough from the ceiling, or on a safety timeout.
		if (_ceiling_distance_lpf >= _param_ceil_dist_thr.get()
		    || hrt_elapsed_time(&_state_entry_time) >= (hrt_abstime)(_param_ceil_detach_to.get() * 1000ULL)) {
			enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
		}
		break;
	case ceiling_contact_status_s::RECOVERY_HOVER_MODE:
		if (hrt_elapsed_time(&_state_entry_time) > 2_s) {
			enter_state(ceiling_contact_status_s::NORMAL_FLIGHT);
		}
		break;
	}
}

void CeilingController::Run()
{
	perf_begin(_loop_perf); perf_count(_loop_interval_perf);
	const hrt_abstime now = hrt_absolute_time();

	if (_first_run) { _first_run = false;
		mavlink_log_info(&_mavlink_log_pub, "[CeilCtrl] STARTED, subscribing to distance_sensor"); }

	parameters_update(false);

	// Select only the body -Z/upward sensor. The new body +X sensor shares
	// distance_sensor as another uORB instance and must never enter this loop.
	for (unsigned instance = 0; instance < _distance_sensor_subs.size(); ++instance) {
		distance_sensor_s distance_msg{};

		if (_distance_sensor_subs[instance].update(&distance_msg)
		    && distance_msg.orientation == distance_sensor_s::ROTATION_UPWARD_FACING
		    && PX4_ISFINITE(distance_msg.current_distance)
		    && distance_msg.current_distance >= 0.f) {
			_ceiling_distance = distance_msg.current_distance;
			_distance_sensor_timestamp = distance_msg.timestamp;
			_distance_sensor_instance = static_cast<int8_t>(instance);

			if (!_lpf_initialized) {
				_ceiling_distance_lpf = _ceiling_distance;
				_lpf_initialized = true;

			} else {
				float alpha = 0.2f;
				_ceiling_distance_lpf += alpha * (_ceiling_distance - _ceiling_distance_lpf);
			}
		}
	}

	static constexpr hrt_abstime kDistanceTimeout = 500_ms;
	_distance_subscription_active = _distance_sensor_timestamp != 0
		&& _distance_sensor_timestamp <= now
		&& now - _distance_sensor_timestamp <= kDistanceTimeout;

	vehicle_local_position_s local_pos{};
	if (_vehicle_local_pos_sub.copy(&local_pos)) {
		_velocity(0) = local_pos.vx; _velocity(1) = local_pos.vy; _velocity(2) = local_pos.vz;
	}
	hover_thrust_estimate_s hte{};
	if (_hover_thrust_estimate_sub.copy(&hte) && hte.valid)
		_hover_thrust = math::constrain(hte.hover_thrust, 0.1f, 0.9f);
	vehicle_attitude_s att{};
	if (_vehicle_attitude_sub.copy(&att)) { Quatf q(att.q); _attitude_euler = Eulerf(q); }

	_compression = _param_ceil_d0.get() - _ceiling_distance_lpf;

	static hrt_abstime last_run{0};
	float dt = 0.01f;
	if (last_run != 0) dt = math::constrain((now - last_run) / 1e6f, 0.001f, 0.05f);
	last_run = now;

	update_state_machine(dt);

	float thrust_body_z_sp = NAN, approach_vz_sp = NAN;
	bool integral_reset_request = false, wheel_stop_request = false;

	if (_state == ceiling_contact_status_s::APPROACH_MODE) {
		// APPROACH is velocity-controlled (no thrust override, no RC dependency).
		// First the module commands a hover (velocity setpoint = 0) for CEIL_APPR_HOVER
		// seconds to let the aircraft settle and the hover thrust estimate converge,
		// then it climbs at CEIL_APPR_VZ toward the ceiling. The hover thrust is recorded
		// during the hover phase so ATTACH/SURFACE lock thrust uses a settled value.
		const float elapsed_s = hrt_elapsed_time(&_state_entry_time) / 1e6f;
		if (elapsed_s < _param_ceil_appr_hover.get()) {
			approach_vz_sp = 0.f;  // hover, hold altitude (module-commanded)
			_recorded_hover_thrust = _hover_thrust;
		} else {
			// Ramp from hover to full climb speed over a short interval. A step change
			// from 0 to -CEIL_APPR_VZ would abruptly activate the z_deriv velocity-state
			// blending in mc_pos_control (line 574-583), causing a measurement discontinuity
			// that makes the aircraft dip before starting to climb. Ramping avoids this.
			const float ramp_duration = 0.5f;  // seconds
			const float target_vz = -math::constrain(_param_ceil_appr_vz.get(), 0.05f, 1.0f);
			const float ramp_t = elapsed_s - _param_ceil_appr_hover.get();
			if (ramp_t < ramp_duration) {
				approach_vz_sp = target_vz * (ramp_t / ramp_duration);
			} else {
				approach_vz_sp = target_vz;
			}
		}
	} else if (_state == ceiling_contact_status_s::DETACH_MODE) {
		// DETACH: constant-velocity descent away from the ceiling. NED z is down-positive,
		// so a positive vz_sp means descending. No thrust override is published, so the Z
		// integrator is preserved and the aircraft peels off smoothly as thrust drops below hover.
		approach_vz_sp = math::max(_param_ceil_detach_vz.get(), 0.05f);
	}
	// Build the desired override thrust for the lock states. ATTACH requests 1.5x,
	// SURFACE requests 1.8x hover. The actual published value is first-order filtered
	// so entering/leaving these states ramps smoothly instead of stepping from ~1x to
	// 1.8x (which would slam the aircraft into the ceiling or drop it on release).
	float override_target = NAN;  // NAN = no override (POSCTL/mc_pos_control in control)
	if (_state == ceiling_contact_status_s::ATTACH_CONTROL_MODE) {
		override_target = compute_lock_thrust();  // negative (up)
	}
	if (_state == ceiling_contact_status_s::SURFACE_MANUAL_MODE) {
		float lock = math::min(_recorded_hover_thrust * _param_ceil_surf_mult.get(), 0.95f);
		override_target = -lock;
	}
	float hover_thr = _recorded_hover_thrust > 0.05f ? _recorded_hover_thrust : _hover_thrust;
	if (PX4_ISFINITE(override_target)) {
		if (!PX4_ISFINITE(_thrust_override_filt)) { _thrust_override_filt = -hover_thr; }
		// First-order lag toward target, time constant CEIL_RAMP_T.
		const float tc = math::max(_param_ceil_ramp_t.get(), 0.05f);
		const float alpha = math::constrain(dt / tc, 0.f, 1.f);
		_thrust_override_filt += alpha * (override_target - _thrust_override_filt);
		thrust_body_z_sp = _thrust_override_filt;
		// mc_pos_control applies this thrust directly; keep its Z integrator cleared so
		// the two control paths do not fight. XY (pilot) integrator is preserved.
		integral_reset_request = true;
	} else {
		// No override in this state: glide the filtered value back to hover so that the
		// next time we need it the step is small, and let POSCTL fully own Z.
		if (PX4_ISFINITE(_thrust_override_filt)) {
			float tc = math::max(_param_ceil_ramp_t.get(), 0.05f);
			float alpha = math::constrain(dt / tc, 0.f, 1.f);
			_thrust_override_filt += alpha * (-hover_thr - _thrust_override_filt);
			if (fabsf(_thrust_override_filt - (-hover_thr)) < 0.01f) { _thrust_override_filt = NAN; }
		}
	}
	if (_state == ceiling_contact_status_s::DETACH_MODE || check_fault_conditions())
		wheel_stop_request = true;

	ceiling_contact_status_s status{};
	status.timestamp = now; status.state = _state;
	status.ceiling_distance = _ceiling_distance_lpf;
	status.target_distance = _target_distance;
	status.compression = _compression;
	status.target_compression = _param_ceil_comp_tgt.get();
	status.thrust_body_z_sp = thrust_body_z_sp;
	status.approach_vz_sp = approach_vz_sp;
	status.ceiling_arm_switch_on = _ceiling_arm_switch;
	status.detach_switch_on = _detach_switch;
	status.attach_confirmed = check_attach_confirmed();
	status.distance_stable = check_distance_stable();
	status.fault_detected = check_fault_conditions();
	status.integral_reset_request = integral_reset_request;
	status.wheel_stop_request = wheel_stop_request;
	status.state_entry_time = _state_entry_time;
	status.fault_count = _fault_count;
	_status_pub.publish(status);

	if (_ceiling_arm_switch != _last_ceiling_arm_switch || _detach_switch != _last_detach_switch) {
		mavlink_log_info(&_mavlink_log_pub, "[CeilCtrl] ARM=%d DET=%d st=%d",
			(int)_ceiling_arm_switch, (int)_detach_switch, (int)_state);
		_last_ceiling_arm_switch = _ceiling_arm_switch; _last_detach_switch = _detach_switch;
	}
	if (hrt_elapsed_time(&_last_status_log_time) > 5_s) {
		mavlink_log_info(&_mavlink_log_pub,
			"[CeilCtrl] st=%d A=%d D=%d dist=%.3f sub=%d inst=%d",
			(int)_state, (int)_ceiling_arm_switch, (int)_detach_switch,
			(double)_ceiling_distance_lpf,
			(int)_distance_subscription_active, (int)_distance_sensor_instance);
		_last_status_log_time = now;
	}
	perf_end(_loop_perf);
}

int CeilingController::task_spawn(int argc, char *argv[])
{
	CeilingController *instance = new CeilingController();
	if (instance) { _object.store(instance); _task_id = task_id_is_work_queue;
		if (instance->init()) return PX4_OK; }
	delete instance; _object.store(nullptr); _task_id = -1; return PX4_ERROR;
}
int CeilingController::custom_command(int, char *[]) { return print_usage("unknown command"); }
int CeilingController::print_usage(const char *reason) {
	if (reason) PX4_WARN("%s\n", reason);
	PRINT_MODULE_USAGE_NAME("ceiling_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start"); PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); return 0; }
extern "C" __EXPORT int ceiling_controller_main(int argc, char *argv[]) { return CeilingController::main(argc, argv); }
