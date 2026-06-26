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
	if (!_distance_sensor_sub.advertised()) {
		PX4_WARN("[CeilCtrl] distance_sensor topic not advertised yet");
	} else {
		PX4_INFO("[CeilCtrl] distance_sensor topic available");
	}
	
	PX4_INFO("[CeilCtrl] Started, subscribing to distance_sensor");
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
		float thrust_floor = -math::min(_hover_thrust * 1.05f, 0.95f);
		thrust_z = math::min(thrust_z, thrust_floor);
	}
	float max_thrust = -math::max(_param_ceil_max_thrust.get(), 0.05f);
	thrust_z = math::constrain(thrust_z, max_thrust, -0.05f);
	return thrust_z;
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
	if (_state >= ceiling_contact_status_s::ATTACH_CONTROL_MODE
	    && _state <= ceiling_contact_status_s::SURFACE_MANUAL_MODE)
		if (_ceiling_distance_lpf > _param_ceil_d0.get() + 0.02f) return true;
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
		if (!_ceiling_arm_switch) { enter_state(ceiling_contact_status_s::NORMAL_FLIGHT); }
		else if (hrt_elapsed_time(&_state_entry_time) > (hrt_abstime)(_param_ceil_appr_to.get() * 1000ULL)) {
			enter_state(ceiling_contact_status_s::NORMAL_FLIGHT);
		} else if (check_attach_confirmed()) {
			enter_state(ceiling_contact_status_s::ATTACH_CONTROL_MODE);
		}
		break;
	case ceiling_contact_status_s::ATTACH_CONTROL_MODE:
		if (!_ceiling_arm_switch) { enter_state(ceiling_contact_status_s::DETACH_MODE); }
		else if (check_distance_stable()) { enter_state(ceiling_contact_status_s::SURFACE_MANUAL_MODE); }
		break;
	case ceiling_contact_status_s::SURFACE_MANUAL_MODE:
		if (!_ceiling_arm_switch) { enter_state(ceiling_contact_status_s::DETACH_MODE); }
		break;
	case ceiling_contact_status_s::DETACH_MODE:
		_target_distance += _param_ceil_ramp_dist.get() * dt;
		if (_target_distance > _param_ceil_d0.get()) { _target_distance = _param_ceil_d0.get(); }
		if (_ceiling_distance_lpf >= _param_ceil_dist_thr.get()) {
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

	// 从 distance_sensor 订阅获取距离数据
	distance_sensor_s distance_msg{};
	if (_distance_sensor_sub.copy(&distance_msg)) {
		_ceiling_distance = distance_msg.current_distance;
		_distance_subscription_active = true;
		
		if (!_lpf_initialized) {
			_ceiling_distance_lpf = _ceiling_distance; _lpf_initialized = true;
		} else {
			float alpha = 0.2f;
			_ceiling_distance_lpf += alpha * (_ceiling_distance - _ceiling_distance_lpf);
		}
	} else {
		// 没有收到距离数据
		_distance_subscription_active = false;
	}

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
		approach_vz_sp = -math::constrain(_param_ceil_appr_vz.get(), 0.05f, 0.1f);
		integral_reset_request = true;
	}
	if (_state == ceiling_contact_status_s::ATTACH_CONTROL_MODE
	    || _state == ceiling_contact_status_s::SURFACE_MANUAL_MODE
	    || _state == ceiling_contact_status_s::DETACH_MODE) {
		integral_reset_request = true; thrust_body_z_sp = compute_distance_control(dt);
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
			"[CeilCtrl] st=%d A=%d D=%d dist=%.3f sub=%d",
			(int)_state, (int)_ceiling_arm_switch, (int)_detach_switch,
			(double)_ceiling_distance_lpf,
			(int)_distance_subscription_active);
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
