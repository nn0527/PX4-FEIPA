#include "ceiling_controller.hpp"

#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace matrix;

namespace
{

hrt_abstime seconds_to_us(float seconds)
{
	return static_cast<hrt_abstime>(math::max(seconds, 0.f) * 1e6f);
}

hrt_abstime milliseconds_to_us(int32_t milliseconds)
{
	return static_cast<hrt_abstime>(math::max(milliseconds, int32_t{0})) * 1000ULL;
}

} // namespace

CeilingController::CeilingController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
	_hover_thrust = math::constrain(_param_mpc_thr_hover.get(), 0.1f, 0.9f);
	_recorded_hover_thrust = _hover_thrust;
	_direct_thrust_sp = -_hover_thrust;
}

CeilingController::~CeilingController()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool CeilingController::init()
{
	parameters_update(true);

	if (!_distance_sensor_subs.advertised()) {
		PX4_WARN("[CeilCtrl] no distance_sensor advertised yet");

	} else {
		PX4_INFO("[CeilCtrl] distance_sensor topic available");
	}

	PX4_INFO("[CeilCtrl] started, selecting upward distance sensor");
	ScheduleOnInterval(10_ms);
	_first_run = true;
	return true;
}

void CeilingController::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate{};
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

bool CeilingController::elapsed_since(hrt_abstime timestamp, hrt_abstime duration, hrt_abstime now) const
{
	return timestamp != 0 && now >= timestamp && (now - timestamp) >= duration;
}

float CeilingController::maximum_thrust() const
{
	const float system_limit = math::constrain(_param_mpc_thr_max.get(), 0.05f, 1.f);
	return math::constrain(_param_ceil_max_thrust.get(), 0.05f, math::min(system_limit, 0.95f));
}

float CeilingController::clearance_distance() const
{
	// A configured clearance at or below D0 must still move beyond the
	// contact-release hysteresis before control is handed back to ALTCTL.
	return math::max(_param_ceil_det_dist.get(), _param_ceil_d0.get() + _param_ceil_cont_hyst.get());
}

bool CeilingController::parameters_valid() const
{
	const float d0 = _param_ceil_d0.get();
	const float compression = _param_ceil_comp_tgt.get();
	const float contact_compression = _param_ceil_cont_comp.get();
	const float trigger_distance = _param_ceil_dist_thr.get();
	const float detach_distance = _param_ceil_det_dist.get();
	const float detach_thrust = _param_ceil_det_thr.get();
	const float handoff_time = _param_ceil_handoff_t.get();

	bool valid = PX4_ISFINITE(d0) && PX4_ISFINITE(compression) && PX4_ISFINITE(contact_compression)
		     && PX4_ISFINITE(trigger_distance) && PX4_ISFINITE(detach_distance) && PX4_ISFINITE(detach_thrust)
		     && PX4_ISFINITE(handoff_time)
		     && compression > 0.f && contact_compression > 0.f
		     && contact_compression <= compression && (d0 - compression) > 0.f
		     && trigger_distance > d0
		     && detach_distance >= d0
		     && detach_thrust > 0.f && detach_thrust < maximum_thrust()
		     && milliseconds_to_us(_param_ceil_appr_to.get()) > seconds_to_us(_param_ceil_appr_hover.get())
		     && _param_ceil_unload_to.get() < _param_ceil_detach_to.get()
		     && milliseconds_to_us(_param_ceil_detach_to.get())
		     >= milliseconds_to_us(_param_ceil_unload_to.get()) + seconds_to_us(handoff_time)
		     && milliseconds_to_us(_param_ceil_unload_to.get()) >= seconds_to_us(_param_ceil_ramp_t.get())
		     && milliseconds_to_us(_param_ceil_attach_to.get()) >= math::max(ATTACH_MIN_TIME,
				     seconds_to_us(_param_ceil_stable_t.get()))
		     && _param_ceil_recov_t.get() > 0.f && handoff_time >= 0.2f && handoff_time <= 1.f
		     && _param_ceil_ramp_t.get() > 0.f && _param_ceil_z_acc.get() > 0.f
		     && _param_ceil_appr_vz.get() <= _param_mpc_z_vel_max_up.get()
		     && _param_ceil_detach_vz.get() <= _param_mpc_z_vel_max_dn.get()
		     && maximum_thrust() + FLT_EPSILON >= _hover_thrust;

	if (PX4_ISFINITE(_selected_sensor_min_distance) && PX4_ISFINITE(_selected_sensor_max_distance)) {
		valid = valid && (d0 - compression) >= _selected_sensor_min_distance
			&& trigger_distance <= _selected_sensor_max_distance
			&& clearance_distance() <= _selected_sensor_max_distance;
	}

	return valid;
}

void CeilingController::update_distance_sensors(hrt_abstime now)
{
	_distance_updated_this_cycle = false;
	distance_sensor_s selected_sample{};
	int selected_instance = -1;
	const uint32_t configured_device_id = static_cast<uint32_t>(math::max(_param_ceil_sens_id.get(), int32_t{0}));
	const hrt_abstime sensor_timeout = milliseconds_to_us(_param_ceil_sensor_to.get());
	const bool engagement_active = _state != ceiling_contact_status_s::NORMAL_FLIGHT;
	const bool keep_current_sensor = _selected_sensor_instance >= 0
					 && (engagement_active
					     || (_distance_timestamp != 0 && now >= _distance_timestamp
							     && (now - _distance_timestamp) <= sensor_timeout
							     && (configured_device_id == 0 || _selected_sensor_device_id == configured_device_id)));

	for (int instance = 0; instance < _distance_sensor_subs.size(); ++instance) {
		distance_sensor_s sample{};

		if (!_distance_sensor_subs[instance].update(&sample)) {
			continue;
		}

		const bool timestamp_valid = sample.timestamp != 0 && sample.timestamp <= now;
		const bool range_valid = PX4_ISFINITE(sample.current_distance) && PX4_ISFINITE(sample.min_distance)
					 && PX4_ISFINITE(sample.max_distance) && sample.min_distance >= 0.f
					 && sample.max_distance > sample.min_distance
					 && sample.current_distance >= sample.min_distance
					 && sample.current_distance <= sample.max_distance;
		const bool quality_valid = sample.signal_quality != 0;
		const bool device_valid = configured_device_id == 0 || sample.device_id == configured_device_id;
		const bool current_instance = instance == _selected_sensor_instance;
		const bool locked_device_valid = !engagement_active || !current_instance
						 || sample.device_id == _selected_sensor_device_id;
		const bool sample_valid = timestamp_valid && range_valid && quality_valid && device_valid
					  && locked_device_valid
					  && sample.orientation == distance_sensor_s::ROTATION_UPWARD_FACING
					  && sample.mode != distance_sensor_s::MODE_DISABLED;

		if (sample_valid && (current_instance || (!keep_current_sensor && selected_instance < 0))) {
			selected_sample = sample;
			selected_instance = instance;
		}
	}

	if (selected_instance < 0) {
		return;
	}

	const bool changed_instance = selected_instance != _selected_sensor_instance
				      || selected_sample.device_id != _selected_sensor_device_id;
	const hrt_abstime previous_sample_timestamp = _distance_sample_timestamp;
	_selected_sensor_instance = selected_instance;
	_selected_sensor_device_id = selected_sample.device_id;
	_selected_sensor_min_distance = selected_sample.min_distance;
	_selected_sensor_max_distance = selected_sample.max_distance;
	_distance_timestamp = selected_sample.timestamp;
	_distance_sample_timestamp = selected_sample.timestamp;
	_ceiling_distance = selected_sample.current_distance;
	_distance_updated_this_cycle = true;

	if (!_lpf_initialized || changed_instance || previous_sample_timestamp == 0
	    || selected_sample.timestamp <= previous_sample_timestamp) {
		_ceiling_distance_lpf = _ceiling_distance;
		_lpf_initialized = true;
		_distance_sample_dt = 0.01f;
		_dist_error_integral = 0.f;
		_dist_error_prev = 0.f;
		_dist_error_initialized = false;

	} else {
		const float sample_dt = math::constrain((selected_sample.timestamp - previous_sample_timestamp) / 1e6f,
							0.001f, 1.f);
		_distance_sample_dt = sample_dt;
		const float time_constant = math::max(_param_ceil_flt_tc.get(), 0.f);
		const float alpha = time_constant > FLT_EPSILON ? sample_dt / (time_constant + sample_dt) : 1.f;
		_ceiling_distance_lpf += math::constrain(alpha, 0.f, 1.f) * (_ceiling_distance - _ceiling_distance_lpf);
	}
}

bool CeilingController::distance_sensor_valid(hrt_abstime now) const
{
	const hrt_abstime timeout = milliseconds_to_us(_param_ceil_sensor_to.get());
	const uint32_t configured_device_id = static_cast<uint32_t>(math::max(_param_ceil_sens_id.get(), int32_t{0}));
	return _lpf_initialized && _distance_timestamp != 0 && now >= _distance_timestamp
	       && (now - _distance_timestamp) <= timeout && PX4_ISFINITE(_ceiling_distance_lpf)
	       && PX4_ISFINITE(_selected_sensor_min_distance) && PX4_ISFINITE(_selected_sensor_max_distance)
	       && (configured_device_id == 0 || _selected_sensor_device_id == configured_device_id)
	       && _ceiling_distance_lpf >= _selected_sensor_min_distance
	       && _ceiling_distance_lpf <= _selected_sensor_max_distance;
}

bool CeilingController::manual_control_valid(hrt_abstime now) const
{
	return _manual_control.valid && _manual_timestamp != 0 && now >= _manual_timestamp
	       && (now - _manual_timestamp) <= MANUAL_TIMEOUT;
}

bool CeilingController::vertical_state_valid(hrt_abstime now) const
{
	const hrt_abstime timeout = milliseconds_to_us(_param_ceil_sensor_to.get());
	return _local_position_timestamp != 0 && now >= _local_position_timestamp
	       && (now - _local_position_timestamp) <= timeout && _vehicle_local_position.z_valid
	       && _vehicle_local_position.v_z_valid && PX4_ISFINITE(_vehicle_local_position.z)
	       && PX4_ISFINITE(_vehicle_local_position.vz);
}

bool CeilingController::attitude_valid(hrt_abstime now) const
{
	const hrt_abstime timeout = milliseconds_to_us(_param_ceil_sensor_to.get());
	bool quaternion_valid = true;
	float quaternion_norm_squared = 0.f;

	for (float element : _vehicle_attitude.q) {
		quaternion_valid = quaternion_valid && PX4_ISFINITE(element);
		quaternion_norm_squared += element * element;
	}

	return quaternion_valid && quaternion_norm_squared > 0.81f && quaternion_norm_squared < 1.21f
	       && PX4_ISFINITE(_attitude_euler.phi()) && PX4_ISFINITE(_attitude_euler.theta())
	       && PX4_ISFINITE(_attitude_euler.psi()) && _attitude_timestamp != 0 && now >= _attitude_timestamp
	       && (now - _attitude_timestamp) <= timeout;
}

bool CeilingController::control_mode_valid(hrt_abstime now) const
{
	const bool control_fresh = _control_mode_timestamp != 0 && now >= _control_mode_timestamp
				   && (now - _control_mode_timestamp) <= MODE_TIMEOUT;
	const bool status_fresh = _vehicle_status_timestamp != 0 && now >= _vehicle_status_timestamp
				  && (now - _vehicle_status_timestamp) <= STATUS_TIMEOUT;

	return control_fresh && status_fresh && _vehicle_control_mode.flag_armed
	       && _vehicle_control_mode.flag_multicopter_position_control_enabled
	       && _vehicle_control_mode.flag_control_manual_enabled
	       && !_vehicle_control_mode.flag_control_auto_enabled
	       && !_vehicle_control_mode.flag_control_offboard_enabled
	       && _vehicle_control_mode.flag_control_altitude_enabled
	       && _vehicle_control_mode.flag_control_climb_rate_enabled
	       && _vehicle_control_mode.flag_control_attitude_enabled
	       && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL
	       && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
}

uint32_t CeilingController::input_faults(hrt_abstime now) const
{
	uint32_t faults = ceiling_contact_status_s::FAULT_NONE;

	if (!parameters_valid()) {
		faults |= ceiling_contact_status_s::FAULT_PARAMETER;
	}

	if (!manual_control_valid(now)) {
		faults |= ceiling_contact_status_s::FAULT_MANUAL_CONTROL;
	}

	if (!distance_sensor_valid(now)) {
		faults |= ceiling_contact_status_s::FAULT_DISTANCE_SENSOR;
	}

	if (!vertical_state_valid(now)) {
		faults |= ceiling_contact_status_s::FAULT_VERTICAL_STATE;
	}

	if (!attitude_valid(now)
	    || fabsf(_attitude_euler.phi()) > math::radians(_param_ceil_max_roll.get())
	    || fabsf(_attitude_euler.theta()) > math::radians(_param_ceil_max_pitch.get())) {
		faults |= ceiling_contact_status_s::FAULT_ATTITUDE;
	}

	if (!control_mode_valid(now)) {
		faults |= ceiling_contact_status_s::FAULT_CONTROL_MODE;
	}

	return faults;
}

void CeilingController::read_switches(hrt_abstime now)
{
	_aux1_rising_edge = false;

	if (manual_control_valid(now) && PX4_ISFINITE(_manual_control.aux1) && PX4_ISFINITE(_manual_control.aux2)) {
		_aux1_raw = _manual_control.aux1;
		_aux2_raw = _manual_control.aux2;
		_ceiling_arm_switch = _aux1_raw > 0.3f;
		_detach_switch = _aux2_raw > 0.3f;

		if (!_ceiling_arm_switch && !_detach_switch) {
			_rearm_required = false;

			if (_state == ceiling_contact_status_s::NORMAL_FLIGHT) {
				_fault_reason_latched = ceiling_contact_status_s::FAULT_NONE;
			}
		}

		_aux1_rising_edge = _ceiling_arm_switch && !_aux1_previous && !_rearm_required;
		_aux1_previous = _ceiling_arm_switch;

	} else {
		_ceiling_arm_switch = false;
		_detach_switch = false;
		_aux1_previous = false;
		_rearm_required = true;
	}
}

void CeilingController::update_inputs(hrt_abstime now)
{
	update_distance_sensors(now);

	manual_control_setpoint_s manual{};

	if (_manual_control_setpoint_sub.update(&manual)) {
		_manual_control = manual;
		_manual_timestamp = manual.timestamp_sample != 0 ? manual.timestamp_sample : manual.timestamp;
	}

	vehicle_local_position_s local_position{};

	if (_vehicle_local_pos_sub.update(&local_position)) {
		_vehicle_local_position = local_position;
		_local_position_timestamp = local_position.timestamp;
		_velocity = Vector3f(local_position.vx, local_position.vy, local_position.vz);
	}

	vehicle_attitude_s attitude{};

	if (_vehicle_attitude_sub.update(&attitude)) {
		_vehicle_attitude = attitude;
		_attitude_timestamp = attitude.timestamp;
		Quatf q(attitude.q);
		_attitude_euler = Eulerf(q);
	}

	vehicle_control_mode_s control_mode{};

	if (_vehicle_control_mode_sub.update(&control_mode)) {
		_vehicle_control_mode = control_mode;
		_control_mode_timestamp = control_mode.timestamp;
	}

	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_vehicle_status = vehicle_status;
		_vehicle_status_timestamp = vehicle_status.timestamp;
	}

	hover_thrust_estimate_s hover_thrust{};

	if (_hover_thrust_estimate_sub.update(&hover_thrust) && hover_thrust.valid
	    && hover_thrust.timestamp != 0 && hover_thrust.timestamp <= now
	    && (now - hover_thrust.timestamp) <= 1_s && PX4_ISFINITE(hover_thrust.hover_thrust)) {
		_hover_thrust = math::constrain(hover_thrust.hover_thrust, 0.1f, 0.9f);
		_hover_thrust_timestamp = hover_thrust.timestamp;
	}

	read_switches(now);
	_compression = _lpf_initialized ? _param_ceil_d0.get() - _ceiling_distance_lpf : NAN;
}

float CeilingController::slew(float current, float target, float rate, float dt) const
{
	if (!PX4_ISFINITE(current)) {
		return target;
	}

	const float maximum_step = math::max(rate, 0.f) * math::max(dt, 0.f);
	return current + math::constrain(target - current, -maximum_step, maximum_step);
}

float CeilingController::compute_distance_control(float dt, float thrust_multiplier)
{
	const float target_distance = _param_ceil_d0.get() - _param_ceil_comp_tgt.get();
	const float error = distance_sensor_valid(hrt_absolute_time()) ? _ceiling_distance_lpf - target_distance : 0.f;
	float derivative = 0.f;

	if (_distance_updated_this_cycle && _dist_error_initialized && _distance_sample_dt > 1e-4f) {
		derivative = (error - _dist_error_prev) / _distance_sample_dt;
	}

	if (_distance_updated_this_cycle) {
		_dist_error_prev = error;
		_dist_error_initialized = true;
	}

	const float integral_candidate = math::constrain(_dist_error_integral + error * dt, -1.f, 1.f);
	const float pid_without_integral = _param_ceil_dist_kp.get() * error + _param_ceil_dist_kd.get() * derivative;
	const float state_thrust_limit = math::min(maximum_thrust(),
					 _recorded_hover_thrust * math::max(thrust_multiplier, 1.f));
	const float raw_thrust = -_recorded_hover_thrust - pid_without_integral
				 - _param_ceil_dist_ki.get() * integral_candidate;
	const float constrained_thrust = math::constrain(raw_thrust, -state_thrust_limit, -_recorded_hover_thrust);

	if (fabsf(raw_thrust - constrained_thrust) <= FLT_EPSILON
	    || (raw_thrust < constrained_thrust && error < 0.f)
	    || (raw_thrust > constrained_thrust && error > 0.f)) {
		_dist_error_integral = integral_candidate;
	}

	const float thrust_rate = maximum_thrust() / math::max(_param_ceil_ramp_t.get(), 0.1f);
	_direct_thrust_sp = slew(_direct_thrust_sp, constrained_thrust, thrust_rate, dt);
	return _direct_thrust_sp;
}

void CeilingController::begin_detach_handoff(hrt_abstime now, bool to_recovery)
{
	if (_detach_unload_stage == DetachUnloadStage::RAMP_TO_HOVER) {
		_detach_handoff_to_recovery = _detach_handoff_to_recovery || to_recovery;
		return;
	}

	_detach_unload_stage = DetachUnloadStage::RAMP_TO_HOVER;
	_detach_handoff_start_time = now;
	_detach_handoff_duration = seconds_to_us(_param_ceil_handoff_t.get());
	_detach_handoff_start_thrust = PX4_ISFINITE(_direct_thrust_sp)
				       ? math::constrain(_direct_thrust_sp, -maximum_thrust(), -0.05f)
				       : -_recorded_hover_thrust;
	_detach_handoff_to_recovery = to_recovery;
	_contact_confirmed = false;

	if (_detach_release_confirmed) {
		_contact_active = false;
	}
}

float CeilingController::compute_unload_thrust(float dt, hrt_abstime now)
{
	_target_distance = math::min(_target_distance + _param_ceil_ramp_dist.get() * dt, _param_ceil_d0.get());

	if (_detach_unload_stage == DetachUnloadStage::RAMP_TO_HOVER) {
		const float progress = (_detach_handoff_duration > 0 && now >= _detach_handoff_start_time)
				       ? math::constrain(static_cast<float>(now - _detach_handoff_start_time)
						 / static_cast<float>(_detach_handoff_duration), 0.f, 1.f)
				       : 0.f;
		_direct_thrust_sp = math::lerp(_detach_handoff_start_thrust, -_recorded_hover_thrust, progress);
		return _direct_thrust_sp;
	}

	const float detach_thrust = math::constrain(_param_ceil_det_thr.get(), 0.05f,
				    math::max(_recorded_hover_thrust - 0.01f, 0.05f));
	// UNLOAD must be monotonic. A distance PID here would command more upward
	// thrust as soon as the vehicle starts separating (distance > target), which
	// can pull it back into the ceiling and prevent both phase and total timeout
	// completion. The range signal is used only for release confirmation while
	// direct thrust slews deterministically to the configured unload value.
	const float thrust_rate = maximum_thrust() / math::max(_param_ceil_ramp_t.get(), 0.1f);
	_direct_thrust_sp = slew(_direct_thrust_sp, -detach_thrust, thrust_rate, dt);
	return _direct_thrust_sp;
}

void CeilingController::update_contact_confirmation(hrt_abstime now)
{
	if (_state != ceiling_contact_status_s::APPROACH_MODE || !distance_sensor_valid(now)) {
		if (_state != ceiling_contact_status_s::ATTACH_CONTROL_MODE
		    && _state != ceiling_contact_status_s::SURFACE_MANUAL_MODE
		    && !(_state == ceiling_contact_status_s::DETACH_MODE
			 && _detach_phase == ceiling_contact_status_s::DETACH_PHASE_UNLOAD)) {
			_contact_confirmed = false;
		}

		_attach_detect_start = 0;
		return;
	}

	if (_compression >= _param_ceil_cont_comp.get()) {
		if (_attach_detect_start == 0) {
			_attach_detect_start = now;
		}

		if (elapsed_since(_attach_detect_start, seconds_to_us(_param_ceil_cont_t.get()), now)) {
			_contact_confirmed = true;
		}

	} else {
		_attach_detect_start = 0;
		_contact_confirmed = false;
	}
}

void CeilingController::update_distance_stability(hrt_abstime now)
{
	if (_state != ceiling_contact_status_s::ATTACH_CONTROL_MODE || !distance_sensor_valid(now)) {
		_dist_stable_start = 0;
		_distance_stable = false;
		return;
	}

	const float target = _param_ceil_d0.get() - _param_ceil_comp_tgt.get();

	if (fabsf(_ceiling_distance_lpf - target) <= DISTANCE_STABLE_WINDOW) {
		if (_dist_stable_start == 0) {
			_dist_stable_start = now;
		}

		_distance_stable = elapsed_since(_dist_stable_start, seconds_to_us(_param_ceil_stable_t.get()), now);

	} else {
		_dist_stable_start = 0;
		_distance_stable = false;
	}
}

void CeilingController::update_detach_release_confirmation(hrt_abstime now)
{
	if (_state != ceiling_contact_status_s::DETACH_MODE
	    || _detach_phase != ceiling_contact_status_s::DETACH_PHASE_UNLOAD
	    || !distance_sensor_valid(now)) {
		_detach_release_start = 0;
		_detach_release_confirmed = false;
		return;
	}

	const float release_distance = _param_ceil_d0.get() + _param_ceil_cont_hyst.get();

	if (_ceiling_distance_lpf >= release_distance) {
		if (_detach_release_start == 0) {
			_detach_release_start = now;
		}

		_detach_release_confirmed = elapsed_since(_detach_release_start,
					    seconds_to_us(_param_ceil_cont_t.get()), now);

	} else {
		_detach_release_start = 0;
		_detach_release_confirmed = false;
	}
}

bool CeilingController::approach_entry_condition() const
{
	return _ceiling_distance_lpf < _param_ceil_dist_thr.get()
	       && fabsf(_vehicle_local_position.vz) < _param_ceil_vel_thr.get();
}

bool CeilingController::contact_lost_condition(hrt_abstime now)
{
	const bool distance_indicates_loss = distance_sensor_valid(now)
					     && _ceiling_distance_lpf > _param_ceil_d0.get() + _param_ceil_cont_hyst.get();

	if (distance_indicates_loss) {
		if (_contact_loss_start == 0) {
			_contact_loss_start = now;
		}

		return elapsed_since(_contact_loss_start, seconds_to_us(_param_ceil_cont_t.get()), now);
	}

	_contact_loss_start = 0;
	return false;
}

void CeilingController::require_rearm(uint32_t reason)
{
	_rearm_required = true;

	if (reason != ceiling_contact_status_s::FAULT_NONE) {
		const uint32_t new_faults = reason & ~_fault_reason_latched;

		if (new_faults != 0) {
			++_fault_count;
		}

		_fault_reason_latched |= reason;
	}
}

void CeilingController::enter_state(uint8_t new_state)
{
	if (_state == new_state) {
		return;
	}

	PX4_INFO("Ceiling state: %d -> %d", _state, new_state);
	_state = new_state;
	_state_entry_time = hrt_absolute_time();
	_entry_condition_start = 0;
	_contact_loss_start = 0;

	switch (_state) {
	case ceiling_contact_status_s::NORMAL_FLIGHT:
		_detach_phase = ceiling_contact_status_s::DETACH_PHASE_NONE;
		_detach_unload_stage = DetachUnloadStage::RAMP_TO_RELEASE_THRUST;
		_detach_handoff_start_time = 0;
		_detach_handoff_duration = 0;
		_detach_handoff_start_thrust = NAN;
		_detach_handoff_to_recovery = false;
		_contact_active = false;
		_contact_confirmed = false;
		_distance_stable = false;
		_vertical_velocity_sp = 0.f;
		break;

	case ceiling_contact_status_s::CEILING_ARM_MODE:
		_attach_detect_start = 0;
		_dist_stable_start = 0;
		_dist_error_integral = 0.f;
		_dist_error_prev = 0.f;
		_dist_error_initialized = false;
		_contact_confirmed = false;
		_distance_stable = false;
		_target_distance = _param_ceil_d0.get() - _param_ceil_comp_tgt.get();
		_fault_reason_latched = ceiling_contact_status_s::FAULT_NONE;
		break;

	case ceiling_contact_status_s::APPROACH_MODE:
		_attach_detect_start = 0;
		_dist_error_integral = 0.f;
		_dist_error_prev = 0.f;
		_dist_error_initialized = false;
		_contact_confirmed = false;
		_recorded_hover_thrust = math::constrain(
						 (_hover_thrust_timestamp != 0 && _state_entry_time >= _hover_thrust_timestamp
						  && (_state_entry_time - _hover_thrust_timestamp) <= 1_s) ? _hover_thrust : _param_mpc_thr_hover.get(),
						 0.1f, maximum_thrust());
		_vertical_velocity_sp = 0.f;
		_integral_reset_pending = true;
		break;

	case ceiling_contact_status_s::ATTACH_CONTROL_MODE: {
			_contact_active = true;
			_contact_confirmed = true;
			_dist_stable_start = 0;
			_distance_stable = false;
			_target_distance = _param_ceil_d0.get() - _param_ceil_comp_tgt.get();
			vehicle_thrust_setpoint_s thrust_setpoint{};

			if (_vehicle_thrust_setpoint_sub.copy(&thrust_setpoint) && PX4_ISFINITE(thrust_setpoint.xyz[2])) {
				_attach_baseline_thrust = math::constrain(thrust_setpoint.xyz[2], -maximum_thrust(), -0.05f);

			} else {
				_attach_baseline_thrust = -_recorded_hover_thrust;
			}

			_direct_thrust_sp = _attach_baseline_thrust;
			_integral_reset_pending = true;
			break;
		}

	case ceiling_contact_status_s::SURFACE_MANUAL_MODE:
		_contact_active = true;
		_contact_confirmed = true;
		_integral_reset_pending = true;
		break;

	case ceiling_contact_status_s::DETACH_MODE:
		_detach_phase = ceiling_contact_status_s::DETACH_PHASE_UNLOAD;
		_detach_unload_stage = DetachUnloadStage::RAMP_TO_RELEASE_THRUST;
		_detach_phase_entry_time = _state_entry_time;
		_detach_handoff_start_time = 0;
		_detach_handoff_duration = 0;
		_detach_handoff_start_thrust = NAN;
		_detach_handoff_to_recovery = false;
		_contact_active = true;
		_detach_release_start = 0;
		_detach_release_confirmed = false;
		_target_distance = _param_ceil_d0.get() - _param_ceil_comp_tgt.get();
		_dist_error_integral = 0.f;
		_dist_error_prev = 0.f;
		_dist_error_initialized = false;
		_integral_reset_pending = true;
		break;

	case ceiling_contact_status_s::RECOVERY_HOVER_MODE:
		_detach_phase = ceiling_contact_status_s::DETACH_PHASE_NONE;
		_detach_unload_stage = DetachUnloadStage::RAMP_TO_RELEASE_THRUST;
		_detach_handoff_start_time = 0;
		_detach_handoff_duration = 0;
		_detach_handoff_start_thrust = NAN;
		_detach_handoff_to_recovery = false;
		_contact_confirmed = false;
		// Recovery is the braking/hold phase. Stop commanding additional
		// clearance descent immediately; the regular Z velocity loop decelerates
		// the vehicle to zero while FlightModeManager re-seeds the height target.
		_vertical_velocity_sp = 0.f;
		_integral_reset_pending = true;
		break;

	default:
		break;
	}
}

void CeilingController::enter_detach_mode()
{
	if (_state != ceiling_contact_status_s::DETACH_MODE) {
		enter_state(ceiling_contact_status_s::DETACH_MODE);
	}
}

void CeilingController::update_state_machine(float dt)
{
	(void)dt;
	const hrt_abstime now = hrt_absolute_time();
	update_contact_confirmation(now);
	update_distance_stability(now);
	update_detach_release_confirmation(now);
	const uint32_t faults = input_faults(now);

	switch (_state) {
	case ceiling_contact_status_s::NORMAL_FLIGHT:
		if (_aux1_rising_edge) {
			if (!_detach_switch && faults == ceiling_contact_status_s::FAULT_NONE) {
				enter_state(ceiling_contact_status_s::CEILING_ARM_MODE);

			} else {
				require_rearm(faults);
			}
		}

		break;

	case ceiling_contact_status_s::CEILING_ARM_MODE:
		if (!_ceiling_arm_switch) {
			enter_state(ceiling_contact_status_s::NORMAL_FLIGHT);
			return;
		}

		if (_detach_switch || faults != ceiling_contact_status_s::FAULT_NONE) {
			require_rearm(faults);
			enter_state(ceiling_contact_status_s::NORMAL_FLIGHT);
			return;
		}

		if (approach_entry_condition()) {
			if (_entry_condition_start == 0) {
				_entry_condition_start = now;
			}

			if (elapsed_since(_entry_condition_start, seconds_to_us(_param_ceil_entry_t.get()), now)) {
				enter_state(ceiling_contact_status_s::APPROACH_MODE);
			}

		} else {
			_entry_condition_start = 0;
		}

		break;

	case ceiling_contact_status_s::APPROACH_MODE: {
			const bool possibly_in_contact = _contact_confirmed
							 || (distance_sensor_valid(now)
							     && _ceiling_distance_lpf <= _param_ceil_d0.get());

			if (!_ceiling_arm_switch) {
				possibly_in_contact ? enter_detach_mode() : enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
				return;
			}

			if (_detach_switch || faults != ceiling_contact_status_s::FAULT_NONE) {
				require_rearm(faults);
				possibly_in_contact ? enter_detach_mode() : enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
				return;
			}

			if (elapsed_since(_state_entry_time, milliseconds_to_us(_param_ceil_appr_to.get()), now)) {
				require_rearm(ceiling_contact_status_s::FAULT_APPROACH_TIMEOUT);
				possibly_in_contact ? enter_detach_mode() : enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
				return;
			}

			if (_contact_confirmed
			    && elapsed_since(_state_entry_time, seconds_to_us(_param_ceil_appr_hover.get()), now)) {
				enter_state(ceiling_contact_status_s::ATTACH_CONTROL_MODE);
			}

			break;
		}

	case ceiling_contact_status_s::ATTACH_CONTROL_MODE:
		if (!_ceiling_arm_switch || _detach_switch || faults != ceiling_contact_status_s::FAULT_NONE) {
			require_rearm(faults);
			enter_detach_mode();
			return;
		}

		if (contact_lost_condition(now)) {
			require_rearm(ceiling_contact_status_s::FAULT_CONTACT_LOST);
			enter_detach_mode();
			return;
		}

		if (elapsed_since(_state_entry_time, milliseconds_to_us(_param_ceil_attach_to.get()), now)) {
			require_rearm(ceiling_contact_status_s::FAULT_ATTACH_TIMEOUT);
			enter_detach_mode();
			return;
		}

		if (elapsed_since(_state_entry_time, ATTACH_MIN_TIME, now) && _contact_confirmed && _distance_stable) {
			enter_state(ceiling_contact_status_s::SURFACE_MANUAL_MODE);
		}

		break;

	case ceiling_contact_status_s::SURFACE_MANUAL_MODE:
		if (!_ceiling_arm_switch || _detach_switch || faults != ceiling_contact_status_s::FAULT_NONE) {
			require_rearm(faults);
			enter_detach_mode();
			return;
		}

		if (contact_lost_condition(now)) {
			require_rearm(ceiling_contact_status_s::FAULT_CONTACT_LOST);
			enter_detach_mode();
		}

		break;

	case ceiling_contact_status_s::DETACH_MODE: {
			const bool detach_timed_out = elapsed_since(_state_entry_time,
						      milliseconds_to_us(_param_ceil_detach_to.get()), now);

			if (detach_timed_out) {
				require_rearm(ceiling_contact_status_s::FAULT_DETACH_TIMEOUT);
			}

			if (_detach_phase == ceiling_contact_status_s::DETACH_PHASE_UNLOAD) {
				if (_detach_unload_stage == DetachUnloadStage::RAMP_TO_RELEASE_THRUST) {
					const bool unload_timed_out = elapsed_since(_detach_phase_entry_time,
							      milliseconds_to_us(_param_ceil_unload_to.get()), now);
					const float detach_thrust = math::constrain(_param_ceil_det_thr.get(), 0.05f,
								    math::max(_recorded_hover_thrust - 0.01f, 0.05f));
					const bool thrust_unloaded = fabsf(_direct_thrust_sp + detach_thrust) < 0.02f;

					if (unload_timed_out) {
						require_rearm(ceiling_contact_status_s::FAULT_DETACH_TIMEOUT);
					}

					if (detach_timed_out) {
						// A total timeout stops any further clearance descent. Stay in
						// DIRECT_THRUST until the current output is back at hover.
						begin_detach_handoff(now, true);

					} else if ((thrust_unloaded && _detach_release_confirmed) || unload_timed_out) {
						// A release or unload timeout starts the pre-velocity handoff
						// from the actual direct-thrust value reached in this cycle.
						begin_detach_handoff(now, false);
					}

				} else {
					if (detach_timed_out) {
						begin_detach_handoff(now, true);
					}

					const bool handoff_complete = elapsed_since(_detach_handoff_start_time,
								      _detach_handoff_duration, now)
							      && fabsf(_direct_thrust_sp + _recorded_hover_thrust) <= 0.01f;

					if (handoff_complete) {
						if (_detach_handoff_to_recovery) {
							enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
							return;
						}

						_detach_phase = ceiling_contact_status_s::DETACH_PHASE_CLEARANCE;
						_detach_phase_entry_time = now;
						_vertical_velocity_sp = 0.f;
						_entry_condition_start = 0;
						_integral_reset_pending = true;
					}
				}

			} else if (_detach_phase == ceiling_contact_status_s::DETACH_PHASE_CLEARANCE) {
				const uint32_t clearance_faults = faults
								  & (ceiling_contact_status_s::FAULT_MANUAL_CONTROL
								     | ceiling_contact_status_s::FAULT_VERTICAL_STATE
								     | ceiling_contact_status_s::FAULT_CONTROL_MODE
								     | ceiling_contact_status_s::FAULT_ATTITUDE
								     | ceiling_contact_status_s::FAULT_PARAMETER);

				if (clearance_faults != 0) {
					require_rearm(clearance_faults);
					enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
					return;
				}

				if (detach_timed_out) {
					enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
					return;
				}

				if (distance_sensor_valid(now) && _ceiling_distance_lpf >= clearance_distance()) {
					if (_entry_condition_start == 0) {
						_entry_condition_start = now;
					}

					if (elapsed_since(_entry_condition_start, seconds_to_us(_param_ceil_entry_t.get()), now)) {
						_contact_active = false;
						enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
						return;
					}

				} else if (distance_sensor_valid(now)) {
					_entry_condition_start = 0;

				} else {
					require_rearm(ceiling_contact_status_s::FAULT_DISTANCE_SENSOR);
					const float fallback_distance = clearance_distance() - _param_ceil_d0.get();
					const float fallback_velocity = math::max(_param_ceil_detach_vz.get(), 0.05f);
					const float fallback_acceleration = math::max(_param_ceil_z_acc.get(), 0.05f);
					const float acceleration_time = fallback_velocity / fallback_acceleration;
					const float acceleration_distance = 0.5f * fallback_velocity * acceleration_time;
					const float fallback_time_s = fallback_distance <= acceleration_distance
								      ? sqrtf(2.f * fallback_distance / fallback_acceleration)
								      : acceleration_time
								      + (fallback_distance - acceleration_distance) / fallback_velocity;

					if (elapsed_since(_detach_phase_entry_time, seconds_to_us(fallback_time_s), now)) {
						enter_state(ceiling_contact_status_s::RECOVERY_HOVER_MODE);
						return;
					}
				}
			}

			break;
		}

	case ceiling_contact_status_s::RECOVERY_HOVER_MODE: {
			const uint32_t recovery_faults = faults & (ceiling_contact_status_s::FAULT_VERTICAL_STATE
							 | ceiling_contact_status_s::FAULT_CONTROL_MODE
							 | ceiling_contact_status_s::FAULT_ATTITUDE
							 | ceiling_contact_status_s::FAULT_PARAMETER);

			const bool recovery_complete = elapsed_since(_state_entry_time,
						       seconds_to_us(_param_ceil_recov_t.get()), now)
						       && fabsf(_vertical_velocity_sp) < 0.02f
						       && vertical_state_valid(now)
						       && fabsf(_vehicle_local_position.vz) < 0.15f;

			if (recovery_faults != 0 || recovery_complete) {
				require_rearm(recovery_faults);
				enter_state(ceiling_contact_status_s::NORMAL_FLIGHT);
			}

			break;
		}

	default:
		require_rearm(ceiling_contact_status_s::FAULT_PARAMETER);
		enter_state(ceiling_contact_status_s::NORMAL_FLIGHT);
		break;
	}
}

void CeilingController::publish_status(hrt_abstime now, float thrust_body_z_sp, float vertical_velocity_sp,
				       uint8_t z_control_mode, bool integral_reset_request)
{
	ceiling_contact_status_s status{};
	status.timestamp = now;
	status.state = _state;
	status.z_control_mode = z_control_mode;
	status.detach_phase = _detach_phase;
	status.ceiling_distance = _ceiling_distance_lpf;
	status.target_distance = _target_distance;
	status.compression = _compression;
	status.target_compression = _param_ceil_comp_tgt.get();
	status.thrust_body_z_sp = thrust_body_z_sp;
	status.vertical_velocity_sp = vertical_velocity_sp;
	status.approach_vz_sp = _state == ceiling_contact_status_s::APPROACH_MODE ? vertical_velocity_sp : NAN;
	status.ceiling_arm_switch_on = _ceiling_arm_switch;
	status.detach_switch_on = _detach_switch;
	status.attach_confirmed = _contact_confirmed;
	status.distance_stable = _distance_stable;
	status.fault_detected = _fault_reason_latched != ceiling_contact_status_s::FAULT_NONE;
	status.integral_reset_request = integral_reset_request;
	status.wheel_stop_request = _state == ceiling_contact_status_s::DETACH_MODE || status.fault_detected;
	status.contact_active = _contact_active;
	status.input_valid = input_faults(now) == ceiling_contact_status_s::FAULT_NONE;
	status.distance_sensor_valid = distance_sensor_valid(now);
	status.rearm_required = _rearm_required;
	status.fault_reason = _fault_reason_latched;
	status.hover_thrust_baseline = _recorded_hover_thrust;
	status.state_entry_time = _state_entry_time;
	status.fault_count = _fault_count;
	_status_pub.publish(status);
}

void CeilingController::Run()
{
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);
	const hrt_abstime now = hrt_absolute_time();

	if (_first_run) {
		_first_run = false;
		mavlink_log_info(&_mavlink_log_pub, "[CeilCtrl] STARTED, selecting upward distance sensor");
	}

	parameters_update(false);
	update_inputs(now);

	float dt = 0.01f;

	if (_last_run != 0 && now > _last_run) {
		dt = math::constrain((now - _last_run) / 1e6f, 0.001f, 0.05f);
	}

	_last_run = now;
	update_state_machine(dt);

	float thrust_body_z_sp = NAN;
	float vertical_velocity_sp = NAN;
	uint8_t z_control_mode = ceiling_contact_status_s::Z_CONTROL_MODE_NONE;
	bool integral_reset_request = _integral_reset_pending;

	switch (_state) {
	case ceiling_contact_status_s::APPROACH_MODE: {
			const bool hover_phase = !elapsed_since(_state_entry_time,
								seconds_to_us(_param_ceil_appr_hover.get()), now);
			const float target_velocity = hover_phase ? 0.f
						      : -math::constrain(_param_ceil_appr_vz.get(), 0.05f, 1.f);
			_vertical_velocity_sp = slew(_vertical_velocity_sp, target_velocity, _param_ceil_z_acc.get(), dt);
			vertical_velocity_sp = _vertical_velocity_sp;
			z_control_mode = ceiling_contact_status_s::Z_CONTROL_MODE_VELOCITY;
			break;
		}

	case ceiling_contact_status_s::ATTACH_CONTROL_MODE:
		thrust_body_z_sp = compute_distance_control(dt, _param_ceil_attach_mult.get());
		z_control_mode = ceiling_contact_status_s::Z_CONTROL_MODE_DIRECT_THRUST;
		integral_reset_request = true;
		break;

	case ceiling_contact_status_s::SURFACE_MANUAL_MODE:
		thrust_body_z_sp = compute_distance_control(dt, _param_ceil_surf_mult.get());
		z_control_mode = ceiling_contact_status_s::Z_CONTROL_MODE_DIRECT_THRUST;
		integral_reset_request = true;
		break;

	case ceiling_contact_status_s::DETACH_MODE:
		if (_detach_phase == ceiling_contact_status_s::DETACH_PHASE_UNLOAD) {
			thrust_body_z_sp = compute_unload_thrust(dt, now);
			z_control_mode = ceiling_contact_status_s::Z_CONTROL_MODE_DIRECT_THRUST;
			integral_reset_request = true;

		} else if (_detach_phase == ceiling_contact_status_s::DETACH_PHASE_CLEARANCE
			   && vertical_state_valid(now) && control_mode_valid(now)) {
			_vertical_velocity_sp = slew(_vertical_velocity_sp,
						     math::constrain(_param_ceil_detach_vz.get(), 0.05f, 1.f),
						     _param_ceil_z_acc.get(), dt);
			vertical_velocity_sp = _vertical_velocity_sp;
			z_control_mode = ceiling_contact_status_s::Z_CONTROL_MODE_VELOCITY;
		}

		break;

	case ceiling_contact_status_s::RECOVERY_HOVER_MODE:
		if (vertical_state_valid(now) && control_mode_valid(now)) {
			_vertical_velocity_sp = slew(_vertical_velocity_sp, 0.f, _param_ceil_z_acc.get(), dt);
			vertical_velocity_sp = _vertical_velocity_sp;
			z_control_mode = ceiling_contact_status_s::Z_CONTROL_MODE_VELOCITY;
		}

		break;

	default:
		break;
	}

	publish_status(now, thrust_body_z_sp, vertical_velocity_sp, z_control_mode, integral_reset_request);
	_integral_reset_pending = false;

	if (_ceiling_arm_switch != _last_ceiling_arm_switch || _detach_switch != _last_detach_switch) {
		mavlink_log_info(&_mavlink_log_pub, "[CeilCtrl] ARM=%d DET=%d st=%d",
				 (int)_ceiling_arm_switch, (int)_detach_switch, (int)_state);
		_last_ceiling_arm_switch = _ceiling_arm_switch;
		_last_detach_switch = _detach_switch;
	}

	if (hrt_elapsed_time(&_last_status_log_time) > 5_s) {
		mavlink_log_info(&_mavlink_log_pub,
				 "[CeilCtrl] st=%d phase=%d A=%d D=%d dist=%.3f valid=%d fault=0x%lx",
				 (int)_state, (int)_detach_phase, (int)_ceiling_arm_switch, (int)_detach_switch,
				 (double)_ceiling_distance_lpf, (int)distance_sensor_valid(now),
				 (unsigned long)_fault_reason_latched);
		_last_status_log_time = now;
	}

	perf_end(_loop_perf);
}

int CeilingController::task_spawn(int argc, char *argv[])
{
	CeilingController *instance = new CeilingController();

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

int CeilingController::custom_command(int, char *[])
{
	return print_usage("unknown command");
}

int CeilingController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("ceiling_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ceiling_controller_main(int argc, char *argv[])
{
	return CeilingController::main(argc, argv);
}
