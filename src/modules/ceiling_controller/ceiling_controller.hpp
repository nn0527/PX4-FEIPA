#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/ceiling_contact_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <lib/systemlib/mavlink_log.h>

using namespace time_literals;

class CeilingController : public ModuleBase<CeilingController>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	CeilingController();
	~CeilingController() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	enum class DetachUnloadStage : uint8_t {
		RAMP_TO_RELEASE_THRUST = 0,
		RAMP_TO_HOVER,
	};

	void Run() override;
	void parameters_update(bool force);
	void update_state_machine(float dt);
	void enter_state(uint8_t new_state);
	void enter_detach_mode();
	void update_inputs(hrt_abstime now);
	void update_distance_sensors(hrt_abstime now);
	void update_contact_confirmation(hrt_abstime now);
	void update_distance_stability(hrt_abstime now);
	void update_detach_release_confirmation(hrt_abstime now);
	void read_switches(hrt_abstime now);
	void publish_status(hrt_abstime now, float thrust_body_z_sp, float vertical_velocity_sp,
			    uint8_t z_control_mode, bool integral_reset_request);

	bool parameters_valid() const;
	bool distance_sensor_valid(hrt_abstime now) const;
	bool manual_control_valid(hrt_abstime now) const;
	bool vertical_state_valid(hrt_abstime now) const;
	bool attitude_valid(hrt_abstime now) const;
	bool control_mode_valid(hrt_abstime now) const;
	uint32_t input_faults(hrt_abstime now) const;
	bool approach_entry_condition() const;
	bool contact_lost_condition(hrt_abstime now);
	bool elapsed_since(hrt_abstime timestamp, hrt_abstime duration, hrt_abstime now) const;

	float compute_distance_control(float dt, float thrust_multiplier);
	float compute_unload_thrust(float dt, hrt_abstime now);
	float slew(float current, float target, float rate, float dt) const;
	float maximum_thrust() const;
	float clearance_distance() const;
	void begin_detach_handoff(hrt_abstime now, bool to_recovery);
	void require_rearm(uint32_t reason = ceiling_contact_status_s::FAULT_NONE);

	float _ceiling_distance{100.f};
	float _target_distance{0.5f};
	float _compression{0.f};
	matrix::Vector3f _velocity{};
	matrix::Eulerf _attitude_euler{};
	float _dist_error_integral{0.f};
	float _dist_error_prev{0.f};
	float _distance_sample_dt{0.01f};
	bool _dist_error_initialized{false};
	float _ceiling_distance_lpf{100.f};
	float _hover_thrust{0.72f};
	float _recorded_hover_thrust{0.72f};
	float _attach_baseline_thrust{-0.72f};
	float _direct_thrust_sp{-0.72f};
	float _vertical_velocity_sp{0.f};
	bool _lpf_initialized{false};
	bool _distance_updated_this_cycle{false};
	float _selected_sensor_min_distance{NAN};
	float _selected_sensor_max_distance{NAN};
	uint32_t _selected_sensor_device_id{0};
	int8_t _selected_sensor_instance{-1};
	hrt_abstime _distance_timestamp{0};
	hrt_abstime _distance_sample_timestamp{0};
	hrt_abstime _local_position_timestamp{0};
	hrt_abstime _attitude_timestamp{0};
	hrt_abstime _manual_timestamp{0};
	hrt_abstime _control_mode_timestamp{0};
	hrt_abstime _vehicle_status_timestamp{0};
	hrt_abstime _hover_thrust_timestamp{0};

	uint8_t _state{ceiling_contact_status_s::NORMAL_FLIGHT};
	uint8_t _detach_phase{ceiling_contact_status_s::DETACH_PHASE_NONE};
	DetachUnloadStage _detach_unload_stage{DetachUnloadStage::RAMP_TO_RELEASE_THRUST};
	hrt_abstime _state_entry_time{0};
	hrt_abstime _detach_phase_entry_time{0};
	hrt_abstime _detach_handoff_start_time{0};
	hrt_abstime _detach_handoff_duration{0};
	float _detach_handoff_start_thrust{NAN};
	bool _detach_handoff_to_recovery{false};
	uint32_t _fault_count{0};
	uint32_t _fault_reason_latched{ceiling_contact_status_s::FAULT_NONE};
	bool _contact_confirmed{false};
	bool _distance_stable{false};
	bool _contact_active{false};
	bool _detach_release_confirmed{false};
	bool _rearm_required{true};
	bool _aux1_previous{false};
	bool _aux1_rising_edge{false};
	hrt_abstime _attach_detect_start{0};
	hrt_abstime _dist_stable_start{0};
	hrt_abstime _entry_condition_start{0};
	hrt_abstime _contact_loss_start{0};
	hrt_abstime _detach_release_start{0};

	static constexpr hrt_abstime MANUAL_TIMEOUT{500_ms};
	static constexpr hrt_abstime MODE_TIMEOUT{1_s};
	static constexpr hrt_abstime STATUS_TIMEOUT{1_s};
	static constexpr hrt_abstime ATTACH_MIN_TIME{500_ms};
	static constexpr float DISTANCE_STABLE_WINDOW{0.01f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CEIL_D0>) _param_ceil_d0,
		(ParamFloat<px4::params::CEIL_COMP_TGT>) _param_ceil_comp_tgt,
		(ParamFloat<px4::params::CEIL_DIST_THR>) _param_ceil_dist_thr,
		(ParamFloat<px4::params::CEIL_VEL_THR>) _param_ceil_vel_thr,
		(ParamFloat<px4::params::CEIL_APPR_VZ>) _param_ceil_appr_vz,
		(ParamFloat<px4::params::CEIL_APPR_HOVER>) _param_ceil_appr_hover,
		(ParamInt<px4::params::CEIL_APPR_TO>) _param_ceil_appr_to,
		(ParamFloat<px4::params::CEIL_DIST_KP>) _param_ceil_dist_kp,
		(ParamFloat<px4::params::CEIL_DIST_KI>) _param_ceil_dist_ki,
		(ParamFloat<px4::params::CEIL_DIST_KD>) _param_ceil_dist_kd,
		(ParamFloat<px4::params::CEIL_STABLE_T>) _param_ceil_stable_t,
		(ParamFloat<px4::params::CEIL_MAX_ROLL>) _param_ceil_max_roll,
		(ParamFloat<px4::params::CEIL_MAX_PITCH>) _param_ceil_max_pitch,
		(ParamFloat<px4::params::CEIL_MAX_THRUST>) _param_ceil_max_thrust,
		(ParamFloat<px4::params::CEIL_RAMP_DIST>) _param_ceil_ramp_dist,
		(ParamFloat<px4::params::CEIL_ATTACH_MULT>) _param_ceil_attach_mult,
		(ParamFloat<px4::params::CEIL_SURF_MULT>) _param_ceil_surf_mult,
		(ParamFloat<px4::params::CEIL_FLT_TC>) _param_ceil_flt_tc,
		(ParamFloat<px4::params::CEIL_ENTRY_T>) _param_ceil_entry_t,
		(ParamFloat<px4::params::CEIL_CONT_COMP>) _param_ceil_cont_comp,
		(ParamFloat<px4::params::CEIL_CONT_T>) _param_ceil_cont_t,
		(ParamFloat<px4::params::CEIL_CONT_HYST>) _param_ceil_cont_hyst,
		(ParamInt<px4::params::CEIL_ATTACH_TO>) _param_ceil_attach_to,
		(ParamFloat<px4::params::CEIL_DETACH_VZ>) _param_ceil_detach_vz,
		(ParamFloat<px4::params::CEIL_DET_THR>) _param_ceil_det_thr,
		(ParamFloat<px4::params::CEIL_DET_DIST>) _param_ceil_det_dist,
		(ParamInt<px4::params::CEIL_UNLOAD_TO>) _param_ceil_unload_to,
		(ParamInt<px4::params::CEIL_DETACH_TO>) _param_ceil_detach_to,
		(ParamFloat<px4::params::CEIL_RECOV_T>) _param_ceil_recov_t,
		(ParamInt<px4::params::CEIL_SENSOR_TO>) _param_ceil_sensor_to,
		(ParamInt<px4::params::CEIL_SENS_ID>) _param_ceil_sens_id,
		(ParamFloat<px4::params::CEIL_RAMP_T>) _param_ceil_ramp_t,
		(ParamFloat<px4::params::CEIL_HANDOFF_T>) _param_ceil_handoff_t,
		(ParamFloat<px4::params::CEIL_Z_ACC>) _param_ceil_z_acc,
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn
	)

	orb_advert_t _mavlink_log_pub{nullptr};
	uORB::Publication<ceiling_contact_status_s> _status_pub{ORB_ID(ceiling_contact_status)};
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	bool _ceiling_arm_switch{false};
	bool _detach_switch{false};
	float _aux1_raw{0.f};
	float _aux2_raw{0.f};
	bool _last_ceiling_arm_switch{false};
	bool _last_detach_switch{false};
	manual_control_setpoint_s _manual_control{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_status_s _vehicle_status{};
	hrt_abstime _last_status_log_time{0};
	hrt_abstime _last_run{0};
	bool _first_run{false};
	bool _integral_reset_pending{false};

	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
