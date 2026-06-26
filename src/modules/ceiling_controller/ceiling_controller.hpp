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
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/ceiling_contact_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
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
	void Run() override;
	void parameters_update(bool force);
	void update_state_machine(float dt);
	void enter_state(uint8_t new_state);
	bool check_attach_confirmed();
	bool check_distance_stable();
	bool check_fault_conditions();
	float compute_distance_control(float dt);
	void read_switches(bool &ceiling_arm, bool &detach);

	float _ceiling_distance{100.f};
	float _target_distance{0.5f};
	float _compression{0.f};
	matrix::Vector3f _velocity{};
	matrix::Eulerf _attitude_euler{};
	float _dist_error_integral{0.f};
	float _dist_error_prev{0.f};
	float _ceiling_distance_lpf{100.f};
	float _hover_thrust{0.72f};
	float _attach_baseline_thrust{-0.72f};
	bool _attach_baseline_initialized{false};
	bool _lpf_initialized{false};

	uint8_t _state{ceiling_contact_status_s::NORMAL_FLIGHT};
	hrt_abstime _state_entry_time{0};
	uint32_t _fault_count{0};
	static constexpr float COMPRESSION_THR{0.005f};
	static constexpr uint64_t ATTACH_TIME_THR{100_ms};
	hrt_abstime _attach_detect_start{0};
	hrt_abstime _dist_stable_start{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CEIL_D0>) _param_ceil_d0,
		(ParamFloat<px4::params::CEIL_COMP_TGT>) _param_ceil_comp_tgt,
		(ParamFloat<px4::params::CEIL_DIST_THR>) _param_ceil_dist_thr,
		(ParamFloat<px4::params::CEIL_VEL_THR>) _param_ceil_vel_thr,
		(ParamFloat<px4::params::CEIL_APPR_VZ>) _param_ceil_appr_vz,
		(ParamInt<px4::params::CEIL_APPR_TO>) _param_ceil_appr_to,
		(ParamFloat<px4::params::CEIL_DIST_KP>) _param_ceil_dist_kp,
		(ParamFloat<px4::params::CEIL_DIST_KI>) _param_ceil_dist_ki,
		(ParamFloat<px4::params::CEIL_DIST_KD>) _param_ceil_dist_kd,
		(ParamFloat<px4::params::CEIL_STABLE_T>) _param_ceil_stable_t,
		(ParamFloat<px4::params::CEIL_MAX_ROLL>) _param_ceil_max_roll,
		(ParamFloat<px4::params::CEIL_MAX_PITCH>) _param_ceil_max_pitch,
		(ParamFloat<px4::params::CEIL_MAX_THRUST>) _param_ceil_max_thrust,
		(ParamFloat<px4::params::CEIL_RAMP_DIST>) _param_ceil_ramp_dist,
		(ParamFloat<px4::params::CEIL_FLT_TC>) _param_ceil_flt_tc
	)

	orb_advert_t _mavlink_log_pub{nullptr};
	uORB::Publication<ceiling_contact_status_s> _status_pub{ORB_ID(ceiling_contact_status)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	bool _ceiling_arm_switch{false};
	bool _detach_switch{false};
	float _aux1_raw{0.f};
	float _aux2_raw{0.f};
	bool _last_ceiling_arm_switch{false};
	bool _last_detach_switch{false};
	hrt_abstime _last_status_log_time{0};
	bool _first_run{false};
	
	// 简单的订阅状态
	bool _distance_subscription_active{false};

	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
