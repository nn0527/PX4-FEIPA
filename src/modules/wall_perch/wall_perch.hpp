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
 * @file wall_perch.hpp
 *
 * wall_perch — RC-triggered wall perching maneuver
 *
 * State machine:
 *   IDLE → SLOW_APPROACH → FLIP_TO_WALL → WALL_CAPTURE
 *   → WALL_HOLD or WALL_PIN → DETACH_ROTATE → RECOVER → EXIT → IDLE
 *   The start-switch rising edge takes control immediately; the former
 *   FRONT_WALL_DETECT and STABILIZE_HOVER states are retained only so the
 *   uORB state values remain backward compatible.
 */

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
#include <uORB/topics/wall_perch_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/actuator_motors.h>
#include <lib/systemlib/mavlink_log.h>

using namespace time_literals;
using namespace matrix;

class WallPerch : public ModuleBase<WallPerch>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	WallPerch();
	~WallPerch() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	enum class State : uint8_t {
		IDLE,
		FRONT_WALL_DETECT,
		STABILIZE_HOVER,
		SLOW_APPROACH,
		FLIP_TO_WALL,
		WALL_CAPTURE,
		WALL_HOLD,
		WALL_PIN,
		DETACH_ROTATE,
		RECOVER,
		EXIT,
		ABORT
	};

	void Run() override;
	void parameters_update(bool force);

	// State machine
	void update_state_machine(float dt);
	void enter_state(State new_state);
	const char *state_name(State s) const;

	// Condition helpers
	bool hold_condition(bool condition, hrt_abstime &start_time, float hold_time_s);

	// Sensor helpers
	bool sensors_valid();
	bool front_sensor_valid() const;
	bool top_sensor_valid() const;
	bool local_position_valid() const;
	void update_distance_sensors();

	// User input helpers
	bool user_start_requested();
	bool user_detach_requested();
	bool user_cancel_requested();

	// Distance condition checks
	bool front_ready();
	bool flip_ready();
	bool top_distance_sufficient() const;
	bool top_contact_ready();

	// Attitude computation
	Quatf compute_hover_attitude(float yaw);
	Quatf compute_wall_attitude(const Quatf &q_hover);
	float compute_approach_thrust(float dt, float pitch_rad);
	void update_wall_thrust_command(float dt);
	bool wall_thrust_at_max() const;

	// Math utilities
	float smoothstep5(float tau);
	Quatf slerp_quat(const Quatf &q0, const Quatf &q1, float s);
	float ramp(float from, float to, float duration);

	// Safety checks
	bool safety_ok();
	bool attitude_recovered();
	bool rate_safe();
	bool vz_safe();

	// Output
	void publish_attitude_setpoint(const Quatf &q_des, float thrust_norm);
	void publish_actuator_motors(float thrust);
	void publish_wall_perch_status();

	bool wall_attitude_ready() const;

	// -----------------------------------------------------------------------
	// Sensor data
	// -----------------------------------------------------------------------
	float _front_wall_distance_m{100.f};
	float _top_wall_distance_m{100.f};
	hrt_abstime _front_distance_ts{0};
	hrt_abstime _top_distance_ts{0};
	int8_t _front_sensor_instance{-1};
	int8_t _top_sensor_instance{-1};

	matrix::Vector3f _velocity{};
	matrix::Vector3f _angular_velocity{};
	matrix::Eulerf _attitude_euler{};
	matrix::Quatf _q_current{};
	float _current_yaw{0.f};
	float _current_altitude{0.f};
	hrt_abstime _local_position_ts{0};
	bool _local_position_data_valid{false};

	// -----------------------------------------------------------------------
	// State machine
	// -----------------------------------------------------------------------
	State _state{State::IDLE};
	hrt_abstime _state_entry_time{0};

	// Timers for hold conditions
	hrt_abstime _front_ready_start{0};
	hrt_abstime _flip_ready_start{0};
	hrt_abstime _top_contact_start{0};

	// Attitude references
	float _yaw_hold{0.f};
	Quatf _q_hover{};
	Quatf _q_wall{};
	Quatf _q_flip_start{};

	// Slerp tracking
	hrt_abstime _flip_start_time{0};
	hrt_abstime _detach_start_time{0};

	// Pin (mixer bypass) timing
	hrt_abstime _pin_start_time{0};
	hrt_abstime _motor_insufficient_start{0};
	float _pin_thrust_command{0.55f};

	// Approach timeout
	hrt_abstime _approach_start_time{0};

	// Sensor lpf init
	bool _lpf_front_initialized{false};
	bool _lpf_top_initialized{false};
	float _front_distance_lpf{100.f};
	float _top_distance_lpf{100.f};

	// Hover thrust from estimator
	float _hover_thrust{0.55f};
	float _hover_thrust_estimate{0.55f};
	bool _hover_thrust_estimate_valid{false};

	// Altitude is latched when the side-perch switch is turned on and is held
	// only during the horizontal approach. It cannot be held once body -Z is
	// horizontal at the wall attitude.
	float _altitude_hold_sp{0.f};
	float _altitude_error{0.f};
	float _altitude_error_integral{0.f};
	float _last_thrust_norm{0.f};

	// Vehicle armed status
	bool _armed{false};
	uint8_t _nav_state{0};

	// Switch raw values
	float _aux1_raw{0.f};
	float _aux2_raw{0.f};
	float _aux3_raw{0.f};
	float _aux4_raw{0.f};
	bool _start_switch_was_on{false};

	// Confirmed transition conditions are stored separately so publishing the
	// diagnostic topic never advances a hold timer.
	bool _flip_ready_confirmed{false};
	bool _top_contact_confirmed{false};

	// Mavlink log
	orb_advert_t _mavlink_log_pub{nullptr};
	hrt_abstime _last_status_log_time{0};
	bool _first_run{true};

	// -----------------------------------------------------------------------
	// Parameters
	// -----------------------------------------------------------------------
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::WP_ENABLE>) _param_wp_enable,
		(ParamInt<px4::params::WP_AUX_CH>) _param_wp_aux_ch,
		(ParamInt<px4::params::WP_DETACH_AUX_CH>) _param_wp_detach_aux_ch,
		(ParamInt<px4::params::WP_CANCEL_AUX_CH>) _param_wp_cancel_aux_ch,

		(ParamFloat<px4::params::WP_FRN_RD_DIST>) _param_wp_frn_rd_dist,
		(ParamFloat<px4::params::WP_FRN_RD_HLD>) _param_wp_frn_rd_hld,

		(ParamFloat<px4::params::WP_FLP_TRD_DIST>) _param_wp_flp_trd_dist,
		(ParamFloat<px4::params::WP_FLP_TRD_HLD>) _param_wp_flp_trd_hld,

		(ParamFloat<px4::params::WP_TOP_CT_DIST>) _param_wp_top_ct_dist,
		(ParamFloat<px4::params::WP_TOP_CT_HLD>) _param_wp_top_ct_hld,

		(ParamFloat<px4::params::WP_STAB_TIME>) _param_wp_stab_time,
		(ParamFloat<px4::params::WP_APPR_TIMEOUT>) _param_wp_appr_timeout,
		(ParamFloat<px4::params::WP_FLIP_TIME>) _param_wp_flip_time,
		(ParamFloat<px4::params::WP_CAPTURE_TIME>) _param_wp_capture_time,
		(ParamFloat<px4::params::WP_HOLD_TIME>) _param_wp_hold_time,
		(ParamFloat<px4::params::WP_DETACH_TIME>) _param_wp_detach_time,
		(ParamFloat<px4::params::WP_RECOVER_TIME>) _param_wp_recover_time,

		(ParamInt<px4::params::WP_WALL_AXIS>) _param_wp_wall_axis,
		(ParamFloat<px4::params::WP_WALL_ANGLE>) _param_wp_wall_angle,
		(ParamFloat<px4::params::WP_APPR_PITCH>) _param_wp_appr_pitch,

		(ParamFloat<px4::params::WP_THR_HOVER>) _param_wp_thr_hover,
		(ParamFloat<px4::params::WP_THR_APPROACH>) _param_wp_thr_approach,
		(ParamFloat<px4::params::WP_ALT_KP>) _param_wp_alt_kp,
		(ParamFloat<px4::params::WP_ALT_KI>) _param_wp_alt_ki,
		(ParamFloat<px4::params::WP_ALT_KD>) _param_wp_alt_kd,
		(ParamFloat<px4::params::WP_ALT_MAX>) _param_wp_alt_max,

		(ParamFloat<px4::params::WP_MU_EST>) _param_wp_mu_est,
		(ParamFloat<px4::params::WP_MARGIN>) _param_wp_margin,

		(ParamFloat<px4::params::WP_MAX_RATE>) _param_wp_max_rate,
		(ParamFloat<px4::params::WP_MAX_VZ_DOWN>) _param_wp_max_vz_down,
		(ParamFloat<px4::params::WP_MIN_ALT>) _param_wp_min_alt,
		(ParamFloat<px4::params::WP_RECOVER_RP>) _param_wp_recover_rp,
		(ParamFloat<px4::params::WP_SENS_TIMEOUT>) _param_wp_sens_timeout,

		(ParamBool<px4::params::WP_PIN_ENABLE>) _param_wp_pin_enable,
		(ParamFloat<px4::params::WP_PIN_PITCH>) _param_wp_pin_pitch,
		(ParamFloat<px4::params::WP_PIN_THR>) _param_wp_pin_thr,
		(ParamFloat<px4::params::WP_PIN_HOLD>) _param_wp_pin_hold,
		(ParamFloat<px4::params::WP_CONTACT_TMO>) _param_wp_contact_tmo
	)

	// -----------------------------------------------------------------------
	// uORB subscriptions
	// -----------------------------------------------------------------------
	// Both rangefinders use the standard multi-instance distance_sensor topic.
	// They are routed by orientation, not by publication order/instance number.
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// -----------------------------------------------------------------------
	// uORB publications
	// -----------------------------------------------------------------------
	uORB::Publication<wall_perch_status_s> _status_pub{ORB_ID(wall_perch_status)};
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	// -----------------------------------------------------------------------
	// Performance counters
	// -----------------------------------------------------------------------
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
