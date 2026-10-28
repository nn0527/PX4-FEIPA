std_msgs/Header header

uint8 STATE_NORMAL_FLIGHT=0
uint8 STATE_CEILING_ARMED=1
uint8 STATE_APPROACH=2
uint8 STATE_ATTACH_CONTROL=3
uint8 STATE_SURFACE_HOLD=4
uint8 STATE_DETACH=5
uint8 STATE_RECOVERY_HOVER=6
uint8 STATE_ABORT=7

uint8 state
string state_name
bool enabled
bool acquire_request
bool keep_ownership
bool candidate_valid
bool range_fresh
bool odom_fresh
bool flight_state_fresh
bool attach_confirmed
bool distance_stable
bool fault_detected
bool detach_failed
# Latched APPROACH timeout: keep FAULT output until actual POSCTL handover.
bool approach_timed_out
bool integral_reset_request
bool wheel_stop_request
bool mechanism_release_requested
bool mechanism_release_confirmed
float32 ceiling_distance_raw_m
float32 ceiling_distance_filtered_m
float32 target_distance_m
float32 compression_m
float32 target_compression_m
# ROS/MAVROS ENU convention: positive z means upward/toward a horizontal ceiling.
float32 approach_velocity_z_enu_mps
# PX4 body FRD convention retained from the reference: negative z is upward thrust.
float32 thrust_body_z_normalized
bool attitude_setpoint_valid
geometry_msgs/Quaternion attitude_setpoint
uint32 fault_count
string reason

# Console test gates and ENU height-difference feedback.
bool activation_ready
float32 detach_start_height_m
float32 descended_height_m
bool detach_height_ready

# Barometer source and frozen ceiling-HOLD pressure baseline (Pa).
string height_source
bool baro_fresh
bool baro_reference_ready
float32 baro_pressure_pa
float32 hold_pressure_pa

# Ceiling POSCTL capture, free-flight motion envelope and recovery diagnostics.
bool motion_control_enabled
bool motion_ready
bool baseline_ready
bool motion_paused
bool recovery_ready
float32 horizontal_drift_m
float32 horizontal_speed_mps
float32 actual_tilt_deg
float32 actual_body_rate_deg_s
float32 yaw_reference_rad
float32 recovery_target_height_m
float32 target_vertical_speed_mps
string motion_reason

# HOLD watchdog: explicit abort requests PX4 AUTO.LAND through the mode manager.
bool abort_land_requested
bool hold_timed_out
string abort_maneuver_id
float32 hold_elapsed_s
float32 hold_timeout_s

# Ceiling manual/automatic horizontal coordination.
bool rc_manual_coordination_enabled
bool rc_manual_feedback_ready
bool rc_manual_active
float32 rc_position_weight

# Stable recovery attitude is held until a later CH12 handover confirmation.
bool recovery_attitude_locked
time recovery_lock_time

# Selected fused height feedback and independent pressure comparison.
bool height_feedback_valid
bool height_reference_ready
float32 fused_height_m
float32 hold_fused_height_m
float32 fused_vertical_speed_mps
float32 baro_descended_height_m

# Current-entry descent reference, DETACH-only boundary and PI diagnostics.
float32 detach_target_drop_m
float32 detach_max_drop_m
float32 detach_target_height_m
float32 vertical_integral_thrust
float32 vertical_support_thrust
float32 vertical_proportional_thrust
