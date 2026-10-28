std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_CEILING=1
uint8 SOURCE_WALL=2

uint8 KIND_NONE=0
uint8 KIND_VERTICAL_VELOCITY=1
uint8 KIND_ATTITUDE_THRUST=2

uint8 source
uint8 kind
bool acquire_request
bool keep_ownership
bool valid

# ROS-side ENU world convention.
float32 velocity_z_enu_mps
# ROS-side ENU/FLU attitude convention; MAVROS performs PX4 conversion.
geometry_msgs/Quaternion attitude_setpoint
# Positive normalized collective thrust expected by mavros_msgs/AttitudeTarget.
float32 thrust_normalized
string reason

# Control metadata travels atomically with attitude/thrust. Header.stamp
# is the generation time in the ROS clock. The mux retains it in diagnostics
# while the MAVROS output header carries the actual transport publication time.
# Motion-enabled ceiling candidates also set phase, maneuver ID and sequence.
bool phase_valid
uint8 control_phase
string maneuver_id
uint64 control_sequence
bool horizontal_velocity_enabled
bool horizontal_velocity_active
bool approach_paused

# Departure support remains continuous through recovery/handover. Limit is
# bounded by the mux global cap; it is not a request to bypass hard limits.
bool recovery_support_valid
float32 recovery_thrust_limit

# Atomic with this candidate's phase, attitude and collective. A range outage
# may retain bounded support; range_feedback_valid then remains false.
uint8 detach_task_phase
uint32 detach_range_episode
bool range_height_control
bool range_feedback_valid

# Range departure before first lock: attitude/feedforward support, no Z/vz loop.
bool departure_support_only

# Captured-attitude demo recovery profile (10 degree output envelope).
bool recover_attitude_locked

# Ceiling final-envelope reference, atomic with this candidate's phase/attitude.
float32 ceiling_yaw_reference_rad

# Ceiling ABORT subtype, atomic with phase and control output.
bool ceiling_abort_land_requested

# Atomic ceiling phase/feedback gate; leaves wall assistance unchanged.
bool ceiling_rc_assist_blocked
