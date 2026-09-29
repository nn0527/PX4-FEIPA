std_msgs/Header header

uint8 STATE_NORMAL_FLIGHT=0
uint8 STATE_CEILING_ARMED=1
uint8 STATE_APPROACH=2
uint8 STATE_ATTACH_CONTROL=3
uint8 STATE_SURFACE_HOLD=4
uint8 STATE_DETACH=5
uint8 STATE_RECOVERY_HOVER=6
uint8 STATE_FAULT=7

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
