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

