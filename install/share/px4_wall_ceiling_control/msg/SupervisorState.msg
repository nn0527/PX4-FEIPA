std_msgs/Header header

uint8 STATE_WAIT_LINK=0
uint8 STATE_MANUAL=1
uint8 STATE_STANDBY=2
uint8 STATE_PRESTREAM=3
uint8 STATE_CEILING_ACTIVE=4
uint8 STATE_WALL_ACTIVE=5
uint8 STATE_FAULT=6

uint8 MODE_NONE=0
uint8 MODE_CEILING=1
uint8 MODE_WALL=2

uint32 FAULT_NONE=0
uint32 FAULT_MAVROS=1
uint32 FAULT_OPERATOR=2
uint32 FAULT_ODOMETRY=4
uint32 FAULT_CEILING_RANGE=8
uint32 FAULT_WALL_RANGE=16
uint32 FAULT_UNKNOWN_MODE=32

uint8 state
uint8 active_mode
bool setpoint_stream_allowed
bool motion_allowed
bool mavros_connected
bool armed
bool offboard
bool operator_fresh
bool odom_fresh
bool ceiling_range_fresh
bool wall_range_fresh
uint32 fault_mask
string reason
