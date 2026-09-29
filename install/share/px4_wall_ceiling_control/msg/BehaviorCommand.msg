std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_CEILING=1
uint8 SOURCE_WALL=2

uint8 STATE_DISABLED=0
uint8 STATE_APPROACH=1
uint8 STATE_TRACK=2
uint8 STATE_RETREAT=3
uint8 STATE_FAULT=4

uint8 source
uint8 state
bool active
float32 target_distance_m
float32 max_speed_mps
string reason
