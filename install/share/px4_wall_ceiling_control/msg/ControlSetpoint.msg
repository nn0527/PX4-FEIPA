std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_CEILING=1
uint8 SOURCE_WALL=2

uint8 KIND_NONE=0
uint8 KIND_LOCAL=1
uint8 KIND_ATTITUDE=2

uint8 source
uint8 kind
bool valid
mavros_msgs/PositionTarget local
mavros_msgs/AttitudeTarget attitude
string reason
