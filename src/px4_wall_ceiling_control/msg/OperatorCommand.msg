std_msgs/Header header

uint8 MODE_MANUAL=0
uint8 MODE_STANDBY=1
uint8 MODE_CEILING=2
uint8 MODE_WALL=3
uint8 MODE_ABORT=4
uint8 MODE_INCLINE=5

uint8 mode
bool enable_control
string source
string reason

# Ceiling-only CH12 confirmation after recovery; wall handback remains automatic.
time pilot_handover_request_time
