# Wall perch maneuver status
uint64 timestamp

uint8 IDLE = 0
uint8 FRONT_WALL_DETECT = 1
uint8 STABILIZE_HOVER = 2
uint8 SLOW_APPROACH = 3
uint8 FLIP_TO_WALL = 4
uint8 WALL_CAPTURE = 5
uint8 WALL_HOLD = 6
uint8 WALL_PIN = 7
uint8 DETACH_ROTATE = 8
uint8 RECOVER = 9
uint8 EXIT = 10
uint8 ABORT = 11

uint8 state

bool active                  # true when wall_perch is publishing attitude setpoint
bool front_ready             # front wall distance condition met
bool flip_ready              # flip trigger distance met
bool top_contact_ready       # top sensor confirms wall contact
bool front_sensor_valid      # fresh body +X range data is available
bool top_sensor_valid        # fresh body -Z suction/contact range data is available
bool local_position_valid    # fresh valid local z and vz are available
bool altitude_hold_active    # wall_perch is holding switch-on altitude during approach
bool wall_attitude_ready     # measured pitch has reached WP_PIN_PITCH
bool motor_boost_active      # wall-normal thrust has risen above the recorded hover value
int8 front_sensor_instance   # selected distance_sensor instance, -1 if never seen
int8 top_sensor_instance     # selected distance_sensor instance, -1 if never seen
float32 front_wall_distance_m
float32 top_wall_distance_m
float32 altitude_setpoint_m
float32 altitude_m
float32 altitude_error_m
float32 progress              # 0..1 progress within current state
float32 thrust_norm           # current normalized thrust command
bool failsafe_triggered       # any safety violation triggered
