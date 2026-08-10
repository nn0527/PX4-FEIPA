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

bool active                  # true when wall_perch owns attitude or direct-motor output
bool front_ready             # front wall distance condition met
bool flip_ready              # flip trigger distance met
bool top_contact_ready       # top sensor confirms wall contact
float32 front_wall_distance_m
float32 top_wall_distance_m
float32 progress              # 0..1 progress within current state
float32 thrust_norm           # current normalized thrust command
bool failsafe_triggered       # any safety violation triggered
