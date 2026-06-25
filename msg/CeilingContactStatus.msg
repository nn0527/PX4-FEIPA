# Ceiling contact controller status.

uint64 timestamp          # time since system start (microseconds)

uint8 NORMAL_FLIGHT = 0
uint8 CEILING_ARM_MODE = 1
uint8 APPROACH_MODE = 2
uint8 ATTACH_CONTROL_MODE = 3
uint8 SURFACE_MANUAL_MODE = 4
uint8 DETACH_MODE = 5
uint8 RECOVERY_HOVER_MODE = 6

uint8 state               # current ceiling controller state

float32 ceiling_distance  # [m] filtered distance from upward sensor to ceiling
float32 target_distance   # [m] target distance during attach/detach control
float32 compression       # [m] estimated compression, CEIL_D0 - ceiling_distance
float32 target_compression # [m] target compression

float32 thrust_body_z_sp  # normalized body-z thrust override request, NAN if inactive
float32 approach_vz_sp    # [m/s] vertical speed request during approach, NAN if inactive

bool ceiling_arm_switch_on
bool detach_switch_on
bool attach_confirmed
bool distance_stable
bool fault_detected
bool integral_reset_request
bool wheel_stop_request

uint64 state_entry_time   # time since system start when current state was entered (microseconds)
uint32 fault_count
