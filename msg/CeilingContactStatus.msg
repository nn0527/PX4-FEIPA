# Ceiling contact inspection system status
uint64 timestamp

uint8 NORMAL_FLIGHT = 0
uint8 CEILING_ARM_MODE = 1
uint8 APPROACH_MODE = 2
uint8 ATTACH_CONTROL_MODE = 3
uint8 SURFACE_MANUAL_MODE = 4
uint8 DETACH_MODE = 5
uint8 RECOVERY_HOVER_MODE = 6

uint8 state

# Vertical controller selected by ceiling_controller. At most one of
# vertical_velocity_sp and thrust_body_z_sp is finite for an active mode.
uint8 Z_CONTROL_MODE_NONE = 0
uint8 Z_CONTROL_MODE_VELOCITY = 1
uint8 Z_CONTROL_MODE_DIRECT_THRUST = 2
uint8 z_control_mode

uint8 DETACH_PHASE_NONE = 0
uint8 DETACH_PHASE_UNLOAD = 1
uint8 DETACH_PHASE_CLEARANCE = 2
uint8 detach_phase

float32 ceiling_distance      # Current top distance (m)
float32 target_distance       # Target top distance for attach control (m)
float32 compression           # Wheel compression = D0 - current_distance (m)
float32 target_compression    # Target compression (m)

# Control output for mc_pos_control
float32 thrust_body_z_sp      # Normalized thrust setpoint (negative = up in NED)
float32 approach_vz_sp        # Approach vertical speed (NED, negative = up)
float32 vertical_velocity_sp  # Generic vertical speed command (NED, negative = up)

# Switch states
bool ceiling_arm_switch_on    # CEILING_ENABLE_SWITCH (ch11/aux1)
bool detach_switch_on         # DETACH_SWITCH (ch12/aux2)

# Status flags
bool attach_confirmed         # Compression threshold reached
bool distance_stable          # Distance stable in target range
bool fault_detected           # Fault condition triggered
bool integral_reset_request   # Request mc_pos_control to reset integrals
bool wheel_stop_request       # Request to stop/disable wheel outputs
bool contact_active           # Confirmed ceiling contact, including detach unload
bool input_valid              # Required control and sensor inputs are fresh and valid
bool distance_sensor_valid    # Selected upward-facing range sample is valid and fresh
bool rearm_required           # AUX1 must be released before another attach attempt

uint32 FAULT_NONE = 0
uint32 FAULT_MANUAL_CONTROL = 1
uint32 FAULT_DISTANCE_SENSOR = 2
uint32 FAULT_VERTICAL_STATE = 4
uint32 FAULT_ATTITUDE = 8
uint32 FAULT_CONTROL_MODE = 16
uint32 FAULT_APPROACH_TIMEOUT = 32
uint32 FAULT_ATTACH_TIMEOUT = 64
uint32 FAULT_DETACH_TIMEOUT = 128
uint32 FAULT_CONTACT_LOST = 256
uint32 FAULT_PARAMETER = 512
uint32 fault_reason

float32 hover_thrust_baseline # Frozen pre-contact hover thrust used for handover

uint64 state_entry_time
uint32 fault_count
