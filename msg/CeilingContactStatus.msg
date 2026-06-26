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

float32 ceiling_distance      # Current top distance (m)
float32 target_distance       # Target top distance for attach control (m)
float32 compression           # Wheel compression = D0 - current_distance (m)
float32 target_compression    # Target compression (m)

# Control output for mc_pos_control
float32 thrust_body_z_sp      # Normalized thrust setpoint (negative = up in NED)
float32 approach_vz_sp        # Approach vertical speed (NED, negative = up)

# Switch states
bool ceiling_arm_switch_on    # CEILING_ENABLE_SWITCH (ch11/aux1)
bool detach_switch_on         # DETACH_SWITCH (ch12/aux2)

# Status flags
bool attach_confirmed         # Compression threshold reached
bool distance_stable          # Distance stable in target range
bool fault_detected           # Fault condition triggered
bool integral_reset_request   # Request mc_pos_control to reset integrals
bool wheel_stop_request       # Request to stop/disable wheel outputs

uint64 state_entry_time
uint32 fault_count
