# Feedback for an actually published ceiling candidate; never a flight command.
std_msgs/Header header
time candidate_stamp
string maneuver_id
uint64 control_sequence
uint8 control_phase
bool input_valid
bool stick_active
# Body roll/pitch trim before the final combined attitude envelope, radians.
float32 roll_trim_rad
float32 pitch_trim_rad
