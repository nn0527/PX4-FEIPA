std_msgs/Header header

bool ceiling_valid
bool front_valid

# Age of the last valid sample as measured at this node. Infinity means that
# no valid sample has been received since startup.
float32 ceiling_age_s
float32 front_age_s

# Monotonic counters allow consumers and tests to distinguish real samples
# from repeated status publications.
uint32 ceiling_sequence
uint32 front_sequence
uint32 ceiling_invalid_count
uint32 front_invalid_count
