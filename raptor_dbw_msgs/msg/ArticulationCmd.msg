# Control signals
ArticulationControlMode control_type
float32 angle_cmd         # deg - clockwise is positive

# Enables & limits
bool enable
bool ignore_driver
float32 velocity_limit    # deg/s

# Watchdog counter
uint8 rolling_counter

# TODO(NERaptor): add checksum support