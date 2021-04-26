# Control signals
ArticulationControlMode cmd
float32 angle             # deg

# Enables & limits
bool enable
bool ignore_driver
float32 velocity_limit    # deg/s

# Watchdog counter
uint8 rolling_counter

# TODO(NERaptor): add checksum support