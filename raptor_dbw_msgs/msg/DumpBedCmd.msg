# Control signals
DumpBedControlMode control_type
DumpBedModeRequest mode_type
float32 lever_pct         # 0-100%
float32 angle_cmd         # deg

# Enables & limits
bool enable
bool ignore_driver
float32 velocity_limit    # deg/s

# Watchdog counter
uint8 rolling_counter

# TODO(NERaptor): add checksum support
