std_msgs/Header header

# Control status
bool enabled
DumpBedControlMode control_type
DumpBedModeRequest mode_actual
DumpBedModeRequest mode_desired
float32 angle_actual              # deg
float32 angle_desired             # deg
float32 lever_pct_actual          # 0-100%
float32 lever_pct_desired         # 0-100%

# Fault handling
DumpBedFault fault
bool driver_activity
uint8 rolling_counter
