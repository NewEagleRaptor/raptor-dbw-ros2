std_msgs/Header header

# Control mode status
bool enabled
EngineControlMode control_type
EngineModeRequest mode_actual
EngineModeRequest mode_desired

# Fault handling
EngineKeyMismatch eng_key_mismatch
EngineFault fault
bool driver_activity

uint8 rolling_counter
