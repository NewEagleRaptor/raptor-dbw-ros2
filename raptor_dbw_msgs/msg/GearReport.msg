std_msgs/Header header

# Current gear enumeration
Gear state_actual
Gear state_desired

# Gear reject flag
bool reject

# Status
bool driver_activity
bool enabled

# Faults
bool gear_select_system_fault
bool gear_mismatch_flash

# Watchdog counter
uint8 rolling_counter
