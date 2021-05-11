std_msgs/Header header

# Control status
bool enabled
ArticulationControlMode control_type
float32 angle_actual      # deg - clockwise is positive
float32 angle_desired     # deg - clockwise is positive
float32 angle_steer       # deg - clockwise is positive

# Fault handling
ArticulationFault fault
bool driver_activity
uint8 rolling_counter
