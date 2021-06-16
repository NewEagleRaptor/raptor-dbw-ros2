Ignition ignition_cmd
bool horn_cmd
bool diff_lock

# Lights
TurnSignal turn_signal_cmd
HighBeam high_beam_cmd
LowBeam low_beam_cmd
RunningLights running_lights
OtherLights other_lights
bool mode_light_red
bool mode_light_yellow
bool mode_light_green
bool mode_light_blue

# Wipers
WiperFront front_wiper_cmd
WiperRear rear_wiper_cmd

# Doors
DoorRequest door_request_right_rear
DoorRequest door_request_left_rear
DoorRequest door_request_lift_gate
DoorLock door_lock_cmd

# Block driver input
bool block_standard_cruise_buttons
bool block_adaptive_cruise_buttons
bool block_turn_signal_stalk

uint8 rolling_counter

# TODO(NERaptor): add checksum support
