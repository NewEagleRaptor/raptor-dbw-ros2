std_msgs/Header header

Ignition ignition_state 
HornState horn_state 
OnOffState diff_lock_state

# Lights
TurnSignal turn_signal_state 
HighBeamState high_beam_state 
LowBeam low_beam_state 
RunningLights running_lights_state
OnOffState other_lights_state
OnOffState mode_light_red
OnOffState mode_light_yellow
OnOffState mode_light_green
OnOffState mode_light_blue

# Wipers
WiperFront front_wiper_state 
WiperRear rear_wiper_state 

# Doors
DoorState right_rear_door_state 
DoorState left_rear_door_state 
DoorState liftgate_door_state 
DoorLock door_lock_state

uint8 rolling_counter
