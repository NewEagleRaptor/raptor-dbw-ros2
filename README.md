# raptor-dbw-ros2

This is a continuation of the [raptor_dbw_ros repo](https://github.com/NewEagleRaptor/raptor-dbw-ros) but in ROS2
This is the product of transferring the ROS1 codebase to ROS2

Running raptor_dbw_can with kvaser hardware:
1. make sure kvaser-interface is built and installed first
2. clone this repository
3. modify the launch parameters file in raptor_dbw_can/launch/launch_params.yaml
    -"hardware_id" is the serial number (S/N) for the kvaser hardware. This must match your hardware
    -"circuit_id" is the can channel number (0-n)
3. in the terminal, with the path set to the base the workspace:
    - colcon build --packages-up-to raptor_dbw_can
    - ros2 launch raptor_dbw_can raptor_dbw_can_launch.py
