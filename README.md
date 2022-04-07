# raptor-dbw-ros2

This is a continuation of the [raptor_dbw_ros repo](https://github.com/NewEagleRaptor/raptor-dbw-ros) but in ROS2.  
This is the product of transferring the ROS1 codebase to ROS2.

This repository contains a collection of ROS2 packages which allow DBW kit developers to quickly implement a generic ROS2 node for interacting with the New Eagle Raptor controller.

# Packages

* can_dbc_parser - module for handling everything related to translating CAN messages to ROS
* raptor_dbw_can - DBW CAN driver
* raptor_dbw_joystick - a demo that allows you to use a game controller to interact with the DBW ROS2 node
* raptor_dbw_msgs - DBW ROS2 message definitions
* raptor_pdu - power distribution unit (PDU) driver. The PDU is a separate CAN-enabled device that's part of New Eagle's product line.
* raptor_pdu_msgs - PDU ROS2 message definitions

# Installing and building

## Official Releases
raptor-dbw-ros2 is being officially released through the ROS Build Farm.  
Latest release version is foxy-1.2.0.  
See the "Tags" tab for other releases.

## Building from source
raptor-dbw-ros2 can be built from source using colcon build. It also requires package kvaser-interface.

1. Create a ROS2 workspace. Instructions below assume the workspace was created in directory YourWorkspace.
2. cd YourWorkspace/src
3. git clone https://github.com/NewEagleRaptor/raptor-dbw-ros2.git

# Usage

## Running raptor_dbw_can with kvaser hardware:

1. Make sure kvaser-interface is built and installed first. You can get it from github:
    - From inside YourWorkspace/src:
        1. git clone https://github.com/astuff/kvaser_interface.git
        2. cd kvaser-interface
        3. git checkout ros2-master
2. clone this repository (see "Building from source")
3. modify the launch parameters file in raptor_dbw_can/launch/launch_params.yaml
    - "hardware_id" is the serial number (S/N) for the kvaser hardware. This must match your hardware.
    - "circuit_id" is the can channel number (0-n)
3. in the terminal, with the path set to the base of the workspace:
    - colcon build --packages-up-to raptor_dbw_can
    - ros2 launch raptor_dbw_can raptor_dbw_can_launch.py
