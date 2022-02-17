cmake_minimum_required(VERSION 3.5)
project(raptor_dbw_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AcceleratorPedalCmd.msg"
  "msg/AcceleratorPedalReport.msg"
  "msg/ActuatorControlMode.msg"
  "msg/Brake2Report.msg"
  "msg/BrakeCmd.msg"
  "msg/BrakeReport.msg"
  "msg/ButtonState.msg"
  "msg/DoorLock.msg"
  "msg/DoorRequest.msg"
  "msg/DoorState.msg"
  "msg/DriverInputReport.msg"
  "msg/ExitReport.msg"
  "msg/FaultActionsReport.msg"
  "msg/Gear.msg"
  "msg/GearCmd.msg"  
  "msg/GearReport.msg"
  "msg/GlobalEnableCmd.msg"
  "msg/GpsReferenceReport.msg"
  "msg/GpsRemainderReport.msg"
  "msg/HighBeam.msg"
  "msg/HighBeamState.msg"
  "msg/HmiGlobalEnableReport.msg"
  "msg/HornState.msg"
  "msg/Ignition.msg"
  "msg/LowBeam.msg"
  "msg/LowBeamState.msg"
  "msg/LowVoltageSystemReport.msg"
  "msg/MiscCmd.msg"
  "msg/MiscReport.msg"
  "msg/OtherActuatorsReport.msg"
  "msg/ParkingBrake.msg"
  "msg/SonarArcNum.msg"  
  "msg/Steering2Report.msg"
  "msg/SteeringCmd.msg"
  "msg/SteeringReport.msg"
  "msg/SurroundReport.msg"  
  "msg/TirePressureReport.msg"
  "msg/TurnSignal.msg"  
  "msg/TwistCmd.msg"
  "msg/WatchdogStatus.msg"
  "msg/WheelPositionReport.msg"
  "msg/WheelSpeedReport.msg"
  "msg/WheelSpeedType.msg"
  "msg/WiperFront.msg"  
  "msg/WiperRear.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
