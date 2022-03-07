// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \brief This file defines the RaptorDbwCAN class.
 * \copyright Copyright 2021 New Eagle LLC
 * \file raptor_dbw_can.hpp
 */

#ifndef RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
#define RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <raptor_pdu_msgs/msg/relay_command.hpp>
#include <raptor_pdu_msgs/msg/relay_state.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/actuator_control_mode.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/brake_report.hpp>
#include <raptor_dbw_msgs/msg/driver_input_report.hpp>
#include <raptor_dbw_msgs/msg/exit_report.hpp>
#include <raptor_dbw_msgs/msg/fault_actions_report.hpp>
#include <raptor_dbw_msgs/msg/gear_cmd.hpp>
#include <raptor_dbw_msgs/msg/gear_report.hpp>
#include <raptor_dbw_msgs/msg/global_enable_cmd.hpp>
#include <raptor_dbw_msgs/msg/gps_reference_report.hpp>
#include <raptor_dbw_msgs/msg/gps_remainder_report.hpp>
#include <raptor_dbw_msgs/msg/hmi_global_enable_report.hpp>
#include <raptor_dbw_msgs/msg/low_voltage_system_report.hpp>
#include <raptor_dbw_msgs/msg/misc_cmd.hpp>
#include <raptor_dbw_msgs/msg/misc_report.hpp>
#include <raptor_dbw_msgs/msg/other_actuators_report.hpp>
#include <raptor_dbw_msgs/msg/steering2_report.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/surround_report.hpp>
#include <raptor_dbw_msgs/msg/tire_pressure_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_position_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "raptor_dbw_can/dispatch.hpp"

using namespace std::chrono_literals;  // NOLINT

using can_msgs::msg::Frame;
using geometry_msgs::msg::TwistStamped;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::JointState;
using std_msgs::msg::Bool;
using std_msgs::msg::Empty;
using std_msgs::msg::String;

using raptor_pdu_msgs::msg::RelayCommand;

using raptor_dbw_msgs::msg::AcceleratorPedalCmd;
using raptor_dbw_msgs::msg::AcceleratorPedalReport;
using raptor_dbw_msgs::msg::ActuatorControlMode;
using raptor_dbw_msgs::msg::Brake2Report;
using raptor_dbw_msgs::msg::BrakeCmd;
using raptor_dbw_msgs::msg::BrakeReport;
using raptor_dbw_msgs::msg::ButtonState;
using raptor_dbw_msgs::msg::DoorLock;
using raptor_dbw_msgs::msg::DoorRequest;
using raptor_dbw_msgs::msg::DoorState;
using raptor_dbw_msgs::msg::DriverInputReport;
using raptor_dbw_msgs::msg::ExitReport;
using raptor_dbw_msgs::msg::FaultActionsReport;
using raptor_dbw_msgs::msg::Gear;
using raptor_dbw_msgs::msg::GearCmd;
using raptor_dbw_msgs::msg::GearReport;
using raptor_dbw_msgs::msg::GlobalEnableCmd;
using raptor_dbw_msgs::msg::GpsReferenceReport;
using raptor_dbw_msgs::msg::GpsRemainderReport;
using raptor_dbw_msgs::msg::HighBeam;
using raptor_dbw_msgs::msg::HighBeamState;
using raptor_dbw_msgs::msg::HmiGlobalEnableReport;
using raptor_dbw_msgs::msg::HornState;
using raptor_dbw_msgs::msg::Ignition;
using raptor_dbw_msgs::msg::LowBeam;
using raptor_dbw_msgs::msg::LowBeamState;
using raptor_dbw_msgs::msg::LowVoltageSystemReport;
using raptor_dbw_msgs::msg::MiscCmd;
using raptor_dbw_msgs::msg::MiscReport;
using raptor_dbw_msgs::msg::OtherActuatorsReport;
using raptor_dbw_msgs::msg::ParkingBrake;
using raptor_dbw_msgs::msg::SonarArcNum;
using raptor_dbw_msgs::msg::Steering2Report;
using raptor_dbw_msgs::msg::SteeringCmd;
using raptor_dbw_msgs::msg::SteeringReport;
using raptor_dbw_msgs::msg::SurroundReport;
using raptor_dbw_msgs::msg::TirePressureReport;
using raptor_dbw_msgs::msg::TurnSignal;
// using raptor_dbw_msgs::msg::TwistCmd;
// using raptor_dbw_msgs::msg::WatchdogStatus;
using raptor_dbw_msgs::msg::WheelPositionReport;
using raptor_dbw_msgs::msg::WheelSpeedReport;
using raptor_dbw_msgs::msg::WiperFront;
using raptor_dbw_msgs::msg::WiperRear;

namespace raptor_dbw_can
{
/** \brief Class for converting Raptor DBW messages between CAN & ROS */
class RaptorDbwCAN : public rclcpp::Node
{
public:
/** \brief Default constructor.
 * \param[in] options The options for this node.
 * \param[in] dbw_dbc_file The name of the DBC file to use.
 * \param[in] max_steer_angle Maximum steering angle allowed, deg
 */
  explicit RaptorDbwCAN(
    const rclcpp::NodeOptions & options,
    std::string dbw_dbc_file,
    float max_steer_angle);
  ~RaptorDbwCAN();

private:
/** \brief If DBW is enabled && there are active driver overrides,
 *    do not send commands on the overridden system. (200ms period)
 */
  void timer_200ms_Callback();

/** \brief If DBW is enabled && messages are timed out,
 *    send error values. (10ms period)
 */
  void timer_10ms_Callback();

/** \brief Attempt to enable the DBW system.
 * \param[in] msg Enable message (must not be null)
 */
  void recvEnable(const Empty::SharedPtr msg);

/** \brief Attempt to disable the DBW system.
 * \param[in] msg Disable message (must not be null)
 */
  void recvDisable(const Empty::SharedPtr msg);

/** \brief Convert reports received over CAN into ROS messages.
 * \param[in] msg The message received over CAN.
 */
  void recvCAN(const Frame::SharedPtr msg);

/** \brief Convert an Accel Pedal Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvAccelPedalRpt(const Frame::SharedPtr msg);

/** \brief Convert a Brake Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvBrakeRpt(const Frame::SharedPtr msg);

/** \brief Convert a Brake2 Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvBrake2Rpt(const Frame::SharedPtr msg);

/** \brief Convert a Driver Input Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvDriverInputRpt(const Frame::SharedPtr msg);

/** \brief Convert an Exit Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvExitRpt(const Frame::SharedPtr msg);

/** \brief Convert a Fault Action Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvFaultActionRpt(const Frame::SharedPtr msg);

/** \brief Convert a Gear Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvGearRpt(const Frame::SharedPtr msg);

/** \brief Convert a GPS Reference Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvGpsReferenceRpt(const Frame::SharedPtr msg);

/** \brief Convert a GPS Remainder Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvGpsRemainderRpt(const Frame::SharedPtr msg);

/** \brief Convert an IMU Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvImuRpt(const Frame::SharedPtr msg);

/** \brief Convert an IMU Report 2 received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvImu2Rpt(const Frame::SharedPtr msg);

/** \brief Convert a Low Voltage System Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvLowVoltageSystemRpt(const Frame::SharedPtr msg);

/** \brief Convert a Misc. Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvMiscRpt(const Frame::SharedPtr msg);

/** \brief Convert an Other Actuators Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvOtherActuatorsRpt(const Frame::SharedPtr msg);

/** \brief Convert a Steering Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvSteeringRpt(const Frame::SharedPtr msg);

/** \brief Convert a Steering2 Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvSteering2Rpt(const Frame::SharedPtr msg);

/** \brief Convert a Surround Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvSurroundRpt(const Frame::SharedPtr msg);

/** \brief Convert a Tire Pressure Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvTirePressureRpt(const Frame::SharedPtr msg);

/** \brief Convert a VIN Report received over CAN into a ROS message. Message is sent over multiple
 *    frames & published once all data is received.
 * \param[in] msg The message received over CAN.
 */
  void recvVinRpt(const Frame::SharedPtr msg);

/** \brief Convert a Wheel Position Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvWheelPositionRpt(const Frame::SharedPtr msg);

/** \brief Convert a Wheel Speed Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvWheelSpeedRpt(const Frame::SharedPtr msg);

/** \brief Convert an Accelerator Pedal Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvAcceleratorPedalCmd(const AcceleratorPedalCmd::SharedPtr msg);

/** \brief Convert a Brake Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvBrakeCmd(const BrakeCmd::SharedPtr msg);

/** \brief Convert a Gear Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvGearCmd(const GearCmd::SharedPtr msg);

/** \brief Convert a Global Enable Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvGlobalEnableCmd(const GlobalEnableCmd::SharedPtr msg);

/** \brief Convert an IMU Command sent as a ROS message into CAN messages.
 *
 * Uses Z-up axis system:
 * +x = forward,
 * +y = left,
 * +z = up
 *
 * roll = rotation around X axis
 * pitch = rotation around Y axis
 * yaw = rotation around Z axis
 * longitudinal = +/- X axis
 * lateral = +/- Y axis
 * vertical = +/- Z axis
 *
 * Note:
 * Yaw/Pitch/Roll commands are received in
 * Z-down orientation & converted to Z-up
 *
 * \param[in] msg The message to send over CAN.
 */
  void recvImuCmd(const Imu::SharedPtr msg);

/** \brief Convert a Misc. Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvMiscCmd(const MiscCmd::SharedPtr msg);

/** \brief Convert a Steering Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvSteeringCmd(const SteeringCmd::SharedPtr msg);

  // Time-related variables
  rclcpp::TimerBase::SharedPtr timer_200ms;
  rclcpp::TimerBase::SharedPtr timer_10ms_;
  rclcpp::Clock m_clock;
  static constexpr int64_t CLOCK_1_SEC = 1000;  // duration in milliseconds
  static constexpr double NSEC_TO_MSEC = 1000.0F;  // convert nanoseconds to milliseconds
  static constexpr double IMU_TIMEOUT_MSEC = 15.0F;  // 15 ms

  /* These timestamps are saved to detect timeouts
   */
  rclcpp::Time t_stamp_last_imu_cmd{};

  // Parameters from launch
  std::string dbw_dbc_file_;
  float max_steer_angle_;

  /* These commands & reports are stored
   * because they need data from multiple sources.
   */
  Imu m_imu_rpt{};

  bool m_seen_imu2_rpt{false};

  // Other useful items

  // Constants
  static constexpr double DEG_TO_RAD = M_PI / 180.0F;

  /** \brief Enumeration of driver ignores */
  enum ListIgnores
  {
    IGNORE_ACCEL = 0,  /**< Acceleration pedal ignore */
    IGNORE_STEER,      /**< Steering ignore */
    NUM_IGNORES   /**< Total number of driver ignores */
  };

  /** \brief Enumeration of driver overrides */
  enum ListOverrides
  {
    OVR_ACCEL = 0,  /**< Acceleration pedal override */
    OVR_BRAKE,      /**< Brake override */
    OVR_GEAR,       /**< PRND gear override */
    OVR_STEER,      /**< Steering override */
    NUM_OVERRIDES   /**< Total number of driver overrides */
  };

  /** \brief Enumeration of system faults */
  enum ListFaults
  {
    FAULT_ACCEL = 0,      /**< Acceleration pedal fault */
    FAULT_BRAKE,          /**< Brake fault */
    FAULT_STEER,          /**< Steering fault */
    FAULT_WATCH,          /**< Watchdog fault */
    NUM_SERIOUS_FAULTS,   /**< Total number of serious faults (disables DBW) */
    FAULT_WATCH_BRAKES = NUM_SERIOUS_FAULTS,  /**< Watchdog braking fault */
    FAULT_WATCH_WARN,     /**< Watchdog non-braking fault warning */
    NUM_FAULTS            /**< Total number of system faults */
  };

  /** \brief Enumeration of system enables */
  enum ListEnables
  {
    EN_ACCEL = 0,   /**< Acceleration pedal system enabled */
    EN_BRAKE,       /**< Brake system enabled */
    EN_STEER,       /**< Steering system enabled */
    EN_DBW,         /**< DBW system enabled */
    EN_DBW_PREV,    /**< DBW system previously enabled (track edge) */
    NUM_ENABLES     /**< Total number of system enables */
  };

  // Helps print warning messages
  const std::string OVR_SYSTEM[NUM_OVERRIDES] = {
    "accelerator pedal",
    "brake",
    "PRND gear",
    "steering"
  };
  const std::string FAULT_SYSTEM[NUM_SERIOUS_FAULTS] = {
    "accelerator pedal",
    "brake",
    "steering",
    "watchdog"
  };

  bool ignores_[NUM_IGNORES];
  bool overrides_[NUM_OVERRIDES];
  bool faults_[NUM_FAULTS];
  bool enables_[NUM_ENABLES];

/** \brief Check for an active fault.
 * \returns TRUE if there is any active fault, FALSE otherwise
 */
  inline bool fault()
  {
    return faults_[FAULT_BRAKE] || faults_[FAULT_ACCEL] || faults_[FAULT_STEER] ||
           faults_[FAULT_WATCH];
  }

/** \brief Check for an active driver override.
 * \returns TRUE if there is any active driver override, FALSE otherwise
 */
  inline bool override () {return overrides_[OVR_BRAKE] ||
           (!ignores_[IGNORE_ACCEL] && overrides_[OVR_ACCEL]) ||
           (!ignores_[IGNORE_STEER] && overrides_[OVR_STEER]) || overrides_[OVR_GEAR];}

/** \brief Check for an active driver override.
 * \returns TRUE if DBW is enabled && there is any active driver override, FALSE otherwise
 */
  inline bool clear() {return enables_[EN_DBW] && override ();}

/** \brief Check whether the DBW Node is in control of the vehicle.
 * \returns TRUE if DBW is enabled && there are no active faults or driver overrides,
 *          FALSE otherwise
 */
  inline bool enabled() {return enables_[EN_DBW] && !fault() && !override ();}

/** \brief DBW Enabled needs to publish when its state changes.
 * \returns TRUE when DBW enable state changes, FALSE otherwise
 */
  bool publishDbwEnabled();

/** \brief Checks faults & overrides to establish DBW control */
  void enableSystem();

/** \brief Disables DBW control */
  void disableSystem();

  /** \brief Set the specified override
   * \param[in] which_ovr Which override to set
   * \param[in] override The value to set the override to
   * \param[in] ignore The value to skip the override
   */
  void setOverride(ListOverrides which_ovr, bool override, bool ignore);

  /** \brief Set the specified fault; these faults disable DBW control when active
   * \param[in] which_fault Which fault to set
   * \param[in] fault The value to set the fault to
   */
  void setFault(ListFaults which_fault, bool fault);

  /** \brief Set a Watchdog fault & track fault source
   * \param[in] fault The value to set the fault to
   * \param[in] src Fault source is a non-braking fault
   * \param[in] braking Fault source is a braking fault
   */
  void faultWatchdog(bool fault, uint8_t src, bool braking);

  /** \brief Set a Watchdog fault & track fault source;
   *    keep current status of braking fault source
   * \param[in] fault The value to set the fault to
   * \param[in] src Fault source is a non-braking fault (default == none)
   */
  void faultWatchdog(bool fault, uint8_t src = 0);

  /** \brief Enumeration of vehicle joints */
  enum ListJoints
  {
    JOINT_FL = 0,   /**< Front left wheel */
    JOINT_FR,       /**< Front right wheel */
    JOINT_RL,       /**< Rear left wheel */
    JOINT_RR,       /**< Rear right wheel */
    JOINT_SL,       /**< Steering left */
    JOINT_SR,       /**< Steering right */
    JOINT_COUNT,    /**< Number of joints */
  };

  JointState joint_state_;

/** \brief Calculates & publishes joint states based on updated steering report.
 *    Overloaded function.
 * \param[in] stamp Updated time stamp
 * \param[in] steering Updated steering report
 */
  void publishJointStates(
    const rclcpp::Time stamp,
    const SteeringReport steering);

/** \brief Calculates & publishes joint states based on updated wheel speed report.
 *    Overloaded function.
 * \param[in] stamp Updated time stamp
 * \param[in] wheels Updated wheel speed report
 */
  void publishJointStates(
    const rclcpp::Time stamp,
    const WheelSpeedReport wheels);

  /** \brief Scale a message value to a real value using gain & offset
   * \param[in] in_val input value
   * \param[in] gain gain to apply
   * \param[in] offset offset to apply
   * \returns the scaled value as a double
   */
  double applyScaling(uint32_t in_val, double gain, double offset);

  // Licensing
  std::string vin_;

  // Frame ID
  std::string frame_id_;

  // Buttons (enable/disable)
  bool buttons_;

  // Ackermann steering
  double acker_wheelbase_;
  double acker_track_;
  double steering_ratio_;

  // Subscribed topics
  rclcpp::Subscription<Empty>::SharedPtr sub_enable_;
  rclcpp::Subscription<Empty>::SharedPtr sub_disable_;
  rclcpp::Subscription<Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<AcceleratorPedalCmd>::SharedPtr sub_accelerator_pedal_;
  rclcpp::Subscription<BrakeCmd>::SharedPtr sub_brake_;
  rclcpp::Subscription<GearCmd>::SharedPtr sub_gear_;
  rclcpp::Subscription<GlobalEnableCmd>::SharedPtr sub_global_enable_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<MiscCmd>::SharedPtr sub_misc_;
  rclcpp::Subscription<SteeringCmd>::SharedPtr sub_steering_;

  // Published topics
  rclcpp::Publisher<Bool>::SharedPtr pub_sys_enable_;
  rclcpp::Publisher<Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<AcceleratorPedalReport>::SharedPtr pub_accel_pedal_;
  rclcpp::Publisher<BrakeReport>::SharedPtr pub_brake_;
  rclcpp::Publisher<Brake2Report>::SharedPtr pub_brake_2_report_;
  rclcpp::Publisher<DriverInputReport>::SharedPtr pub_driver_input_;
  rclcpp::Publisher<ExitReport>::SharedPtr pub_exit_report_;
  rclcpp::Publisher<FaultActionsReport>::SharedPtr pub_fault_actions_report_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<GpsReferenceReport>::SharedPtr pub_gps_reference_report_;
  rclcpp::Publisher<GpsRemainderReport>::SharedPtr pub_gps_remainder_report_;
  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<LowVoltageSystemReport>::SharedPtr pub_low_voltage_system_;
  rclcpp::Publisher<MiscReport>::SharedPtr pub_misc_;
  rclcpp::Publisher<OtherActuatorsReport>::SharedPtr pub_other_actuators_report_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<Steering2Report>::SharedPtr pub_steering_2_report_;
  rclcpp::Publisher<SurroundReport>::SharedPtr pub_surround_;
  rclcpp::Publisher<TirePressureReport>::SharedPtr pub_tire_pressure_;
  rclcpp::Publisher<String>::SharedPtr pub_vin_;
  rclcpp::Publisher<WheelPositionReport>::SharedPtr pub_wheel_positions_;
  rclcpp::Publisher<WheelSpeedReport>::SharedPtr pub_wheel_speeds_;

  NewEagle::Dbc dbwDbc_;

  // Test stuff
  rclcpp::Publisher<RelayCommand>::SharedPtr pdu1_relay_pub_;
  uint32_t count_;
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
