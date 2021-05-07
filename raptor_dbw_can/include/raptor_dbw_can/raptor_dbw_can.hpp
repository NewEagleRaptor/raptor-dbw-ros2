// Copyright (c) 2021 New Eagle, All rights reserved.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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

#include <raptor_can_dbc_parser/DbcMessage.hpp>
#include <raptor_can_dbc_parser/DbcSignal.hpp>
#include <raptor_can_dbc_parser/Dbc.hpp>
#include <raptor_can_dbc_parser/DbcBuilder.hpp>

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
using raptor_dbw_msgs::msg::DoorLock;
using raptor_dbw_msgs::msg::DoorRequest;
using raptor_dbw_msgs::msg::DoorState;
using raptor_dbw_msgs::msg::DriverInputReport;
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
class RaptorDbwCAN : public rclcpp::Node
{
public:
  explicit RaptorDbwCAN(
    const rclcpp::NodeOptions & options,
    std::string dbw_dbc_file,
    float max_steer_angle);
  ~RaptorDbwCAN();

private:
  void timerCallback();
  void recvEnable(const Empty::SharedPtr msg);
  void recvDisable(const Empty::SharedPtr msg);
  void recvCAN(const Frame::SharedPtr msg);
  void recvAccelPedalRpt(const Frame::SharedPtr msg);
  void recvBrakeRpt(const Frame::SharedPtr msg);
  void recvBrake2Rpt(const Frame::SharedPtr msg);
  void recvDriverInputRpt(const Frame::SharedPtr msg);
  void recvFaultActionRpt(const Frame::SharedPtr msg);
  void recvGearRpt(const Frame::SharedPtr msg);
  void recvGpsReferenceRpt(const Frame::SharedPtr msg);
  void recvGpsRemainderRpt(const Frame::SharedPtr msg);
  void recvImuRpt(const Frame::SharedPtr msg);
  void recvLowVoltageSystemRpt(const Frame::SharedPtr msg);
  void recvMiscRpt(const Frame::SharedPtr msg);
  void recvOtherActuatorsRpt(const Frame::SharedPtr msg);
  void recvSteeringRpt(const Frame::SharedPtr msg);
  void recvSteering2Rpt(const Frame::SharedPtr msg);
  void recvSurroundRpt(const Frame::SharedPtr msg);
  void recvVinRpt(const Frame::SharedPtr msg);
  void recvTirePressureRpt(const Frame::SharedPtr msg);
  void recvWheelPositionRpt(const Frame::SharedPtr msg);
  void recvWheelSpeedRpt(const Frame::SharedPtr msg);
  void recvCanImu(const std::vector<Frame> msgs);
  void recvCanGps(const std::vector<Frame> msgs);
  void recvAcceleratorPedalCmd(const AcceleratorPedalCmd::SharedPtr msg);
  void recvBrakeCmd(const BrakeCmd::SharedPtr msg);
  void recvGearCmd(const GearCmd::SharedPtr msg);
  void recvGlobalEnableCmd(const GlobalEnableCmd::SharedPtr msg);
  void recvMiscCmd(const MiscCmd::SharedPtr msg);
  void recvSteeringCmd(const SteeringCmd::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock m_clock;
  static constexpr int64_t CLOCK_1_SEC = 1000;  // duration in milliseconds

  // Parameters from launch
  std::string dbw_dbc_file_;
  float max_steer_angle_;

  // Other useful variables
  enum ListOverrides
  {
    OVR_ACCEL = 0,
    OVR_BRAKE,
    OVR_STEER,
    OVR_GEAR,
    NUM_OVERRIDES
  };
  enum ListFaults
  {
    FAULT_ACCEL = 0,
    FAULT_BRAKE,
    FAULT_STEER,
    FAULT_STEER_CAL,
    FAULT_WATCH,
    FAULT_WATCH_BRAKES,
    FAULT_WATCH_WARN,
    NUM_FAULTS
  };
  enum ListTimeouts
  {
    TO_ACCEL = 0,
    TO_BRAKE,
    TO_STEER,
    NUM_TIMEOUTS
  };
  enum ListEnables
  {
    EN_ACCEL = 0,
    EN_BRAKE,
    EN_STEER,
    EN_DBW,
    EN_DBW_PREV,
    NUM_ENABLES
  };

  /** \brief Convert ListTimeouts enum type to ListEnables enum type
   * \param[in] in_to ListTimeouts enum value to convert
   * \returns equivalent ListEnables value, or NUM_ENABLES on invalid input
   */
  ListEnables convEnable(ListTimeouts in_to);

  const std::string OVR_SYSTEM[NUM_OVERRIDES] = {
    "accelerator pedal",
    "brake",
    "PRND gear",
    "steering"
  };
  const std::string FAULT_SYSTEM[NUM_FAULTS] = {
    "accelerator pedal",
    "brake",
    "steering",
    "steering calibration",
    "watchdog",
    "",
    ""
  };
  const std::string TO_SYSTEM[NUM_TIMEOUTS] = {
    "Accelerator Pedal",
    "Brake",
    "Steering"
  };

  bool overrides_[NUM_OVERRIDES];
  bool faults_[NUM_FAULTS];
  bool timeouts_[NUM_TIMEOUTS];
  bool enables_[NUM_ENABLES];

  inline bool fault()
  {
    return faults_[FAULT_BRAKE] || faults_[FAULT_ACCEL] || faults_[FAULT_STEER] ||
           faults_[FAULT_STEER_CAL] || faults_[FAULT_WATCH];
  }
  inline bool override () {return overrides_[OVR_BRAKE] || overrides_[OVR_ACCEL] ||
           overrides_[OVR_STEER] || overrides_[OVR_GEAR];}
  inline bool clear() {return enables_[EN_DBW] && override ();}
  inline bool enabled() {return enables_[EN_DBW] && !fault() && !override ();}

/** \brief DBW Enabled needs to publish when its state changes.
 * \returns TRUE when DBW enable state changes, FALSE otherwise
 */
  bool publishDbwEnabled();
  void enableSystem();
  void disableSystem();
  void buttonCancel();

  /** \brief Set the specified override
   * \param[in] which_ovr Which override to set
   * \param[in] override The value to set the override to
   */
  void setOverride(ListOverrides which_ovr, bool override);

  /** \brief Set the specified timeout
   * \param[in] which_to Which timeout to set
   * \param[in] timeout The value to set the timeout to
   * \param[in] enabled Whether to enable/disable the system
   */
  void setTimeout(ListTimeouts which_to, bool timeout, bool enabled);

  /** \brief Set the specified fault
   * \param[in] which_fault Which fault to set
   * \param[in] fault The value to set the fault to
   */
  void setFault(ListFaults which_fault, bool fault);
  void faultWatchdog(bool fault, uint8_t src, bool braking);
  void faultWatchdog(bool fault, uint8_t src = 0);

  enum
  {
    JOINT_FL = 0,     // Front left wheel
    JOINT_FR,     // Front right wheel
    JOINT_RL,     // Rear left wheel
    JOINT_RR,     // Rear right wheel
    JOINT_SL,     // Steering left
    JOINT_SR,     // Steering right
    JOINT_COUNT,     // Number of joints
  };

  JointState joint_state_;

  void publishJointStates(
    const rclcpp::Time stamp,
    const SteeringReport steering);
  void publishJointStates(
    const rclcpp::Time stamp,
    const WheelSpeedReport wheels);

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
  rclcpp::Subscription<MiscCmd>::SharedPtr sub_misc_;
  rclcpp::Subscription<SteeringCmd>::SharedPtr sub_steering_;

  // Published topics
  rclcpp::Publisher<Bool>::SharedPtr pub_sys_enable_;
  rclcpp::Publisher<Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<AcceleratorPedalReport>::SharedPtr pub_accel_pedal_;
  rclcpp::Publisher<BrakeReport>::SharedPtr pub_brake_;
  rclcpp::Publisher<Brake2Report>::SharedPtr pub_brake_2_report_;
  rclcpp::Publisher<DriverInputReport>::SharedPtr pub_driver_input_;
  rclcpp::Publisher<FaultActionsReport>::SharedPtr pub_fault_actions_report_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<GpsReferenceReport>::SharedPtr pub_gps_reference_report_;
  rclcpp::Publisher<GpsRemainderReport>::SharedPtr pub_gps_remainder_report_;
  rclcpp::Publisher<HmiGlobalEnableReport>::SharedPtr pub_hmi_global_enable_report_;
  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<LowVoltageSystemReport>::SharedPtr pub_low_voltage_system_;
  rclcpp::Publisher<MiscReport>::SharedPtr pub_misc_;
  rclcpp::Publisher<OtherActuatorsReport>::SharedPtr pub_other_actuators_report_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<Steering2Report>::SharedPtr pub_steering_2_report_;
  rclcpp::Publisher<SurroundReport>::SharedPtr pub_surround_;
  rclcpp::Publisher<TirePressureReport>::SharedPtr pub_tire_pressure_;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_;
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
