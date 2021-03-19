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
  explicit RaptorDbwCAN(const rclcpp::NodeOptions & options);
  ~RaptorDbwCAN();

private:
  void timerCallback();
  void recvEnable(const std_msgs::msg::Empty::SharedPtr msg);
  void recvDisable(const std_msgs::msg::Empty::SharedPtr msg);
  void recvCAN(const can_msgs::msg::Frame::SharedPtr msg);
  void recvBrakeRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvAccelPedalRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvSteeringRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvGearRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvCWheelSpeedRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvWheelPositionRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvTirePressureRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvSurroundRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvVinRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvImuRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvDriverInputRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvMiscRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvLowVoltageSystemRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvBrake2Rpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvSteering2Rpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvFaultActionRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvOtherActuatorsRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvGpsReferenceRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvGpsRemainderRpt(const can_msgs::msg::Frame::SharedPtr msg);
  void recvCanImu(const std::vector<can_msgs::msg::Frame> msgs);
  void recvCanGps(const std::vector<can_msgs::msg::Frame> msgs);
  void recvBrakeCmd(const BrakeCmd::SharedPtr msg);
  void recvAcceleratorPedalCmd(const AcceleratorPedalCmd::SharedPtr msg);
  void recvSteeringCmd(const SteeringCmd::SharedPtr msg);
  void recvGearCmd(const GearCmd::SharedPtr msg);
  void recvMiscCmd(const MiscCmd::SharedPtr msg);
  void recvGlobalEnableCmd(const GlobalEnableCmd::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock m_clock;
  static constexpr int64_t CLOCK_1_SEC = 1000;  // duration in milliseconds

  bool prev_enable_;
  bool enable_;
  bool override_brake_;
  bool override_accelerator_pedal_;
  bool override_steering_;
  bool override_gear_;
  bool fault_brakes_;
  bool fault_accelerator_pedal_;
  bool fault_steering_;
  bool fault_steering_cal_;
  bool fault_watchdog_;
  bool fault_watchdog_using_brakes_;
  bool fault_watchdog_warned_;
  bool timeout_brakes_;
  bool timeout_accelerator_pedal_;
  bool timeout_steering_;
  bool enabled_brakes_;
  bool enabled_accelerator_pedal_;
  bool enabled_steering_;
  bool gear_warned_;
  inline bool fault()
  {
    return fault_brakes_ || fault_accelerator_pedal_ || fault_steering_ || fault_steering_cal_ ||
           fault_watchdog_;
  }
  inline bool override () {return override_brake_ || override_accelerator_pedal_ ||
           override_steering_ || override_gear_;}
  inline bool clear() {return enable_ && override ();}
  inline bool enabled() {return enable_ && !fault() && !override ();}
  bool publishDbwEnabled();
  void enableSystem();
  void disableSystem();
  void buttonCancel();
  void overrideBrake(bool override);
  void overrideAcceleratorPedal(bool override);
  void overrideSteering(bool override);
  void overrideGear(bool override);
  void timeoutBrake(bool timeout, bool enabled);
  void timeoutAcceleratorPedal(bool timeout, bool enabled);
  void timeoutSteering(bool timeout, bool enabled);
  void faultBrakes(bool fault);
  void faultAcceleratorPedal(bool fault);
  void faultSteering(bool fault);
  void faultSteeringCal(bool fault);
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

  sensor_msgs::msg::JointState joint_state_;

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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_enable_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_disable_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<BrakeCmd>::SharedPtr sub_brake_;
  rclcpp::Subscription<AcceleratorPedalCmd>::SharedPtr sub_accelerator_pedal_;
  rclcpp::Subscription<SteeringCmd>::SharedPtr sub_steering_;
  rclcpp::Subscription<GearCmd>::SharedPtr sub_gear_;
  rclcpp::Subscription<MiscCmd>::SharedPtr sub_misc_;
  rclcpp::Subscription<GlobalEnableCmd>::SharedPtr sub_global_enable_;

  // Published topics
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<BrakeReport>::SharedPtr pub_brake_;
  rclcpp::Publisher<AcceleratorPedalReport>::SharedPtr pub_accel_pedal_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<MiscReport>::SharedPtr pub_misc_;
  rclcpp::Publisher<WheelSpeedReport>::SharedPtr pub_wheel_speeds_;
  rclcpp::Publisher<WheelPositionReport>::SharedPtr pub_wheel_positions_;
  rclcpp::Publisher<TirePressureReport>::SharedPtr pub_tire_pressure_;
  rclcpp::Publisher<SurroundReport>::SharedPtr pub_surround_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_vin_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sys_enable_;
  rclcpp::Publisher<DriverInputReport>::SharedPtr pub_driver_input_;
  rclcpp::Publisher<LowVoltageSystemReport>::SharedPtr
    pub_low_voltage_system_;

  rclcpp::Publisher<Brake2Report>::SharedPtr pub_brake_2_report_;
  rclcpp::Publisher<Steering2Report>::SharedPtr pub_steering_2_report_;
  rclcpp::Publisher<FaultActionsReport>::SharedPtr pub_fault_actions_report_;
  rclcpp::Publisher<HmiGlobalEnableReport>::SharedPtr
    pub_hmi_global_enable_report_;
  rclcpp::Publisher<OtherActuatorsReport>::SharedPtr
    pub_other_actuators_report_;
  rclcpp::Publisher<GpsReferenceReport>::SharedPtr pub_gps_reference_report_;
  rclcpp::Publisher<GpsRemainderReport>::SharedPtr pub_gps_remainder_report_;

  NewEagle::Dbc dbwDbc_;
  std::string dbcFile_;

  // Test stuff
  rclcpp::Publisher<raptor_pdu_msgs::msg::RelayCommand>::SharedPtr pdu1_relay_pub_;
  uint32_t count_;
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
