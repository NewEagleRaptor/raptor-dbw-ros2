/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2019 New Eagle 
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef _DBW_NODE_H_
#define _DBW_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include "raptor_dbw_can/dispatch.hpp"
#include  <math.h>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/brake_report.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/gear_cmd.hpp>
#include <raptor_dbw_msgs/msg/gear_report.hpp>
#include <raptor_dbw_msgs/msg/misc_cmd.hpp>
#include <raptor_dbw_msgs/msg/misc_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_position_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/tire_pressure_report.hpp>
#include <raptor_dbw_msgs/msg/surround_report.hpp>
#include <raptor_dbw_msgs/msg/driver_input_report.hpp>
#include <raptor_dbw_msgs/msg/low_voltage_system_report.hpp>
#include <raptor_dbw_msgs/msg/actuator_control_mode.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <raptor_dbw_msgs/msg/steering2_report.hpp>
#include <raptor_dbw_msgs/msg/global_enable_cmd.hpp>
#include <raptor_dbw_msgs/msg/fault_actions_report.hpp>
#include <raptor_dbw_msgs/msg/hmi_global_enable_report.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

//#include <can_dbc_parser/DbcUtilities.hpp>
#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>

#include <pdu_msgs/msg/relay_command.hpp>
#include <pdu_msgs/msg/relay_state.hpp>

using namespace std::chrono_literals;

namespace raptor_dbw_can
{
  class DbwNode : public rclcpp::Node
  {
    public:
      DbwNode(const rclcpp::NodeOptions & options);
      ~DbwNode();

    private:
      void timerCallback();
      void recvEnable(const std_msgs::msg::Empty::SharedPtr msg);
      void recvDisable(const std_msgs::msg::Empty::SharedPtr msg);
      void recvCAN(const can_msgs::msg::Frame::SharedPtr msg);
      void recvCanImu(const std::vector<can_msgs::msg::Frame> msgs);
      void recvCanGps(const std::vector<can_msgs::msg::Frame> msgs);
      void recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg);
      void recvAcceleratorPedalCmd(const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg);
      void recvSteeringCmd(const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg);
      void recvGearCmd(const raptor_dbw_msgs::msg::GearCmd::SharedPtr msg);
      void recvMiscCmd(const raptor_dbw_msgs::msg::MiscCmd::SharedPtr msg);
      void recvGlobalEnableCmd(const raptor_dbw_msgs::msg::GlobalEnableCmd::SharedPtr msg);

      rclcpp::TimerBase::SharedPtr timer_;
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
      inline bool fault() { return fault_brakes_ || fault_accelerator_pedal_ || fault_steering_ || fault_steering_cal_ || fault_watchdog_; }
      inline bool override() { return override_brake_ || override_accelerator_pedal_ || override_steering_ || override_gear_; }
      inline bool clear() { return enable_ && override(); }
      inline bool enabled() { return enable_ && !fault() && !override(); }
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

      enum {
        JOINT_FL = 0, // Front left wheel
        JOINT_FR, // Front right wheel
        JOINT_RL, // Rear left wheel
        JOINT_RR, // Rear right wheel
        JOINT_SL, // Steering left
        JOINT_SR, // Steering right
        JOINT_COUNT, // Number of joints
      };

      sensor_msgs::msg::JointState joint_state_;
      
      void publishJointStates(const rclcpp::Time stamp, const raptor_dbw_msgs::msg::SteeringReport steering);
      void publishJointStates(const rclcpp::Time stamp, const raptor_dbw_msgs::msg::WheelSpeedReport wheels);

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
      rclcpp::Subscription<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr sub_brake_;
      rclcpp::Subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>::SharedPtr sub_accelerator_pedal_;
      rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr sub_steering_;
      rclcpp::Subscription<raptor_dbw_msgs::msg::GearCmd>::SharedPtr sub_gear_;
      rclcpp::Subscription<raptor_dbw_msgs::msg::MiscCmd>::SharedPtr sub_misc_;
      rclcpp::Subscription<raptor_dbw_msgs::msg::GlobalEnableCmd>::SharedPtr sub_global_enable_;

      // Published topics
      rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::BrakeReport>::SharedPtr pub_brake_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>::SharedPtr pub_accel_pedal_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr pub_steering_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::GearReport>::SharedPtr pub_gear_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::MiscReport>::SharedPtr pub_misc_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr pub_wheel_speeds_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::WheelPositionReport>::SharedPtr pub_wheel_positions_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::TirePressureReport>::SharedPtr pub_tire_pressure_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::SurroundReport>::SharedPtr pub_surround_;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_vin_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sys_enable_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::DriverInputReport>::SharedPtr pub_driver_input_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::LowVoltageSystemReport>::SharedPtr pub_low_voltage_system_;

      rclcpp::Publisher<raptor_dbw_msgs::msg::Brake2Report>::SharedPtr pub_brake_2_report_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::Steering2Report>::SharedPtr pub_steering_2_report_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::FaultActionsReport>::SharedPtr pub_fault_actions_report_;
      rclcpp::Publisher<raptor_dbw_msgs::msg::HmiGlobalEnableReport>::SharedPtr pub_hmi_global_enable_report_;

      NewEagle::Dbc dbwDbc_;
      std::string dbcFile_;

      // Test stuff
      rclcpp::Publisher<pdu_msgs::msg::RelayCommand>::SharedPtr pdu1_relay_pub_;
      uint32_t count_;
  };

} // raptor_dbw_can

#endif // _DBW_NODE_H_

