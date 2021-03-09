// Copyright (c) 2018-2019 New Eagle, Copyright (c) 2015-2018, Dataspeed Inc.
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

#include "raptor_dbw_joystick/raptor_dbw_joystick.hpp"

#include <cmath>

namespace raptor_dbw_joystick
{

RaptorDbwJoystick::RaptorDbwJoystick(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_joystick_node", options)
{
  ignore_ = false;
  enable_ = true;
  svel_ = 0.0;
  this->declare_parameter("ignore", ignore_);
  this->declare_parameter("enable", enable_);
  this->declare_parameter("svel", svel_);

  data_.brake_joy = 0.0;
  data_.gear_cmd = raptor_dbw_msgs::msg::Gear::NONE;
  data_.steering_joy = 0.0;
  data_.steering_mult = false;
  data_.accelerator_pedal_joy = 0.0;
  data_.turn_signal_cmd = raptor_dbw_msgs::msg::TurnSignal::NONE;
  data_.joy_accelerator_pedal_valid = false;
  data_.joy_brake_valid = false;

  joy_.axes.resize(AXIS_COUNT, 0);
  joy_.buttons.resize(BTN_COUNT, 0);

  sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 600, std::bind(&RaptorDbwJoystick::recvJoy, this, std::placeholders::_1));

  pub_accelerator_pedal_ = this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>(
    "accelerator_pedal_cmd", 1);
  pub_brake_ = this->create_publisher<raptor_dbw_msgs::msg::BrakeCmd>("brake_cmd", 1);
  pub_misc_ = this->create_publisher<raptor_dbw_msgs::msg::MiscCmd>("misc_cmd", 1);
  pub_steering_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringCmd>("steering_cmd", 1);
  pub_global_enable_ = this->create_publisher<raptor_dbw_msgs::msg::GlobalEnableCmd>(
    "global_enable_cmd", 1);
  pub_gear_ = this->create_publisher<raptor_dbw_msgs::msg::GearCmd>("gear_cmd", 1);
  if (enable_) {
    pub_enable_ = this->create_publisher<std_msgs::msg::Empty>("enable", 1);
    pub_disable_ = this->create_publisher<std_msgs::msg::Empty>("disable", 1);
  }

  timer_ = this->create_wall_timer(200ms, std::bind(&RaptorDbwJoystick::cmdCallback, this));
}

void RaptorDbwJoystick::cmdCallback()
{
  // Detect joy timeouts and reset
  double message_timeout_sec = 0.1;
  std::chrono::steady_clock::duration dt = std::chrono::steady_clock::now() - data_.stamp;
  double seconds_passed = static_cast<double>(dt.count()) *
    std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;

  if (seconds_passed > message_timeout_sec) {
    data_.joy_accelerator_pedal_valid = false;
    data_.joy_brake_valid = false;
    return;
  }

  // watchdog counter
  counter_++;
  if (counter_ > 15) {
    counter_ = 0;
  }

  // Accelerator Pedal
  raptor_dbw_msgs::msg::AcceleratorPedalCmd accelerator_pedal_msg;
  accelerator_pedal_msg.enable = true;
  accelerator_pedal_msg.ignore = ignore_;
  accelerator_pedal_msg.rolling_counter = counter_;
  accelerator_pedal_msg.pedal_cmd = data_.accelerator_pedal_joy * 100;
  accelerator_pedal_msg.control_type.value = raptor_dbw_msgs::msg::ActuatorControlMode::OPEN_LOOP;
  pub_accelerator_pedal_->publish(accelerator_pedal_msg);

  // Brake
  raptor_dbw_msgs::msg::BrakeCmd brake_msg;
  brake_msg.enable = true;
  brake_msg.rolling_counter = counter_;
  brake_msg.pedal_cmd = data_.brake_joy * 100;
  brake_msg.control_type.value = raptor_dbw_msgs::msg::ActuatorControlMode::OPEN_LOOP;
  pub_brake_->publish(brake_msg);

  // Steering
  raptor_dbw_msgs::msg::SteeringCmd steering_msg;
  steering_msg.enable = true;
  steering_msg.ignore = ignore_;
  steering_msg.rolling_counter = counter_;
  steering_msg.angle_cmd = data_.steering_joy;
  steering_msg.angle_velocity = svel_;

  steering_msg.control_type.value = raptor_dbw_msgs::msg::ActuatorControlMode::CLOSED_LOOP_ACTUATOR;
  if (!data_.steering_mult) {
    steering_msg.angle_cmd *= 0.5;
  }
  pub_steering_->publish(steering_msg);

  // Gear
  raptor_dbw_msgs::msg::GearCmd gear_msg;
  gear_msg.cmd.gear = data_.gear_cmd;
  gear_msg.enable = true;
  gear_msg.rolling_counter = counter_;
  pub_gear_->publish(gear_msg);

  // Turn signal
  raptor_dbw_msgs::msg::MiscCmd misc_msg;
  misc_msg.cmd.value = data_.turn_signal_cmd;
  misc_msg.rolling_counter = counter_;
  pub_misc_->publish(misc_msg);

  raptor_dbw_msgs::msg::GlobalEnableCmd globalEnable_msg;
  globalEnable_msg.global_enable = true;
  globalEnable_msg.enable_joystick_limits = true;
  globalEnable_msg.rolling_counter = counter_;
  pub_global_enable_->publish(globalEnable_msg);
}

void RaptorDbwJoystick::recvJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Check for expected sizes
  if (msg->axes.size() != (size_t)AXIS_COUNT) {
    RCLCPP_ERROR(this->get_logger(), "Axis count is wrong.");
  }
  if (msg->buttons.size() != (size_t)BTN_COUNT) {
    RCLCPP_ERROR(this->get_logger(), "Button count is wrong");
  }

  // Handle joystick startup
  if (msg->axes[AXIS_ACCELERATOR_PEDAL] != 0.0) {
    data_.joy_accelerator_pedal_valid = true;
  }
  if (msg->axes[AXIS_BRAKE] != 0.0) {
    data_.joy_brake_valid = true;
  }

  // Accelerator pedal
  if (data_.joy_accelerator_pedal_valid) {
    data_.accelerator_pedal_joy = 0.5 - 0.5 * msg->axes[AXIS_ACCELERATOR_PEDAL];
  }

  // Brake
  if (data_.joy_brake_valid) {
    data_.brake_joy = 0.5 - 0.5 * msg->axes[AXIS_BRAKE];
  }

  // Gear
  if (msg->buttons[BTN_PARK]) {
    data_.gear_cmd = raptor_dbw_msgs::msg::Gear::PARK;
  } else if (msg->buttons[BTN_REVERSE]) {
    data_.gear_cmd = raptor_dbw_msgs::msg::Gear::REVERSE;
  } else if (msg->buttons[BTN_DRIVE]) {
    data_.gear_cmd = raptor_dbw_msgs::msg::Gear::DRIVE;
  } else if (msg->buttons[BTN_NEUTRAL]) {
    data_.gear_cmd = raptor_dbw_msgs::msg::Gear::NEUTRAL;
  } else {
    data_.gear_cmd = raptor_dbw_msgs::msg::Gear::NONE;
  }

  // Steering
  data_.steering_joy = 470.0 * M_PI / 180.0 *
    ((fabs(msg->axes[AXIS_STEER_1]) >
    fabs(msg->axes[AXIS_STEER_2])) ? msg->axes[AXIS_STEER_1] : msg->axes[AXIS_STEER_2]);
  data_.steering_mult = msg->buttons[BTN_STEER_MULT_1] || msg->buttons[BTN_STEER_MULT_2];

  // Turn signal
  if (msg->axes[AXIS_TURN_SIG] != joy_.axes[AXIS_TURN_SIG]) {
    switch (data_.turn_signal_cmd) {
      case raptor_dbw_msgs::msg::TurnSignal::NONE:
        if (msg->axes[AXIS_TURN_SIG] < -0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::msg::TurnSignal::RIGHT;
        } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::msg::TurnSignal::LEFT;
        }
        break;
      case raptor_dbw_msgs::msg::TurnSignal::LEFT:
        if (msg->axes[AXIS_TURN_SIG] < -0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::msg::TurnSignal::RIGHT;
        } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::msg::TurnSignal::NONE;
        }
        break;
      case raptor_dbw_msgs::msg::TurnSignal::RIGHT:
        if (msg->axes[AXIS_TURN_SIG] < -0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::msg::TurnSignal::NONE;
        } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
          data_.turn_signal_cmd = raptor_dbw_msgs::msg::TurnSignal::LEFT;
        }
        break;
    }
  }

  // Optional enable and disable buttons
  if (enable_) {
    const std_msgs::msg::Empty empty;
    if (msg->buttons[BTN_ENABLE]) {
      pub_enable_->publish(empty);
    }
    if (msg->buttons[BTN_DISABLE]) {
      pub_disable_->publish(empty);
    }
  }

  data_.stamp = std::chrono::steady_clock::now();
  joy_ = *msg;
}

}  // namespace raptor_dbw_joystick
