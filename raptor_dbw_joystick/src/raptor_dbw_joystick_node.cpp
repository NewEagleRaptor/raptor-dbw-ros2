// Copyright (c) 2018-2021 New Eagle, Copyright (c) 2015-2018, Dataspeed Inc.
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

#include "raptor_dbw_joystick/raptor_dbw_joystick.hpp"

#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  rclcpp::executors::SingleThreadedExecutor exec{};

  // Get parameter values
  auto temp = std::make_shared<rclcpp::Node>("get_joy_params_node", options);
  temp->declare_parameter("ignore", rclcpp::PARAMETER_BOOL);
  temp->declare_parameter("enable", rclcpp::PARAMETER_BOOL);
  temp->declare_parameter("svel", rclcpp::PARAMETER_DOUBLE);
  temp->declare_parameter("max_steer_angle", rclcpp::PARAMETER_DOUBLE);

  bool n_ignore = temp->get_parameter("ignore").as_bool();
  bool n_enable = temp->get_parameter("enable").as_bool();
  double n_svel = temp->get_parameter("svel").as_double();
  float n_max_steer_angle = temp->get_parameter("max_steer_angle").as_double();

  // Create RaptorDbwJoystick class
  auto node = std::make_shared<raptor_dbw_joystick::RaptorDbwJoystick>(
    options,
    n_ignore,
    n_enable,
    n_svel,
    n_max_steer_angle
  );

  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
