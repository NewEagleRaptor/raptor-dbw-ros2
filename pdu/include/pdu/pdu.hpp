/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 New Eagle
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
 *   * Neither the name of New Eagle nor the names of its
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

#ifndef NEWEAGLE_PDU_H_
#define NEWEAGLE_PDU_H_

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <pdu_msgs/msg/fuse_report.hpp>
#include <pdu_msgs/msg/relay_report.hpp>
#include <pdu_msgs/msg/relay_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <can_msgs/msg/frame.hpp>

#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/LineParser.hpp>


namespace NewEagle
{
class pdu : public rclcpp::Node
{
  enum
  {
    RELAY_STATUS_BASE_ADDR = 0x18ffa100,
    FUSE_STATUS_BASE_ADDR = 0x18ffa000,
    RELAY_COMMAND_BASE_ADDR = 0x18ef0000
  };

public:
  pdu(const rclcpp::NodeOptions & options);

private:
  uint32_t id_;
  uint32_t relayCommandAddr_;
  uint32_t relayStatusAddr_;
  uint32_t fuseStatusAddr_;

  uint32_t count_;

  NewEagle::Dbc pduDbc_;
  std::string pduFile_;

  void recvCAN(const can_msgs::msg::Frame::SharedPtr msg);
  void recvRelayCmd(const pdu_msgs::msg::RelayCommand::SharedPtr msg);

  // Subscribed topics
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<pdu_msgs::msg::RelayCommand>::SharedPtr sub_relay_cmd_;

  // Published topics
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<pdu_msgs::msg::FuseReport>::SharedPtr fuse_report_pub_;
  rclcpp::Publisher<pdu_msgs::msg::RelayReport>::SharedPtr relay_report_pub_;
};
}

#endif /* NEWEAGLE_PDU_H_ */
