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

/** \brief This file defines the raptor_pdu class.
 * \copyright Copyright 2021 New Eagle LLC
 * \file raptor_pdu.hpp
 */

#ifndef RAPTOR_PDU__RAPTOR_PDU_HPP_
#define RAPTOR_PDU__RAPTOR_PDU_HPP_

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <raptor_pdu_msgs/msg/fuse_report.hpp>
#include <raptor_pdu_msgs/msg/relay_command.hpp>
#include <raptor_pdu_msgs/msg/relay_report.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>
#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/LineParser.hpp>

#include <string>

using can_msgs::msg::Frame;
using raptor_pdu_msgs::msg::FuseReport;
using raptor_pdu_msgs::msg::RelayCommand;
using raptor_pdu_msgs::msg::RelayReport;

namespace NewEagle
{
/** \brief Class for interacting with the PDU */
class raptor_pdu : public rclcpp::Node
{
  /** \brief Enumeration of message base addresses */
  enum ListAddresses
  {
    RELAY_STATUS_BASE_ADDR = 0x18ffa100,  /**< Relay status message base address */
    FUSE_STATUS_BASE_ADDR = 0x18ffa000,   /**< Fuse status message base address */
    RELAY_COMMAND_BASE_ADDR = 0x18ef0000  /**< Relay command message base address */
  };

public:
/** \brief Default constructor.
 * \param[in] options The options for this node.
 */
  explicit raptor_pdu(const rclcpp::NodeOptions & options);

private:
  uint32_t id_;
  uint32_t relayCommandAddr_;
  uint32_t relayStatusAddr_;
  uint32_t fuseStatusAddr_;

  uint32_t count_;

  rclcpp::Clock m_clock;
  static constexpr int64_t CLOCK_1_SEC = 1000;  // duration in milliseconds

  NewEagle::Dbc pduDbc_;
  std::string pduFile_;

/** \brief Convert reports received over CAN into ROS messages.
 * \param[in] msg The message received over CAN.
 */
  void recvCAN(const Frame::SharedPtr msg);

/** \brief Send out a Relay Command over CAN
 * \param[in] msg The RelayCommand received as a ROS message.
 */
  void recvRelayCmd(const RelayCommand::SharedPtr msg);

  // Subscribed topics
  rclcpp::Subscription<Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<RelayCommand>::SharedPtr sub_relay_cmd_;

  // Published topics
  rclcpp::Publisher<Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<FuseReport>::SharedPtr fuse_report_pub_;
  rclcpp::Publisher<RelayReport>::SharedPtr relay_report_pub_;
};
}  // namespace NewEagle

#endif  // RAPTOR_PDU__RAPTOR_PDU_HPP_
