// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include "raptor_dbw_can/raptor_dbw_can.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace raptor_dbw_can
{

RaptorDbwCAN::RaptorDbwCAN(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_can_node", options)
{
  dbcFile_ = this->declare_parameter("dbw_dbc_file", "");

  // Initialize enable state machine
  prev_enable_ = true;
  enable_ = false;
  override_brake_ = false;
  override_accelerator_pedal_ = false;
  override_steering_ = false;
  override_gear_ = false;
  fault_brakes_ = false;
  fault_accelerator_pedal_ = false;
  fault_steering_ = false;
  fault_steering_cal_ = false;
  fault_watchdog_ = false;
  fault_watchdog_using_brakes_ = false;
  fault_watchdog_warned_ = false;
  timeout_brakes_ = false;
  timeout_accelerator_pedal_ = false;
  timeout_steering_ = false;
  enabled_brakes_ = false;
  enabled_accelerator_pedal_ = false;
  enabled_steering_ = false;
  gear_warned_ = false;

  // Frame ID
  frame_id_ = "base_footprint";
  this->declare_parameter<std::string>("frame_id", frame_id_);

  // Buttons (enable/disable)
  buttons_ = true;
  this->declare_parameter<bool>("buttons", buttons_);


  // Ackermann steering parameters
  acker_wheelbase_ = 2.8498;   // 112.2 inches
  acker_track_ = 1.5824;   // 62.3 inches
  steering_ratio_ = 14.8;
  this->declare_parameter<double>("ackermann_wheelbase", acker_wheelbase_);
  this->declare_parameter<double>("ackermann_track", acker_track_);
  this->declare_parameter<double>("steering_ratio", steering_ratio_);

  // Initialize joint states
  joint_state_.position.resize(JOINT_COUNT);
  joint_state_.velocity.resize(JOINT_COUNT);
  joint_state_.effort.resize(JOINT_COUNT);
  joint_state_.name.resize(JOINT_COUNT);
  joint_state_.name[JOINT_FL] = "wheel_fl";   // Front Left
  joint_state_.name[JOINT_FR] = "wheel_fr";   // Front Right
  joint_state_.name[JOINT_RL] = "wheel_rl";   // Rear Left
  joint_state_.name[JOINT_RR] = "wheel_rr";   // Rear Right
  joint_state_.name[JOINT_SL] = "steer_fl";
  joint_state_.name[JOINT_SR] = "steer_fr";

  // Set up Publishers
  pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 20);
  pub_brake_ = this->create_publisher<BrakeReport>("brake_report", 20);
  pub_accel_pedal_ = this->create_publisher<AcceleratorPedalReport>(
    "accelerator_pedal_report", 20);
  pub_steering_ =
    this->create_publisher<SteeringReport>("steering_report", 20);
  pub_gear_ = this->create_publisher<GearReport>("gear_report", 20);
  pub_wheel_speeds_ = this->create_publisher<WheelSpeedReport>(
    "wheel_speed_report", 20);
  pub_wheel_positions_ = this->create_publisher<WheelPositionReport>(
    "wheel_position_report", 20);
  pub_tire_pressure_ = this->create_publisher<TirePressureReport>(
    "tire_pressure_report", 20);
  pub_surround_ =
    this->create_publisher<SurroundReport>("surround_report", 20);

  pub_low_voltage_system_ = this->create_publisher<LowVoltageSystemReport>(
    "low_voltage_system_report", 2);

  pub_brake_2_report_ = this->create_publisher<Brake2Report>(
    "brake_2_report",
    20);
  pub_steering_2_report_ = this->create_publisher<Steering2Report>(
    "steering_2_report", 20);
  pub_fault_actions_report_ = this->create_publisher<FaultActionsReport>(
    "fault_actions_report", 20);
  pub_hmi_global_enable_report_ =
    this->create_publisher<HmiGlobalEnableReport>(
    "hmi_global_enable_report", 20);
  pub_other_actuators_report_ = this->create_publisher<OtherActuatorsReport>(
    "other_actuators_report", 20);
  pub_gps_reference_report_ = this->create_publisher<GpsReferenceReport>(
    "gps_reference_report", 20);
  pub_gps_remainder_report_ = this->create_publisher<GpsRemainderReport>(
    "gps_remainder_report", 20);

  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 10);
  pub_vin_ = this->create_publisher<std_msgs::msg::String>("vin", 1);
  pub_driver_input_ = this->create_publisher<DriverInputReport>(
    "driver_input_report", 2);
  pub_misc_ = this->create_publisher<MiscReport>("misc_report", 2);
  pub_sys_enable_ = this->create_publisher<std_msgs::msg::Bool>("dbw_enabled", 1);
  publishDbwEnabled();

  // Set up Subscribers
  sub_enable_ = this->create_subscription<std_msgs::msg::Empty>(
    "enable", 10, std::bind(&RaptorDbwCAN::recvEnable, this, std::placeholders::_1));

  sub_disable_ = this->create_subscription<std_msgs::msg::Empty>(
    "disable", 10, std::bind(&RaptorDbwCAN::recvDisable, this, std::placeholders::_1));

  sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_tx", 500, std::bind(&RaptorDbwCAN::recvCAN, this, std::placeholders::_1));

  sub_brake_ = this->create_subscription<BrakeCmd>(
    "brake_cmd", 1, std::bind(&RaptorDbwCAN::recvBrakeCmd, this, std::placeholders::_1));

  sub_accelerator_pedal_ = this->create_subscription<AcceleratorPedalCmd>(
    "accelerator_pedal_cmd", 1,
    std::bind(&RaptorDbwCAN::recvAcceleratorPedalCmd, this, std::placeholders::_1));

  sub_steering_ = this->create_subscription<SteeringCmd>(
    "steering_cmd", 1, std::bind(&RaptorDbwCAN::recvSteeringCmd, this, std::placeholders::_1));

  sub_gear_ = this->create_subscription<GearCmd>(
    "gear_cmd", 1, std::bind(&RaptorDbwCAN::recvGearCmd, this, std::placeholders::_1));

  sub_misc_ = this->create_subscription<MiscCmd>(
    "misc_cmd", 1, std::bind(&RaptorDbwCAN::recvMiscCmd, this, std::placeholders::_1));

  sub_global_enable_ = this->create_subscription<GlobalEnableCmd>(
    "global_enable_cmd", 1,
    std::bind(&RaptorDbwCAN::recvGlobalEnableCmd, this, std::placeholders::_1));

  pdu1_relay_pub_ = this->create_publisher<raptor_pdu_msgs::msg::RelayCommand>("/pduB/relay_cmd",
      1000);
  count_ = 0;

  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFile_);

  // Set up Timer
  timer_ = this->create_wall_timer(
    200ms, std::bind(&RaptorDbwCAN::timerCallback, this));
}

RaptorDbwCAN::~RaptorDbwCAN()
{
}

void RaptorDbwCAN::recvEnable(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (msg != NULL) {
    enableSystem();
  }
}

void RaptorDbwCAN::recvDisable(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (msg != NULL) {
    disableSystem();
  }
}

void RaptorDbwCAN::recvCAN(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {
      case ID_BRAKE_REPORT:
        recvBrakeRpt(msg);
        break;

      case ID_ACCEL_PEDAL_REPORT:
        recvAccelPedalRpt(msg);
        break;

      case ID_STEERING_REPORT:
        recvSteeringRpt(msg);
        break;

      case ID_GEAR_REPORT:
        recvGearRpt(msg);
        break;

      case ID_REPORT_WHEEL_SPEED:
        recvWheelSpeedRpt(msg);
        break;

      case ID_REPORT_WHEEL_POSITION:
        recvWheelPositionRpt(msg);
        break;

      case ID_REPORT_TIRE_PRESSURE:
        recvTirePressureRpt(msg);
        break;

      case ID_REPORT_SURROUND:
        recvSurroundRpt(msg);
        break;

      case ID_VIN:
        recvVinRpt(msg);
        break;

      case ID_REPORT_IMU:
        recvImuRpt(msg);
        break;

      case ID_REPORT_DRIVER_INPUT:
        recvDriverInputRpt(msg);
        break;

      case ID_MISC_REPORT:
        recvMiscRpt(msg);
        break;

      case ID_LOW_VOLTAGE_SYSTEM_REPORT:
        recvLowVoltageSystemRpt(msg);
        break;

      case ID_BRAKE_2_REPORT:
        recvBrake2Rpt(msg);
        break;

      case ID_STEERING_2_REPORT:
        recvSteering2Rpt(msg);
        break;

      case ID_FAULT_ACTION_REPORT:
        recvFaultActionRpt(msg);
        break;

      case ID_OTHER_ACTUATORS_REPORT:
        recvOtherActuatorsRpt(msg);
        break;

      case ID_GPS_REFERENCE_REPORT:
        recvGpsReferenceRpt(msg);
        break;

      case ID_GPS_REMAINDER_REPORT:
        recvGpsRemainderRpt(msg);
        break;

      case ID_BRAKE_CMD:
        break;
      case ID_ACCELERATOR_PEDAL_CMD:
        break;
      case ID_STEERING_CMD:
        break;
      case ID_GEAR_CMD:
        break;
      default:
        break;
    }
  }
}

void RaptorDbwCAN::recvBrakeRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_BRAKE_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    bool brakeSystemFault =
      message->GetSignal("DBW_BrakeFault")->GetResult() ? true : false;
    bool dbwSystemFault = brakeSystemFault;

    faultBrakes(brakeSystemFault);
    faultWatchdog(dbwSystemFault, brakeSystemFault);

    overrideBrake(message->GetSignal("DBW_BrakeDriverActivity")->GetResult());
    BrakeReport brakeReport;
    brakeReport.header.stamp = msg->header.stamp;
    brakeReport.pedal_position = message->GetSignal("DBW_BrakePdlDriverInput")->GetResult();
    brakeReport.pedal_output = message->GetSignal("DBW_BrakePdlPosnFdbck")->GetResult();

    brakeReport.enabled =
      message->GetSignal("DBW_BrakeEnabled")->GetResult() ? true : false;
    brakeReport.driver_activity =
      message->GetSignal("DBW_BrakeDriverActivity")->GetResult() ? true : false;

    brakeReport.fault_brake_system = brakeSystemFault;

    brakeReport.rolling_counter = message->GetSignal("DBW_BrakeRollingCntr")->GetResult();

    brakeReport.brake_torque_actual =
      message->GetSignal("DBW_BrakePcntTorqueActual")->GetResult();

    brakeReport.intervention_active =
      message->GetSignal("DBW_BrakeInterventionActv")->GetResult() ? true : false;
    brakeReport.intervention_ready =
      message->GetSignal("DBW_BrakeInterventionReady")->GetResult() ? true : false;

    brakeReport.parking_brake.status =
      message->GetSignal("DBW_BrakeParkingBrkStatus")->GetResult();

    brakeReport.control_type.value = message->GetSignal("DBW_BrakeCtrlType")->GetResult();

    pub_brake_->publish(brakeReport);
    if (brakeSystemFault) {
      RCLCPP_WARN(this->get_logger(), "Brake report received a system fault.");
    }
  }
}

void RaptorDbwCAN::recvAccelPedalRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_ACCEL_PEDAL_REPORT);
  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    bool faultCh1 = message->GetSignal("DBW_AccelPdlFault_Ch1")->GetResult() ? true : false;
    bool faultCh2 = message->GetSignal("DBW_AccelPdlFault_Ch2")->GetResult() ? true : false;
    bool accelPdlSystemFault =
      message->GetSignal("DBW_AccelPdlFault")->GetResult() ? true : false;
    bool dbwSystemFault = accelPdlSystemFault;

    faultAcceleratorPedal(faultCh1 && faultCh2);
    faultWatchdog(dbwSystemFault, accelPdlSystemFault);

    overrideAcceleratorPedal(message->GetSignal("DBW_AccelPdlDriverActivity")->GetResult());

    AcceleratorPedalReport accelPedalReprt;
    accelPedalReprt.header.stamp = msg->header.stamp;
    accelPedalReprt.pedal_input =
      message->GetSignal("DBW_AccelPdlDriverInput")->GetResult();
    accelPedalReprt.pedal_output = message->GetSignal("DBW_AccelPdlPosnFdbck")->GetResult();
    accelPedalReprt.enabled =
      message->GetSignal("DBW_AccelPdlEnabled")->GetResult() ? true : false;
    accelPedalReprt.ignore_driver =
      message->GetSignal("DBW_AccelPdlIgnoreDriver")->GetResult() ? true : false;
    accelPedalReprt.driver_activity =
      message->GetSignal("DBW_AccelPdlDriverActivity")->GetResult() ? true : false;
    accelPedalReprt.torque_actual =
      message->GetSignal("DBW_AccelPcntTorqueActual")->GetResult();

    accelPedalReprt.control_type.value =
      message->GetSignal("DBW_AccelCtrlType")->GetResult();

    accelPedalReprt.rolling_counter =
      message->GetSignal("DBW_AccelPdlRollingCntr")->GetResult();

    accelPedalReprt.fault_accel_pedal_system = accelPdlSystemFault;

    accelPedalReprt.fault_ch1 = faultCh1;
    accelPedalReprt.fault_ch2 = faultCh2;

    pub_accel_pedal_->publish(accelPedalReprt);

    if (faultCh1 || faultCh2) {
      RCLCPP_WARN(this->get_logger(), "Acclerator pedal report received a system fault.");
    }
  }
}

void RaptorDbwCAN::recvSteeringRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_STEERING_REPORT);
  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    bool steeringSystemFault =
      message->GetSignal("DBW_SteeringFault")->GetResult() ? true : false;
    bool dbwSystemFault = steeringSystemFault;

    faultSteering(steeringSystemFault);

    faultWatchdog(dbwSystemFault);
    overrideSteering(
      message->GetSignal(
        "DBW_SteeringDriverActivity")->GetResult() ? true : false);

    SteeringReport steeringReport;
    steeringReport.header.stamp = msg->header.stamp;
    steeringReport.steering_wheel_angle =
      message->GetSignal("DBW_SteeringWhlAngleAct")->GetResult() * (M_PI / 180);
    steeringReport.steering_wheel_angle_cmd =
      message->GetSignal("DBW_SteeringWhlAngleDes")->GetResult() * (M_PI / 180);
    steeringReport.steering_wheel_torque =
      message->GetSignal("DBW_SteeringWhlPcntTrqCmd")->GetResult() * 0.0625;

    steeringReport.enabled =
      message->GetSignal("DBW_SteeringEnabled")->GetResult() ? true : false;
    steeringReport.driver_activity =
      message->GetSignal("DBW_SteeringDriverActivity")->GetResult() ? true : false;

    steeringReport.rolling_counter =
      message->GetSignal("DBW_SteeringRollingCntr")->GetResult();

    steeringReport.control_type.value =
      message->GetSignal("DBW_SteeringCtrlType")->GetResult();

    steeringReport.overheat_prevention_mode =
      message->GetSignal("DBW_OverheatPreventMode")->GetResult() ? true : false;

    steeringReport.steering_overheat_warning = message->GetSignal(
      "DBW_SteeringOverheatWarning")->GetResult() ? true : false;

    steeringReport.fault_steering_system = steeringSystemFault;

    pub_steering_->publish(steeringReport);

    publishJointStates(msg->header.stamp, steeringReport);

    if (steeringSystemFault) {
      RCLCPP_WARN(this->get_logger(), "Steering report received a system fault.");
    }
  }
}

void RaptorDbwCAN::recvGearRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_GEAR_REPORT);

  if (msg->dlc >= 1) {
    message->SetFrame(msg);

    bool driverActivity =
      message->GetSignal("DBW_PrndDriverActivity")->GetResult() ? true : false;

    overrideGear(driverActivity);
    GearReport out;
    out.header.stamp = msg->header.stamp;

    out.enabled = message->GetSignal("DBW_PrndCtrlEnabled")->GetResult() ? true : false;
    out.state.gear = message->GetSignal("DBW_PrndStateActual")->GetResult();
    out.driver_activity = driverActivity;
    out.gear_select_system_fault =
      message->GetSignal("DBW_PrndFault")->GetResult() ? true : false;

    out.reject = message->GetSignal("DBW_PrndStateReject")->GetResult() ? true : false;

    out.trans_curr_gear = message->GetSignal("DBW_TransCurGear")->GetResult();
    out.gear_mismatch_flash =
      message->GetSignal("DBW_PrndMismatchFlash")->GetResult() ? true : false;

    if (out.gear_mismatch_flash) {
      std::string err_msg(
        "ERROR - shift lever is in Park, but transmission is in Drive.");
      err_msg = err_msg + "Please adjust the shift lever.";
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    }

    pub_gear_->publish(out);
  }
}

void RaptorDbwCAN::recvWheelSpeedRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_REPORT_WHEEL_SPEED);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    WheelSpeedReport out;
    out.header.stamp = msg->header.stamp;

    out.front_left = message->GetSignal("DBW_WhlSpd_FL")->GetResult();
    out.front_right = message->GetSignal("DBW_WhlSpd_FR")->GetResult();
    out.rear_left = message->GetSignal("DBW_WhlSpd_RL")->GetResult();
    out.rear_right = message->GetSignal("DBW_WhlSpd_RR")->GetResult();

    pub_wheel_speeds_->publish(out);
    publishJointStates(msg->header.stamp, out);
  }
}

void RaptorDbwCAN::recvWheelPositionRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_REPORT_WHEEL_POSITION);
  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    WheelPositionReport out;
    out.header.stamp = msg->header.stamp;
    out.front_left = message->GetSignal("DBW_WhlPulseCnt_FL")->GetResult();
    out.front_right = message->GetSignal("DBW_WhlPulseCnt_FR")->GetResult();
    out.rear_left = message->GetSignal("DBW_WhlPulseCnt_RL")->GetResult();
    out.rear_right = message->GetSignal("DBW_WhlPulseCnt_RR")->GetResult();
    out.wheel_pulses_per_rev = message->GetSignal("DBW_WhlPulsesPerRev")->GetResult();

    pub_wheel_positions_->publish(out);
  }
}

void RaptorDbwCAN::recvTirePressureRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_REPORT_TIRE_PRESSURE);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    TirePressureReport out;
    out.header.stamp = msg->header.stamp;
    out.front_left = message->GetSignal("DBW_TirePressFL")->GetResult();
    out.front_right = message->GetSignal("DBW_TirePressFR")->GetResult();
    out.rear_left = message->GetSignal("DBW_TirePressRL")->GetResult();
    out.rear_right = message->GetSignal("DBW_TirePressRR")->GetResult();
    pub_tire_pressure_->publish(out);
  }
}

void RaptorDbwCAN::recvSurroundRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_REPORT_SURROUND);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    SurroundReport out;
    out.header.stamp = msg->header.stamp;

    out.front_radar_object_distance = message->GetSignal("DBW_Reserved2")->GetResult();
    out.rear_radar_object_distance = message->GetSignal("DBW_SonarRearDist")->GetResult();

    out.front_radar_distance_valid =
      message->GetSignal("DBW_Reserved3")->GetResult() ? true : false;
    out.parking_sonar_data_valid =
      message->GetSignal("DBW_SonarVld")->GetResult() ? true : false;

    out.rear_right.status = message->GetSignal("DBW_SonarArcNumRR")->GetResult();
    out.rear_left.status = message->GetSignal("DBW_SonarArcNumRL")->GetResult();
    out.rear_center.status = message->GetSignal("DBW_SonarArcNumRC")->GetResult();

    out.front_right.status = message->GetSignal("DBW_SonarArcNumFR")->GetResult();
    out.front_left.status = message->GetSignal("DBW_SonarArcNumFL")->GetResult();
    out.front_center.status = message->GetSignal("DBW_SonarArcNumFC")->GetResult();

    pub_surround_->publish(out);
  }
}

void RaptorDbwCAN::recvVinRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_VIN);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    if (message->GetSignal("DBW_VinMultiplexor")->GetResult() == VIN_MUX_VIN0) {
      vin_.push_back(message->GetSignal("DBW_VinDigit_01")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_02")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_03")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_04")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_05")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_06")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_07")->GetResult());
    } else if (message->GetSignal("DBW_VinMultiplexor")->GetResult() == VIN_MUX_VIN1) {
      vin_.push_back(message->GetSignal("DBW_VinDigit_08")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_09")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_10")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_11")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_12")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_13")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_14")->GetResult());
    } else if (message->GetSignal("DBW_VinMultiplexor")->GetResult() == VIN_MUX_VIN2) {
      vin_.push_back(message->GetSignal("DBW_VinDigit_15")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_16")->GetResult());
      vin_.push_back(message->GetSignal("DBW_VinDigit_17")->GetResult());
      std_msgs::msg::String msg; msg.data = vin_;
      pub_vin_->publish(msg);
    }
  }
}

void RaptorDbwCAN::recvImuRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_REPORT_IMU);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    sensor_msgs::msg::Imu out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_id_;

    out.angular_velocity.z =
      static_cast<double>(message->GetSignal("DBW_ImuYawRate")->GetResult()) *
      (M_PI / 180.0F);
    out.linear_acceleration.x =
      static_cast<double>(message->GetSignal("DBW_ImuAccelX")->GetResult());
    out.linear_acceleration.y =
      static_cast<double>(message->GetSignal("DBW_ImuAccelY")->GetResult());

    pub_imu_->publish(out);
  }
}

void RaptorDbwCAN::recvDriverInputRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_REPORT_DRIVER_INPUT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    DriverInputReport out;
    out.header.stamp = msg->header.stamp;

    out.turn_signal.value = message->GetSignal("DBW_DrvInptTurnSignal")->GetResult();
    out.high_beam_headlights.status = message->GetSignal("DBW_DrvInptHiBeam")->GetResult();
    out.wiper.status = message->GetSignal("DBW_DrvInptWiper")->GetResult();

    out.cruise_resume_button =
      message->GetSignal("DBW_DrvInptCruiseResumeBtn")->GetResult() ? true : false;
    out.cruise_cancel_button =
      message->GetSignal("DBW_DrvInptCruiseCancelBtn")->GetResult() ? true : false;
    out.cruise_accel_button =
      message->GetSignal("DBW_DrvInptCruiseAccelBtn")->GetResult() ? true : false;
    out.cruise_decel_button =
      message->GetSignal("DBW_DrvInptCruiseDecelBtn")->GetResult() ? true : false;
    out.cruise_on_off_button =
      message->GetSignal("DBW_DrvInptCruiseOnOffBtn")->GetResult() ? true : false;

    out.adaptive_cruise_on_off_button =
      message->GetSignal("DBW_DrvInptAccOnOffBtn")->GetResult() ? true : false;
    out.adaptive_cruise_increase_distance_button = message->GetSignal(
      "DBW_DrvInptAccIncDistBtn")->GetResult() ? true : false;
    out.adaptive_cruise_decrease_distance_button = message->GetSignal(
      "DBW_DrvInptAccDecDistBtn")->GetResult() ? true : false;

    out.steer_wheel_button_a =
      message->GetSignal("DBW_DrvInputStrWhlBtnA")->GetResult() ? true : false;
    out.steer_wheel_button_b =
      message->GetSignal("DBW_DrvInputStrWhlBtnB")->GetResult() ? true : false;
    out.steer_wheel_button_c =
      message->GetSignal("DBW_DrvInputStrWhlBtnC")->GetResult() ? true : false;
    out.steer_wheel_button_d =
      message->GetSignal("DBW_DrvInputStrWhlBtnD")->GetResult() ? true : false;
    out.steer_wheel_button_e =
      message->GetSignal("DBW_DrvInputStrWhlBtnE")->GetResult() ? true : false;

    out.door_or_hood_ajar =
      message->GetSignal("DBW_OccupAnyDoorOrHoodAjar")->GetResult() ? true : false;

    out.airbag_deployed =
      message->GetSignal("DBW_OccupAnyAirbagDeployed")->GetResult() ? true : false;
    out.any_seatbelt_unbuckled =
      message->GetSignal("DBW_OccupAnySeatbeltUnbuckled")->GetResult() ? true : false;

    pub_driver_input_->publish(out);
  }
}

void RaptorDbwCAN::recvMiscRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_MISC_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    MiscReport out;
    out.header.stamp = msg->header.stamp;

    out.fuel_level =
      static_cast<double>(message->GetSignal("DBW_MiscFuelLvl")->GetResult());
    out.drive_by_wire_enabled =
      static_cast<bool>(message->GetSignal("DBW_MiscByWireEnabled")->GetResult());
    out.vehicle_speed =
      static_cast<double>(message->GetSignal("DBW_MiscVehicleSpeed")->GetResult());
    out.software_build_number =
      message->GetSignal("DBW_SoftwareBuildNumber")->GetResult();
    out.general_actuator_fault =
      message->GetSignal("DBW_MiscFault")->GetResult() ? true : false;
    out.by_wire_ready =
      message->GetSignal("DBW_MiscByWireReady")->GetResult() ? true : false;
    out.general_driver_activity =
      message->GetSignal("DBW_MiscDriverActivity")->GetResult() ? true : false;
    out.comms_fault =
      message->GetSignal("DBW_MiscAKitCommFault")->GetResult() ? true : false;
    out.ambient_temp =
      static_cast<double>(message->GetSignal("DBW_AmbientTemp")->GetResult());

    pub_misc_->publish(out);
  }
}

void RaptorDbwCAN::recvLowVoltageSystemRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_LOW_VOLTAGE_SYSTEM_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    LowVoltageSystemReport lvSystemReport;
    lvSystemReport.header.stamp = msg->header.stamp;

    lvSystemReport.vehicle_battery_volts =
      static_cast<double>(message->GetSignal("DBW_LvVehBattVlt")->GetResult());
    lvSystemReport.vehicle_battery_current =
      static_cast<double>(message->GetSignal("DBW_LvBattCurr")->GetResult());
    lvSystemReport.vehicle_alternator_current =
      static_cast<double>(message->GetSignal("DBW_LvAlternatorCurr")->GetResult());

    lvSystemReport.dbw_battery_volts =
      static_cast<double>(message->GetSignal("DBW_LvDbwBattVlt")->GetResult());
    lvSystemReport.dcdc_current =
      static_cast<double>(message->GetSignal("DBW_LvDcdcCurr")->GetResult());

    lvSystemReport.aux_inverter_contactor =
      message->GetSignal("DBW_LvInvtrContactorCmd")->GetResult() ? true : false;

    pub_low_voltage_system_->publish(lvSystemReport);
  }
}

void RaptorDbwCAN::recvBrake2Rpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_BRAKE_2_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    Brake2Report brake2Report;
    brake2Report.header.stamp = msg->header.stamp;

    brake2Report.brake_pressure = message->GetSignal("DBW_BrakePress_bar")->GetResult();

    brake2Report.estimated_road_slope =
      message->GetSignal("DBW_RoadSlopeEstimate")->GetResult();

    brake2Report.speed_set_point = message->GetSignal("DBW_SpeedSetpt")->GetResult();

    pub_brake_2_report_->publish(brake2Report);
  }
}

void RaptorDbwCAN::recvSteering2Rpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_STEERING_2_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    Steering2Report steering2Report;
    steering2Report.header.stamp = msg->header.stamp;

    steering2Report.vehicle_curvature_actual = message->GetSignal(
      "DBW_SteeringVehCurvatureAct")->GetResult();

    steering2Report.max_torque_driver =
      message->GetSignal("DBW_SteerTrq_Driver")->GetResult();

    steering2Report.max_torque_motor =
      message->GetSignal("DBW_SteerTrq_Motor")->GetResult();

    steering2Report.expect_torque_driver =
      message->GetSignal("DBW_SteerTrq_DriverExpectedValue")->GetResult();

    pub_steering_2_report_->publish(steering2Report);
  }
}

void RaptorDbwCAN::recvFaultActionRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_FAULT_ACTION_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    FaultActionsReport faultActionsReport;
    faultActionsReport.header.stamp = msg->header.stamp;

    faultActionsReport.autonomous_disabled_no_brakes = message->GetSignal(
      "DBW_FltAct_AutonDsblNoBrakes")->GetResult();

    faultActionsReport.autonomous_disabled_apply_brakes = message->GetSignal(
      "DBW_FltAct_AutonDsblApplyBrakes")->GetResult();
    faultActionsReport.can_gateway_disabled =
      message->GetSignal("DBW_FltAct_CANGatewayDsbl")->GetResult();
    faultActionsReport.inverter_contactor_disabled = message->GetSignal(
      "DBW_FltAct_InvtrCntctrDsbl")->GetResult();
    faultActionsReport.prevent_enter_autonomous_mode = message->GetSignal(
      "DBW_FltAct_PreventEnterAutonMode")->GetResult();
    faultActionsReport.warn_driver_only =
      message->GetSignal("DBW_FltAct_WarnDriverOnly")->GetResult();
    faultActionsReport.chime_fcw_beeps =
      message->GetSignal("DBW_FltAct_Chime_FcwBeeps")->GetResult();

    pub_fault_actions_report_->publish(faultActionsReport);
  }
}

void RaptorDbwCAN::recvOtherActuatorsRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_OTHER_ACTUATORS_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    OtherActuatorsReport out;
    out.header.stamp = msg->header.stamp;

    out.ignition_state.status = message->GetSignal(
      "DBW_IgnitionState")->GetResult();
    out.horn_state.status = message->GetSignal(
      "DBW_HornState")->GetResult();

    out.turn_signal_state.value = message->GetSignal(
      "DBW_TurnSignalState")->GetResult();
    out.turn_signal_sync = message->GetSignal(
      "DBW_TurnSignalSyncBit")->GetResult() ? true : false;
    out.high_beam_state.value = message->GetSignal(
      "DBW_HighBeamState")->GetResult();
    out.low_beam_state.status = message->GetSignal(
      "DBW_LowBeamState")->GetResult();

    out.front_wiper_state.status = message->GetSignal(
      "DBW_FrontWiperState")->GetResult();
    out.rear_wiper_state.status = message->GetSignal(
      "DBW_RearWiperState")->GetResult();

    out.right_rear_door_state.value = message->GetSignal(
      "DBW_RightRearDoorState")->GetResult();
    out.left_rear_door_state.value = message->GetSignal(
      "DBW_LeftRearDoorState")->GetResult();
    out.liftgate_door_state.value = message->GetSignal(
      "DBW_LiftgateDoorState")->GetResult();
    out.door_lock_state.value = message->GetSignal(
      "DBW_DoorLockState")->GetResult();

    pub_other_actuators_report_->publish(out);
  }
}

void RaptorDbwCAN::recvGpsReferenceRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_GPS_REFERENCE_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    GpsReferenceReport out;
    out.header.stamp = msg->header.stamp;

    out.ref_latitude = message->GetSignal(
      "DBW_GpsRefLat")->GetResult();

    out.ref_longitude = message->GetSignal(
      "DBW_GpsRefLong")->GetResult();

    pub_gps_reference_report_->publish(out);
  }
}

void RaptorDbwCAN::recvGpsRemainderRpt(const can_msgs::msg::Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_GPS_REMAINDER_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    GpsRemainderReport out;
    out.header.stamp = msg->header.stamp;

    out.rem_latitude = message->GetSignal(
      "DBW_GpsRemainderLat")->GetResult();

    out.rem_longitude = message->GetSignal(
      "DBW_GpsRemainderLong")->GetResult();

    pub_gps_remainder_report_->publish(out);
  }
}

void RaptorDbwCAN::recvBrakeCmd(const BrakeCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_BrakeRequest");

  message->GetSignal("AKit_BrakePedalReq")->SetResult(0);
  message->GetSignal("AKit_BrakeCtrlEnblReq")->SetResult(0);
  message->GetSignal("AKit_BrakeCtrlReqType")->SetResult(0);
  message->GetSignal("AKit_BrakePcntTorqueReq")->SetResult(0);
  message->GetSignal("AKit_SpeedModeDecelLim")->SetResult(0);
  message->GetSignal("AKit_SpeedModeNegJerkLim")->SetResult(0);
  message->GetSignal("AKit_ParkingBrkReq")->SetResult(0);

  if (enabled()) {
    if (msg->control_type.value == ActuatorControlMode::OPEN_LOOP) {
      message->GetSignal("AKit_BrakeCtrlReqType")->SetResult(0);
      message->GetSignal("AKit_BrakePedalReq")->SetResult(msg->pedal_cmd);
    } else if (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_ACTUATOR) {
      message->GetSignal("AKit_BrakeCtrlReqType")->SetResult(1);
      message->GetSignal("AKit_BrakePcntTorqueReq")->SetResult(msg->torque_cmd);
    } else if (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_VEHICLE) {
      message->GetSignal("AKit_BrakeCtrlReqType")->SetResult(2);
      message->GetSignal("AKit_SpeedModeDecelLim")->SetResult(msg->decel_limit);
      message->GetSignal("AKit_SpeedModeNegJerkLim")->SetResult(msg->decel_negative_jerk_limit);
    } else {
      message->GetSignal("AKit_BrakeCtrlReqType")->SetResult(0);
    }

    if (msg->enable) {
      message->GetSignal("AKit_BrakeCtrlEnblReq")->SetResult(1);
    }

    if ((msg->control_type.value == ActuatorControlMode::OPEN_LOOP) ||
      (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_ACTUATOR) ||
      (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_VEHICLE))
    {
      message->GetSignal("AKit_ParkingBrkReq")->SetResult(msg->park_brake_cmd.status);
    }
  }

  NewEagle::DbcSignal * cnt = message->GetSignal("AKit_BrakeRollingCntr");
  cnt->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvAcceleratorPedalCmd(
  const AcceleratorPedalCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_AccelPdlRequest");

  message->GetSignal("AKit_AccelPdlReq")->SetResult(0);
  message->GetSignal("AKit_AccelPdlEnblReq")->SetResult(0);
  message->GetSignal("Akit_AccelPdlIgnoreDriverOvrd")->SetResult(0);
  message->GetSignal("AKit_AccelPdlRollingCntr")->SetResult(0);
  message->GetSignal("AKit_AccelReqType")->SetResult(0);
  message->GetSignal("AKit_AccelPcntTorqueReq")->SetResult(0);
  message->GetSignal("AKit_AccelPdlChecksum")->SetResult(0);
  message->GetSignal("AKit_SpeedReq")->SetResult(0);
  message->GetSignal("AKit_SpeedModeRoadSlope")->SetResult(0);
  message->GetSignal("AKit_SpeedModeAccelLim")->SetResult(0);
  message->GetSignal("AKit_SpeedModePosJerkLim")->SetResult(0);

  if (enabled()) {
    if (msg->control_type.value == ActuatorControlMode::OPEN_LOOP) {
      message->GetSignal("AKit_AccelReqType")->SetResult(0);
      message->GetSignal("AKit_AccelPdlReq")->SetResult(msg->pedal_cmd);
    } else if (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_ACTUATOR) {
      message->GetSignal("AKit_AccelReqType")->SetResult(1);
      message->GetSignal("AKit_AccelPcntTorqueReq")->SetResult(msg->torque_cmd);
    } else if (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_VEHICLE) {
      message->GetSignal("AKit_AccelReqType")->SetResult(2);
      message->GetSignal("AKit_SpeedReq")->SetResult(msg->speed_cmd);
      message->GetSignal("AKit_SpeedModeRoadSlope")->SetResult(msg->road_slope);
      message->GetSignal("AKit_SpeedModeAccelLim")->SetResult(msg->accel_limit);
      message->GetSignal("AKit_SpeedModePosJerkLim")->SetResult(msg->accel_positive_jerk_limit);
    } else {
      message->GetSignal("AKit_AccelReqType")->SetResult(0);
    }

    if (msg->enable) {
      message->GetSignal("AKit_AccelPdlEnblReq")->SetResult(1);
    }
  }

  NewEagle::DbcSignal * cnt = message->GetSignal("AKit_AccelPdlRollingCntr");
  cnt->SetResult(msg->rolling_counter);

  if (msg->ignore) {
    message->GetSignal("Akit_AccelPdlIgnoreDriverOvrd")->SetResult(1);
  }

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvSteeringCmd(const SteeringCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_SteeringRequest");

  message->GetSignal("AKit_SteeringWhlAngleReq")->SetResult(0);
  message->GetSignal("AKit_SteeringWhlAngleVelocityLim")->SetResult(0);
  message->GetSignal("AKit_SteerCtrlEnblReq")->SetResult(0);
  message->GetSignal("AKit_SteeringWhlIgnoreDriverOvrd")->SetResult(0);
  message->GetSignal("AKit_SteeringWhlPcntTrqReq")->SetResult(0);
  message->GetSignal("AKit_SteeringReqType")->SetResult(0);
  message->GetSignal("AKit_SteeringVehCurvatureReq")->SetResult(0);
  message->GetSignal("AKit_SteeringChecksum")->SetResult(0);

  if (enabled()) {
    if (msg->control_type.value == ActuatorControlMode::OPEN_LOOP) {
      message->GetSignal("AKit_SteeringReqType")->SetResult(0);
      message->GetSignal("AKit_SteeringWhlPcntTrqReq")->SetResult(msg->torque_cmd);
    } else if (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_ACTUATOR) {
      message->GetSignal("AKit_SteeringReqType")->SetResult(1);
      double scmd =
        std::max(
        -470.0F,
        std::min(
          470.0F, static_cast<float>(
            msg->angle_cmd * (180.0F / M_PI * 1.0F))));
      message->GetSignal("AKit_SteeringWhlAngleReq")->SetResult(scmd);
    } else if (msg->control_type.value == ActuatorControlMode::CLOSED_LOOP_VEHICLE) {
      message->GetSignal("AKit_SteeringReqType")->SetResult(2);
      message->GetSignal("AKit_SteeringVehCurvatureReq")->SetResult(msg->vehicle_curvature_cmd);
    } else {
      message->GetSignal("AKit_SteeringReqType")->SetResult(0);
    }

    if (fabsf(msg->angle_velocity) > 0) {
      uint16_t vcmd =
        std::max(
        1.0F,
        std::min(
          254.0F, static_cast<float>(
            std::roundf(std::fabs(msg->angle_velocity) * 180.0F / M_PI / 2.0F))));

      message->GetSignal("AKit_SteeringWhlAngleVelocityLim")->SetResult(vcmd);
    }
    if (msg->enable) {
      message->GetSignal("AKit_SteerCtrlEnblReq")->SetResult(1);
    }
  }

  if (msg->ignore) {
    message->GetSignal("AKit_SteeringWhlIgnoreDriverOvrd")->SetResult(1);
  }

  message->GetSignal("AKit_SteerRollingCntr")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvGearCmd(const GearCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_PrndRequest");

  message->GetSignal("AKit_PrndCtrlEnblReq")->SetResult(0);
  message->GetSignal("AKit_PrndStateReq")->SetResult(0);
  message->GetSignal("AKit_PrndChecksum")->SetResult(0);

  if (enabled()) {
    if (msg->enable) {
      message->GetSignal("AKit_PrndCtrlEnblReq")->SetResult(1);
    }

    message->GetSignal("AKit_PrndStateReq")->SetResult(msg->cmd.gear);
  }

  message->GetSignal("AKit_PrndRollingCntr")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvGlobalEnableCmd(const GlobalEnableCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_GlobalEnbl");

  message->GetSignal("AKit_GlobalEnblRollingCntr")->SetResult(0);
  message->GetSignal("AKit_GlobalByWireEnblReq")->SetResult(0);
  message->GetSignal("AKit_EnblJoystickLimits")->SetResult(0);
  message->GetSignal("AKit_SoftwareBuildNumber")->SetResult(0);
  message->GetSignal("Akit_GlobalEnblChecksum")->SetResult(0);

  if (enabled()) {
    if (msg->global_enable) {
      message->GetSignal("AKit_GlobalByWireEnblReq")->SetResult(1);
    }

    if (msg->enable_joystick_limits) {
      message->GetSignal("AKit_EnblJoystickLimits")->SetResult(1);
    }

    message->GetSignal("AKit_SoftwareBuildNumber")->SetResult(msg->ecu_build_number);
  }

  message->GetSignal("AKit_GlobalEnblRollingCntr")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvMiscCmd(const MiscCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_OtherActuators");

  message->GetSignal("AKit_TurnSignalReq")->SetResult(0);
  message->GetSignal("AKit_RightRearDoorReq")->SetResult(0);
  message->GetSignal("AKit_HighBeamReq")->SetResult(0);
  message->GetSignal("AKit_FrontWiperReq")->SetResult(0);
  message->GetSignal("AKit_RearWiperReq")->SetResult(0);
  message->GetSignal("AKit_IgnitionReq")->SetResult(0);
  message->GetSignal("AKit_LeftRearDoorReq")->SetResult(0);
  message->GetSignal("AKit_LiftgateDoorReq")->SetResult(0);
  message->GetSignal("AKit_BlockBasicCruiseCtrlBtns")->SetResult(0);
  message->GetSignal("AKit_BlockAdapCruiseCtrlBtns")->SetResult(0);
  message->GetSignal("AKit_BlockTurnSigStalkInpts")->SetResult(0);
  message->GetSignal("AKit_OtherChecksum")->SetResult(0);
  message->GetSignal("AKit_HornReq")->SetResult(0);
  message->GetSignal("AKit_LowBeamReq")->SetResult(0);
  message->GetSignal("AKit_DoorLockReq")->SetResult(0);

  if (enabled()) {
    message->GetSignal("AKit_TurnSignalReq")->SetResult(msg->cmd.value);

    message->GetSignal("AKit_RightRearDoorReq")->SetResult(msg->door_request_right_rear.value);
    message->GetSignal("AKit_HighBeamReq")->SetResult(msg->high_beam_cmd.status);

    message->GetSignal("AKit_FrontWiperReq")->SetResult(msg->front_wiper_cmd.status);
    message->GetSignal("AKit_RearWiperReq")->SetResult(msg->rear_wiper_cmd.status);

    message->GetSignal("AKit_IgnitionReq")->SetResult(msg->ignition_cmd.status);

    message->GetSignal("AKit_LeftRearDoorReq")->SetResult(msg->door_request_left_rear.value);
    message->GetSignal("AKit_LiftgateDoorReq")->SetResult(msg->door_request_lift_gate.value);

    message->GetSignal("AKit_BlockBasicCruiseCtrlBtns")->SetResult(
      msg->block_standard_cruise_buttons);
    message->GetSignal("AKit_BlockAdapCruiseCtrlBtns")->SetResult(
      msg->block_adaptive_cruise_buttons);
    message->GetSignal("AKit_BlockTurnSigStalkInpts")->SetResult(msg->block_turn_signal_stalk);

    message->GetSignal("AKit_HornReq")->SetResult(msg->horn_cmd);
    message->GetSignal("AKit_LowBeamReq")->SetResult(msg->low_beam_cmd.status);
    message->GetSignal("AKit_DoorLockReq")->SetResult(msg->door_lock_cmd.value);
  }

  message->GetSignal("AKit_OtherRollingCntr")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

/// \brief DBW Enabled needs to publish when its state changes.
/// \returns TRUE when DBW enable state changes, FALSE otherwise
bool RaptorDbwCAN::publishDbwEnabled()
{
  bool change = false;
  bool en = enabled();
  if (prev_enable_ != en) {
    std_msgs::msg::Bool msg;
    msg.data = en;
    pub_sys_enable_->publish(msg);
    change = true;
  }
  prev_enable_ = en;
  return change;
}

void RaptorDbwCAN::timerCallback()
{
  if (clear()) {
    can_msgs::msg::Frame out;
    out.is_extended = false;

    if (override_brake_) {
      // Might have an issue with WatchdogCntr when these are set.
      NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_BrakeRequest");
      message->GetSignal("AKit_BrakePedalReq")->SetResult(0);
      message->GetSignal("AKit_BrakeCtrlEnblReq")->SetResult(0);
      // message->GetSignal("AKit_BrakePedalCtrlMode")->SetResult(0);
      pub_can_->publish(message->GetFrame());
    }

    if (override_accelerator_pedal_) {
      // Might have an issue with WatchdogCntr when these are set.
      NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_AccelPdlRequest");
      message->GetSignal("AKit_AccelPdlReq")->SetResult(0);
      message->GetSignal("AKit_AccelPdlEnblReq")->SetResult(0);
      message->GetSignal("Akit_AccelPdlIgnoreDriverOvrd")->SetResult(0);
      // message->GetSignal("AKit_AccelPdlCtrlMode")->SetResult(0);
      pub_can_->publish(message->GetFrame());
    }

    if (override_steering_) {
      // Might have an issue with WatchdogCntr when these are set.
      NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_SteeringRequest");
      message->GetSignal("AKit_SteeringWhlAngleReq")->SetResult(0);
      message->GetSignal("AKit_SteeringWhlAngleVelocityLim")->SetResult(0);
      message->GetSignal("AKit_SteeringWhlIgnoreDriverOvrd")->SetResult(0);
      message->GetSignal("AKit_SteeringWhlPcntTrqReq")->SetResult(0);
      // message->GetSignal("AKit_SteeringWhlCtrlMode")->SetResult(0);
      // message->GetSignal("AKit_SteeringWhlCmdType")->SetResult(0);

      pub_can_->publish(message->GetFrame());
    }

    if (override_gear_) {
      NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_GearRequest");
      message->GetSignal("AKit_PrndStateCmd")->SetResult(0);
      message->GetSignal("AKit_PrndChecksum")->SetResult(0);
      pub_can_->publish(message->GetFrame());
    }
  }
}

void RaptorDbwCAN::enableSystem()
{
  if (!enable_) {
    if (fault()) {
      if (fault_steering_cal_) {
        RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Steering calibration fault.");
      }
      if (fault_brakes_) {
        RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Braking fault.");
      }
      if (fault_accelerator_pedal_) {
        RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Accelerator Pedal fault.");
      }
      if (fault_steering_) {
        RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Steering fault.");
      }
      if (fault_watchdog_) {
        RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Watchdog fault.");
      }
    } else {
      enable_ = true;
      if (publishDbwEnabled()) {
        RCLCPP_INFO(this->get_logger(), "DBW system enabled.");
      } else {
        RCLCPP_WARN(this->get_logger(), "DBW system failed to enable. Check driver overrides.");
      }
    }
  }
}

void RaptorDbwCAN::disableSystem()
{
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    RCLCPP_INFO(this->get_logger(), "DBW system disabled - system disabled.");
  }
}

void RaptorDbwCAN::buttonCancel()
{
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    RCLCPP_INFO(this->get_logger(), "DBW system disabled - button canceled.");
  }
}

void RaptorDbwCAN::overrideBrake(bool override)
{
  bool en = enabled();
  if (override && en) {
    enable_ = false;
  }
  override_brake_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_WARN(this->get_logger(), "DBW system disabled - brake override");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no brake override");
    }
  }
}

void RaptorDbwCAN::overrideAcceleratorPedal(bool override)
{
  bool en = enabled();
  if (override && en) {
    enable_ = false;
  }
  override_accelerator_pedal_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_WARN(this->get_logger(), "DBW system disabled - accelerator pedal override");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no accelerator pedal override");
    }
  }
}

void RaptorDbwCAN::overrideSteering(bool override)
{
  bool en = enabled();
  if (override && en) {
    enable_ = false;
  }
  override_steering_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_WARN(this->get_logger(), "DBW system disabled - steering override");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no steering override");
    }
  }
}

void RaptorDbwCAN::overrideGear(bool override)
{
  bool en = enabled();
  if (override && en) {
    enable_ = false;
  }
  override_gear_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_WARN(this->get_logger(), "DBW system disabled - PRND gear override");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no PRND gear override");
    }
  }
}

void RaptorDbwCAN::timeoutBrake(bool timeout, bool enabled)
{
  if (!timeout_brakes_ && enabled_brakes_ && timeout && !enabled) {
    RCLCPP_WARN(this->get_logger(), "Brakes have timed out");
  }
  timeout_brakes_ = timeout;
  enabled_brakes_ = enabled;
}

void RaptorDbwCAN::timeoutAcceleratorPedal(bool timeout, bool enabled)
{
  if (!timeout_accelerator_pedal_ && enabled_accelerator_pedal_ && timeout && !enabled) {
    RCLCPP_WARN(this->get_logger(), "Accelerator Pedal has timed out");
  }
  timeout_accelerator_pedal_ = timeout;
  enabled_accelerator_pedal_ = enabled;
}

void RaptorDbwCAN::timeoutSteering(bool timeout, bool enabled)
{
  if (!timeout_steering_ && enabled_steering_ && timeout && !enabled) {
    RCLCPP_WARN(this->get_logger(), "Steering has timed out");
  }
  timeout_steering_ = timeout;
  enabled_steering_ = enabled;
}

void RaptorDbwCAN::faultBrakes(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_brakes_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Braking fault.");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no braking fault");
    }
  }
}

void RaptorDbwCAN::faultAcceleratorPedal(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_accelerator_pedal_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Accelerator Pedal fault.");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no accelerator pedal fault");
    }
  }
}

void RaptorDbwCAN::faultSteering(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Steering fault.");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no steering fault");
    }
  }
}

void RaptorDbwCAN::faultSteeringCal(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_cal_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Steering calibration fault.");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no steering calibration fault");
    }
  }
}

void RaptorDbwCAN::faultWatchdog(bool fault, uint8_t src, bool braking)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_watchdog_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(this->get_logger(), "DBW system disabled. Watchdog fault.");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBW system enabled - no watchdog fault");
    }
  }
  if (braking && !fault_watchdog_using_brakes_) {
    RCLCPP_ERROR(this->get_logger(), "Watchdog - new braking fault.");
  } else if (!braking && fault_watchdog_using_brakes_) {
    RCLCPP_INFO(this->get_logger(), "Watchdog - braking fault is cleared.");
  }
  if (fault && src && !fault_watchdog_warned_) {
    RCLCPP_WARN(this->get_logger(), "Watchdog - new fault warning.");
    fault_watchdog_warned_ = true;
  } else if (!fault) {
    fault_watchdog_warned_ = false;
  }
  fault_watchdog_using_brakes_ = braking;
  if (fault && !fault_watchdog_using_brakes_ && fault_watchdog_warned_) {
    RCLCPP_ERROR(this->get_logger(), "Watchdog - new non-braking fault.");
  }
}

void RaptorDbwCAN::faultWatchdog(bool fault, uint8_t src)
{
  faultWatchdog(fault, src, fault_watchdog_using_brakes_);   // No change to 'using brakes' status
}

void RaptorDbwCAN::publishJointStates(
  const rclcpp::Time stamp,
  const WheelSpeedReport wheels)
{
  double dt = stamp.seconds() - joint_state_.header.stamp.sec;
  joint_state_.velocity[JOINT_FL] = wheels.front_left;
  joint_state_.velocity[JOINT_FR] = wheels.front_right;
  joint_state_.velocity[JOINT_RL] = wheels.rear_left;
  joint_state_.velocity[JOINT_RR] = wheels.rear_right;

  if (dt < 0.5) {
    for (unsigned int i = JOINT_FL; i <= JOINT_RR; i++) {
      joint_state_.position[i] = fmod(
        joint_state_.position[i] + dt * joint_state_.velocity[i],
        2 * M_PI);
    }
  }
  joint_state_.header.stamp = rclcpp::Time(stamp);
  pub_joint_states_->publish(joint_state_);
}

void RaptorDbwCAN::publishJointStates(
  const rclcpp::Time stamp,
  const SteeringReport steering)
{
  double dt = stamp.seconds() - joint_state_.header.stamp.sec;
  const double L = acker_wheelbase_;
  const double W = acker_track_;
  const double r = L / tan(steering.steering_wheel_angle / steering_ratio_);
  joint_state_.position[JOINT_SL] = atan(L / (r - W / 2));
  joint_state_.position[JOINT_SR] = atan(L / (r + W / 2));

  if (dt < 0.5) {
    for (unsigned int i = JOINT_FL; i <= JOINT_RR; i++) {
      joint_state_.position[i] = fmod(
        joint_state_.position[i] + dt * joint_state_.velocity[i],
        2 * M_PI);
    }
  }
  joint_state_.header.stamp = rclcpp::Time(stamp);
  pub_joint_states_->publish(joint_state_);
}
}  // namespace raptor_dbw_can
