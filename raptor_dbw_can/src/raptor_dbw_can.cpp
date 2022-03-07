// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include "raptor_dbw_can/raptor_dbw_can.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace raptor_dbw_can
{

RaptorDbwCAN::RaptorDbwCAN(
  const rclcpp::NodeOptions & options,
  std::string dbw_dbc_file,
  float max_steer_angle)
: Node("raptor_dbw_can_node", options),
  dbw_dbc_file_{dbw_dbc_file},
  max_steer_angle_{max_steer_angle}
{
  // Initialize enable state machine
  int i{0};
  for (i = 0; i < NUM_ENABLES; i++) {
    enables_[i] = (i == EN_DBW_PREV) ? true : false;
  }
  for (i = 0; i < NUM_OVERRIDES; i++) {
    overrides_[i] = false;
  }
  for (i = 0; i < NUM_IGNORES; i++) {
    ignores_[i] = false;
  }
  for (i = 0; i < NUM_FAULTS; i++) {
    faults_[i] = false;
  }

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
  pub_can_ = this->create_publisher<Frame>("can_rx", 20);
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
  pub_exit_report_ = this->create_publisher<ExitReport>(
    "exit_report", 20);
  pub_fault_actions_report_ = this->create_publisher<FaultActionsReport>(
    "fault_actions_report", 20);
  pub_other_actuators_report_ = this->create_publisher<OtherActuatorsReport>(
    "other_actuators_report", 20);
  pub_gps_reference_report_ = this->create_publisher<GpsReferenceReport>(
    "gps_reference_report", 20);
  pub_gps_remainder_report_ = this->create_publisher<GpsRemainderReport>(
    "gps_remainder_report", 20);

  pub_imu_ = this->create_publisher<Imu>("imu/data_raw", 10);
  pub_joint_states_ = this->create_publisher<JointState>("joint_states", 10);
  pub_vin_ = this->create_publisher<String>("vin", 1);
  pub_driver_input_ = this->create_publisher<DriverInputReport>(
    "driver_input_report", 2);
  pub_misc_ = this->create_publisher<MiscReport>("misc_report", 2);
  pub_sys_enable_ = this->create_publisher<Bool>("dbw_enabled", 1);
  publishDbwEnabled();

  // Set up Subscribers
  sub_enable_ = this->create_subscription<Empty>(
    "enable", 10, std::bind(&RaptorDbwCAN::recvEnable, this, std::placeholders::_1));

  sub_disable_ = this->create_subscription<Empty>(
    "disable", 10, std::bind(&RaptorDbwCAN::recvDisable, this, std::placeholders::_1));

  sub_can_ = this->create_subscription<Frame>(
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

  sub_imu_ = this->create_subscription<Imu>(
    "imu_cmd", 1, std::bind(&RaptorDbwCAN::recvImuCmd, this, std::placeholders::_1));

  sub_misc_ = this->create_subscription<MiscCmd>(
    "misc_cmd", 1, std::bind(&RaptorDbwCAN::recvMiscCmd, this, std::placeholders::_1));

  sub_global_enable_ = this->create_subscription<GlobalEnableCmd>(
    "global_enable_cmd", 1,
    std::bind(&RaptorDbwCAN::recvGlobalEnableCmd, this, std::placeholders::_1));

  pdu1_relay_pub_ = this->create_publisher<RelayCommand>(
    "/pduB/relay_cmd", 1000);
  count_ = 0;

  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

  // Set up Timer
  timer_200ms = this->create_wall_timer(
    200ms, std::bind(&RaptorDbwCAN::timer_200ms_Callback, this));

  // Set up Timeout check
  timer_10ms_ = this->create_wall_timer(
    10ms, std::bind(&RaptorDbwCAN::timer_10ms_Callback, this));
}

RaptorDbwCAN::~RaptorDbwCAN()
{
}

void RaptorDbwCAN::recvEnable(const Empty::SharedPtr msg)
{
  if (msg != NULL) {
    enableSystem();
  }
}

void RaptorDbwCAN::recvDisable(const Empty::SharedPtr msg)
{
  if (msg != NULL) {
    disableSystem();
  }
}

void RaptorDbwCAN::recvCAN(const Frame::SharedPtr msg)
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

      case ID_IMU_REPORT:
        recvImuRpt(msg);
        break;

      case ID_IMU_2_REPORT:
        recvImu2Rpt(msg);
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

      case ID_EXIT_REPORT:
        recvExitRpt(msg);
        break;

      case ID_BRAKE_CMD:
        break;
      case ID_ACCELERATOR_PEDAL_CMD:
        break;
      case ID_STEERING_CMD:
        break;
      case ID_GEAR_CMD:
        break;
      case ID_IMU_ACCEL_CMD:
        break;
      case ID_IMU_ROTATE_CMD:
        break;
      default:
        break;
    }
  }
}

void RaptorDbwCAN::recvBrakeRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_BRAKE_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    bool brakeSystemFault =
      message->GetSignal("DBW_BrakeFault")->GetResult() ? true : false;
    bool dbwSystemFault = brakeSystemFault;
    bool driverActivity = message->GetSignal("DBW_BrakeDriverActivity")->GetResult() ? true : false;

    setFault(FAULT_BRAKE, brakeSystemFault);
    faultWatchdog(dbwSystemFault, brakeSystemFault);
    setOverride(OVR_BRAKE, driverActivity, false);

    BrakeReport brakeReport;
    brakeReport.header.stamp = msg->header.stamp;
    brakeReport.pedal_position = message->GetSignal("DBW_BrakePdlDriverInput")->GetResult();
    brakeReport.pedal_output = message->GetSignal("DBW_BrakePdlPosnFdbck")->GetResult();

    brakeReport.enabled =
      message->GetSignal("DBW_BrakeEnabled")->GetResult() ? true : false;
    brakeReport.driver_activity = driverActivity;

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
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), m_clock, CLOCK_1_SEC,
        "Brake report received a system fault.");
    }
  }
}

void RaptorDbwCAN::recvAccelPedalRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_ACCEL_PEDAL_REPORT);
  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    bool faultCh1 = message->GetSignal("DBW_AccelPdlFault_Ch1")->GetResult() ? true : false;
    bool faultCh2 = message->GetSignal("DBW_AccelPdlFault_Ch2")->GetResult() ? true : false;
    bool accelPdlSystemFault =
      message->GetSignal("DBW_AccelPdlFault")->GetResult() ? true : false;
    bool dbwSystemFault = accelPdlSystemFault;

    setFault(FAULT_ACCEL, faultCh1 && faultCh2);
    faultWatchdog(dbwSystemFault, accelPdlSystemFault);
    setOverride(
      OVR_ACCEL, message->GetSignal("DBW_AccelPdlDriverActivity")->GetResult(),
      ignores_[IGNORE_ACCEL]);

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
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), m_clock, CLOCK_1_SEC,
        "Acclerator pedal report received a system fault.");
    }
  }
}

void RaptorDbwCAN::recvSteeringRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_STEERING_REPORT);
  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    bool steeringSystemFault =
      message->GetSignal("DBW_SteeringFault")->GetResult() ? true : false;
    bool dbwSystemFault = steeringSystemFault;
    bool driverActivity =
      message->GetSignal("DBW_SteeringDriverActivity")->GetResult() ? true : false;

    setFault(FAULT_STEER, steeringSystemFault);
    faultWatchdog(dbwSystemFault);
    setOverride(OVR_STEER, driverActivity, ignores_[IGNORE_STEER]);

    SteeringReport steeringReport;
    steeringReport.header.stamp = msg->header.stamp;
    steeringReport.steering_wheel_angle =
      message->GetSignal("DBW_SteeringWhlAngleAct")->GetResult();
    steeringReport.steering_wheel_angle_cmd =
      message->GetSignal("DBW_SteeringWhlAngleDes")->GetResult();
    steeringReport.steering_wheel_torque =
      message->GetSignal("DBW_SteeringWhlPcntTrqCmd")->GetResult() * 0.0625;

    steeringReport.enabled =
      message->GetSignal("DBW_SteeringEnabled")->GetResult() ? true : false;
    steeringReport.driver_activity = driverActivity;

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
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), m_clock, CLOCK_1_SEC,
        "Steering report received a system fault.");
    }
  }
}

void RaptorDbwCAN::recvGearRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_GEAR_REPORT);

  if (msg->dlc >= 1) {
    message->SetFrame(msg);

    bool driverActivity =
      message->GetSignal("DBW_PrndDriverActivity")->GetResult() ? true : false;

    setOverride(OVR_GEAR, driverActivity, false);
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
      err_msg = err_msg + " Please adjust the shift lever.";
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), m_clock, CLOCK_1_SEC, err_msg.c_str());
    }

    pub_gear_->publish(out);
  }
}

void RaptorDbwCAN::recvWheelSpeedRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvWheelPositionRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvTirePressureRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvSurroundRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvVinRpt(const Frame::SharedPtr msg)
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
      String msg; msg.data = vin_;
      pub_vin_->publish(msg);
    }
  }
}

void RaptorDbwCAN::recvImuRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_IMU_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    m_imu_rpt.header.stamp = msg->header.stamp;
    m_imu_rpt.header.frame_id = frame_id_;

    // We don't use the orientation variable
    m_imu_rpt.orientation.w = 0.0F;
    m_imu_rpt.orientation.x = 0.0F;
    m_imu_rpt.orientation.y = 0.0F;
    m_imu_rpt.orientation.z = 0.0F;

    // We don't use the covariance variables
    m_imu_rpt.orientation_covariance[0] = -1.0F;
    m_imu_rpt.angular_velocity_covariance[0] = 0.0F;
    m_imu_rpt.linear_acceleration_covariance[0] = 0.0F;

    // ROS spec for their IMU message specifies angular velocity = rads/sec
    // J1939 message spec requires deg/sec
    m_imu_rpt.angular_velocity.z =
      static_cast<double>(message->GetSignal("DBW_ImuYawRate")->GetResult()) *
      DEG_TO_RAD;

    m_imu_rpt.linear_acceleration.x =
      static_cast<double>(message->GetSignal("DBW_ImuAccelX")->GetResult());
    m_imu_rpt.linear_acceleration.y =
      static_cast<double>(message->GetSignal("DBW_ImuAccelY")->GetResult());
    m_imu_rpt.linear_acceleration.z =
      static_cast<double>(message->GetSignal("DBW_ImuAccelZ")->GetResult());

    if (m_seen_imu2_rpt) {
      m_seen_imu2_rpt = false;
      pub_imu_->publish(m_imu_rpt);
    }
  }
}

void RaptorDbwCAN::recvImu2Rpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_IMU_2_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    // ROS spec for their IMU message specifies angular velocity = rads/sec
    // J1939 message spec requires deg/sec
    m_imu_rpt.angular_velocity.x =
      static_cast<double>(message->GetSignal("DBW_ImuRollRate")->GetResult()) *
      DEG_TO_RAD;
    m_imu_rpt.angular_velocity.y =
      static_cast<double>(message->GetSignal("DBW_ImuPitchRate")->GetResult()) *
      DEG_TO_RAD;

    m_seen_imu2_rpt = true;
  }
}

void RaptorDbwCAN::recvDriverInputRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvMiscRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvLowVoltageSystemRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvBrake2Rpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvSteering2Rpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvFaultActionRpt(const Frame::SharedPtr msg)
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
    faultActionsReport.last_active_fault_idx =
      message->GetSignal("DBW_IdxOfLastActiveFault")->GetResult();
    faultActionsReport.estop_btn_pressed =
      message->GetSignal("DBW_EmgrStopBtnPrssd")->GetResult();
    faultActionsReport.remote_estop_btn_pressed.value =
      message->GetSignal("DBW_RemoteEmgrStopBtnPrssd")->GetResult();

    pub_fault_actions_report_->publish(faultActionsReport);
  }
}

void RaptorDbwCAN::recvOtherActuatorsRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvGpsReferenceRpt(const Frame::SharedPtr msg)
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

    out.ref_heading = message->GetSignal(
      "Dbw_GpsHeading")->GetResult();

    pub_gps_reference_report_->publish(out);
  }
}

void RaptorDbwCAN::recvGpsRemainderRpt(const Frame::SharedPtr msg)
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

void RaptorDbwCAN::recvExitRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_EXIT_REPORT);

  if (msg->dlc >= message->GetDlc()) {
    message->SetFrame(msg);

    ExitReport out;
    out.header.stamp = msg->header.stamp;

    out.akit_disable = message->GetSignal(
      "DBW_Exit_AKitDsbl")->GetResult();

    out.driver_in_control = message->GetSignal(
      "DBW_Exit_DrvInCtrl")->GetResult();

    out.idx_auton_disable_no_brakes = message->GetSignal(
      "DBW_Exit_AutonDsblNoBrakes")->GetResult();

    out.idx_auton_disable_apply_brakes = message->GetSignal(
      "DBW_Exit_AutonDsblAppyBrakes")->GetResult();

    out.auton_counter = message->GetSignal(
      "DBW_Exit_Cntr")->GetResult();

    pub_exit_report_->publish(out);
  }
}

void RaptorDbwCAN::recvBrakeCmd(const BrakeCmd::SharedPtr msg)
{
  // TODO(NERaptor): add checksum support
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

  Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvAcceleratorPedalCmd(
  const AcceleratorPedalCmd::SharedPtr msg)
{
  // TODO(NERaptor): add checksum support
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
    ignores_[IGNORE_ACCEL] = true;
  } else {
    ignores_[IGNORE_ACCEL] = false;
  }

  Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvSteeringCmd(const SteeringCmd::SharedPtr msg)
{
  // TODO(NERaptor): add checksum support
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
        -1.0F * max_steer_angle_,
        std::min(
          max_steer_angle_ * 1.0F, static_cast<float>(
            msg->angle_cmd * 1.0F)));
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
            std::roundf(std::fabs(msg->angle_velocity) / 2.0F))));

      message->GetSignal("AKit_SteeringWhlAngleVelocityLim")->SetResult(vcmd);
    }
    if (msg->enable) {
      message->GetSignal("AKit_SteerCtrlEnblReq")->SetResult(1);
    }
  }

  if (msg->ignore) {
    message->GetSignal("AKit_SteeringWhlIgnoreDriverOvrd")->SetResult(1);
    ignores_[IGNORE_STEER] = true;
  } else {
    ignores_[IGNORE_STEER] = false;
  }

  message->GetSignal("AKit_SteerRollingCntr")->SetResult(msg->rolling_counter);

  Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvGearCmd(const GearCmd::SharedPtr msg)
{
  // TODO(NERaptor): add checksum support
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

  Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvGlobalEnableCmd(const GlobalEnableCmd::SharedPtr msg)
{
  // TODO(NERaptor): add checksum support
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

  Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvImuCmd(const Imu::SharedPtr msg)
{
  NewEagle::DbcMessage * msg_accel = dbwDbc_.GetMessage("AKit_ACCS");
  NewEagle::DbcMessage * msg_rotate = dbwDbc_.GetMessage("AKit_ARI");

  msg_accel->GetSignal("VerticalAccelerationExRange")->SetResult(0);
  msg_accel->GetSignal("LateralAccelerationExRange")->SetResult(0);
  msg_accel->GetSignal("LongitudinalAccelerationExRange")->SetResult(0);

  msg_rotate->GetSignal("AngularRateMeasurementLatency")->SetResult(0);
  msg_rotate->GetSignal("YawRateExRange")->SetResult(0);
  msg_rotate->GetSignal("RollRateExRange")->SetResult(0);
  msg_rotate->GetSignal("PitchRateExRange")->SetResult(0);

  if (enabled()) {
    msg_accel->GetSignal("VerticalAccelerationExRange")->SetResult(
      msg->linear_acceleration.z);
    msg_accel->GetSignal("LateralAccelerationExRange")->SetResult(msg->linear_acceleration.y);
    msg_accel->GetSignal("LongitudinalAccelerationExRange")->SetResult(
      msg->linear_acceleration.x);

    msg_rotate->GetSignal("YawRateExRange")->SetResult(msg->angular_velocity.z * -1);
    msg_rotate->GetSignal("RollRateExRange")->SetResult(msg->angular_velocity.x);
    msg_rotate->GetSignal("PitchRateExRange")->SetResult(msg->angular_velocity.y * -1);

    // Calculate latency
    rclcpp::Time curr_time = m_clock.now();
    double latency_msec = 0.0F;

    if (curr_time > msg->header.stamp) { // Check if header time is valid
      rclcpp::Duration latency_nsec = curr_time - msg->header.stamp;
      latency_msec = latency_nsec.nanoseconds() / NSEC_TO_MSEC;
    }

    msg_rotate->GetSignal("AngularRateMeasurementLatency")->SetResult(latency_msec);

    // Update timestamp for timeout check
    t_stamp_last_imu_cmd = curr_time;
  }

  // Publish both messages
  Frame frame1 = msg_accel->GetFrame();
  pub_can_->publish(frame1);

  Frame frame2 = msg_rotate->GetFrame();
  pub_can_->publish(frame2);
}

void RaptorDbwCAN::recvMiscCmd(const MiscCmd::SharedPtr msg)
{
  // TODO(NERaptor): add checksum support
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

  Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

/// \brief DBW Enabled needs to publish when its state changes.
/// \returns TRUE when DBW enable state changes, FALSE otherwise
bool RaptorDbwCAN::publishDbwEnabled()
{
  bool change = false;
  bool en = enabled();
  if (enables_[EN_DBW_PREV] != en) {
    Bool msg;
    msg.data = en;
    pub_sys_enable_->publish(msg);
    change = true;
  }
  enables_[EN_DBW_PREV] = en;
  return change;
}

void RaptorDbwCAN::timer_10ms_Callback()
{
  // Check for timeouts
  if (enabled()) {
    rclcpp::Time curr_time = m_clock.now();

    // Check if IMU command has timed out
    rclcpp::Duration timeout_nsec = curr_time - t_stamp_last_imu_cmd;

    if ((timeout_nsec.nanoseconds() / NSEC_TO_MSEC) > IMU_TIMEOUT_MSEC) {
      double accel_err_val = applyScaling(0xFFFFu, 0.01F, -320.0F);
      double ang_rate_err_val = applyScaling(0xFFFFu, 0.0078125F, -250.0F);
      double latency_err_val = applyScaling(0xFFu, 0.5F, 0.0F);

      NewEagle::DbcMessage * msg_accel = dbwDbc_.GetMessage("AKit_ACCS");
      NewEagle::DbcMessage * msg_rotate = dbwDbc_.GetMessage("AKit_ARI");

      msg_accel->GetSignal("VerticalAccelerationExRange")->SetResult(accel_err_val);
      msg_accel->GetSignal("LateralAccelerationExRange")->SetResult(accel_err_val);
      msg_accel->GetSignal("LongitudinalAccelerationExRange")->SetResult(accel_err_val);

      msg_rotate->GetSignal("AngularRateMeasurementLatency")->SetResult(latency_err_val);
      msg_rotate->GetSignal("YawRateExRange")->SetResult(ang_rate_err_val);
      msg_rotate->GetSignal("RollRateExRange")->SetResult(ang_rate_err_val);
      msg_rotate->GetSignal("PitchRateExRange")->SetResult(ang_rate_err_val);

      // Publish both messages
      pub_can_->publish(msg_accel->GetFrame());
      pub_can_->publish(msg_rotate->GetFrame());
    }
  }
}

void RaptorDbwCAN::timer_200ms_Callback()
{
  // Check for driver overrides
  if (clear()) {
    Frame out;
    out.is_extended = false;

    if (overrides_[OVR_BRAKE]) {
      // Might have an issue with WatchdogCntr when these are set.
      NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_BrakeRequest");
      message->GetSignal("AKit_BrakePedalReq")->SetResult(0);
      message->GetSignal("AKit_BrakeCtrlEnblReq")->SetResult(0);
      // message->GetSignal("AKit_BrakePedalCtrlMode")->SetResult(0);
      pub_can_->publish(message->GetFrame());
    }

    if (overrides_[OVR_ACCEL] && !ignores_[IGNORE_ACCEL]) {
      // Might have an issue with WatchdogCntr when these are set.
      NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_AccelPdlRequest");
      message->GetSignal("AKit_AccelPdlReq")->SetResult(0);
      message->GetSignal("AKit_AccelPdlEnblReq")->SetResult(0);
      message->GetSignal("Akit_AccelPdlIgnoreDriverOvrd")->SetResult(0);
      // message->GetSignal("AKit_AccelPdlCtrlMode")->SetResult(0);
      pub_can_->publish(message->GetFrame());
    }

    if (overrides_[OVR_STEER] && !ignores_[IGNORE_STEER]) {
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

    if (overrides_[OVR_GEAR]) {
      NewEagle::DbcMessage * message = dbwDbc_.GetMessage("AKit_GearRequest");
      message->GetSignal("AKit_PrndStateCmd")->SetResult(0);
      message->GetSignal("AKit_PrndChecksum")->SetResult(0);
      pub_can_->publish(message->GetFrame());
    }
  }
}

void RaptorDbwCAN::enableSystem()
{
  if (!enables_[EN_DBW]) {
    if (fault()) {
      int i{0};
      for (i = FAULT_ACCEL; i < NUM_SERIOUS_FAULTS; i++) {
        if (faults_[i]) {
          std::string err_msg("DBW system disabled - ");
          err_msg = err_msg + FAULT_SYSTEM[i];
          err_msg = err_msg + " fault.";
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), m_clock, CLOCK_1_SEC, err_msg.c_str());
        }
      }
    } else {
      enables_[EN_DBW] = true;
      if (publishDbwEnabled()) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), m_clock, CLOCK_1_SEC,
          "DBW system enabled.");
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), m_clock, CLOCK_1_SEC,
          "DBW system failed to enable. Check driver overrides.");
      }
    }
  }
}

void RaptorDbwCAN::disableSystem()
{
  if (enables_[EN_DBW]) {
    enables_[EN_DBW] = false;
    publishDbwEnabled();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), m_clock, CLOCK_1_SEC,
      "DBW system disabled - system disabled.");
  }
}

void RaptorDbwCAN::setOverride(ListOverrides which_ovr, bool override, bool ignore)
{
  if (which_ovr < NUM_OVERRIDES) {
    bool en = enabled();
    if (override && en && !ignore) {
      enables_[EN_DBW] = false;
    }
    overrides_[which_ovr] = override;
    if (publishDbwEnabled()) {
      if (en && !ignore) {
        std::string err_msg("DBW system disabled - ");
        err_msg = err_msg + OVR_SYSTEM[which_ovr];
        err_msg = err_msg + " override";
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), m_clock, CLOCK_1_SEC, err_msg.c_str());
      } else if (en && ignore) {
        std::string err_msg("DBW system enabled - ");
        err_msg = err_msg + OVR_SYSTEM[which_ovr];
        err_msg = err_msg + " ignored";
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), m_clock, CLOCK_1_SEC, err_msg.c_str());
      } else {
        std::string err_msg("DBW system enabled - no ");
        err_msg = err_msg + OVR_SYSTEM[which_ovr];
        err_msg = err_msg + " override";
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), m_clock, CLOCK_1_SEC, err_msg.c_str());
      }
    }
  }
}

void RaptorDbwCAN::setFault(ListFaults which_fault, bool fault)
{
  if (which_fault < NUM_SERIOUS_FAULTS) {
    bool en = enabled();
    if (fault && en) {
      enables_[EN_DBW] = false;
    }
    faults_[which_fault] = fault;
    if (publishDbwEnabled()) {
      if (en) {
        std::string err_msg("DBW system disabled - ");
        err_msg = err_msg + FAULT_SYSTEM[which_fault];
        err_msg = err_msg + " fault.";
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), m_clock, CLOCK_1_SEC, err_msg.c_str());
      } else {
        std::string err_msg("DBW system enabled - no ");
        err_msg = err_msg + FAULT_SYSTEM[which_fault];
        err_msg = err_msg + " fault.";
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), m_clock, CLOCK_1_SEC, err_msg.c_str());
      }
    }
  }
}

void RaptorDbwCAN::faultWatchdog(bool fault, uint8_t src, bool braking)
{
  setFault(FAULT_WATCH, fault);

  if (braking && !faults_[FAULT_WATCH_BRAKES]) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), m_clock, CLOCK_1_SEC,
      "Watchdog - new braking fault.");
  } else if (!braking && faults_[FAULT_WATCH_BRAKES]) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), m_clock, CLOCK_1_SEC,
      "Watchdog - braking fault is cleared.");
  } else {}

  if (fault && src && !faults_[FAULT_WATCH_WARN]) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), m_clock, CLOCK_1_SEC,
      "Watchdog - new fault warning.");
    faults_[FAULT_WATCH_WARN] = true;
  } else if (!fault) {
    faults_[FAULT_WATCH_WARN] = false;
  } else {}

  faults_[FAULT_WATCH_BRAKES] = braking;
  if (fault && !faults_[FAULT_WATCH_BRAKES] && faults_[FAULT_WATCH_WARN]) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), m_clock, CLOCK_1_SEC,
      "Watchdog - new non-braking fault.");
  }
}

void RaptorDbwCAN::faultWatchdog(bool fault, uint8_t src)
{
  faultWatchdog(fault, src, faults_[FAULT_WATCH_BRAKES]);   // No change to 'using brakes' status
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

double RaptorDbwCAN::applyScaling(uint32_t in_val, double gain, double offset)
{
  double out_val = static_cast<double>(in_val);
  out_val = (out_val * gain) + offset;
  return out_val;
}

}  // namespace raptor_dbw_can
