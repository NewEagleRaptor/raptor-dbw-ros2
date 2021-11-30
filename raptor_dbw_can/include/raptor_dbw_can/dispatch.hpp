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

#ifndef RAPTOR_DBW_CAN__DISPATCH_HPP_
#define RAPTOR_DBW_CAN__DISPATCH_HPP_

#include <stdint.h>

namespace raptor_dbw_can
{
/** \brief Enumeration of VIN multiplex control values */
typedef enum
{
  VIN_MUX_VIN0  = 0x00,   /**< VIN mux value = 0 */
  VIN_MUX_VIN1  = 0x01,   /**< VIN mux value = 1 */
  VIN_MUX_VIN2  = 0x02,   /**< VIN mux value = 2 */
} VinMux;

/** \brief Enumeration of wheel speed multiplex control values */
typedef enum
{
  WHEEL_SPEED_MUX0  = 0x00,   /**< Wheel speed mux value = 0 */
  WHEEL_SPEED_MUX1  = 0x01,   /**< Wheel speed mux value = 1 */
  WHEEL_SPEED_MUX2  = 0x02,   /**< Wheel speed mux value = 2 */
} WheelSpeedMux;

#undef BUILD_ASSERT

/** \brief Enumeration of CAN message IDs */
enum ListMessageIDs
{
  ID_BRAKE_CMD                  = 0x2F04,   /**< Brake command ID */
  ID_BRAKE_REPORT               = 0x1F04,   /**< Brake report ID */
  ID_ACCELERATOR_PEDAL_CMD      = 0x2F01,   /**< Accelerator pedal command ID */
  ID_ACCEL_PEDAL_REPORT         = 0x1F02,   /**< Accelerator pedal report ID */
  ID_STEERING_CMD               = 0x2F03,   /**< Steering command ID */
  ID_STEERING_REPORT            = 0x1F03,   /**< Steering report ID */
  ID_GEAR_CMD                   = 0x2F05,   /**< PRND gear command ID */
  ID_GEAR_REPORT                = 0x1F05,   /**< PRND gear report ID */
  ID_REPORT_WHEEL_SPEED         = 0x1F0B,   /**< Wheel speed report ID */
  ID_REPORT_IMU                 = 0x1F0A,   /**< IMU report ID */
  ID_REPORT_TIRE_PRESSURE       = 0x1f07,   /**< Tire pressure report ID */
  ID_REPORT_SURROUND            = 0x1f10,   /**< Surround report ID */
  ID_VIN                        = 0x1F08,   /**< VIN report ID */
  ID_REPORT_DRIVER_INPUT        = 0x1F0F,   /**< Driver input report ID */
  ID_REPORT_WHEEL_POSITION      = 0x1F06,   /**< Wheel position report ID */
  ID_MISC_REPORT                = 0x1F01,   /**< Misc. report ID */
  ID_LOW_VOLTAGE_SYSTEM_REPORT  = 0x1F11,   /**< Low voltage system report ID */
  ID_BRAKE_2_REPORT             = 0x1F12,   /**< Brake2 report ID */
  ID_STEERING_2_REPORT          = 0x1F13,   /**< Steering2 report ID */
  ID_OTHER_ACTUATORS_REPORT     = 0x1F14,   /**< Other actuators report ID */
  ID_FAULT_ACTION_REPORT        = 0x1F15,   /**< Fault action report ID */
  ID_GPS_REFERENCE_REPORT       = 0x1F16,   /**< GPS reference report ID */
  ID_GPS_REMAINDER_REPORT       = 0x1F17,   /**< GPS remainder report ID */
  ID_EXIT_REPORT                = 0x1F24,   /**< Exit report ID */
  ID_HMI_GLOBAL_ENABLE_REPORT   = 0x3f01,   /**< HMI global enable report ID */
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DISPATCH_HPP_
