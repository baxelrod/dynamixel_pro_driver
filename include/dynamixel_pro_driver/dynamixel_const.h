/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage
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
 *   * Neither the name of Willow Garage nor the names of its
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

#ifndef DYNAMIXEL_CONST_H__
#define DYNAMIXEL_CONST_H__

#include <stdint.h>
#include <string>

/**
 * \brief Contains various constants used by the dynamixel pro driver
 */
namespace dynamixel_pro_driver
{
const uint16_t HEADER_SIZE = 7;
const uint16_t ERROR_INDEX = HEADER_SIZE + 1;
const uint16_t REPLY_BEGIN_INDEX = ERROR_INDEX + 1;

// Control Table Constants
typedef enum DynamixelControlEnum
{
    DXL_MODEL_NUMBER = 0,
    DXL_MODEL_INFO=2,
    DXL_FIRMWARE_VERSION = 6,
    DXL_ID = 7,
    DXL_BAUD_RATE = 8,
    DXL_RETURN_DELAY_TIME = 9,
    DXL_MAX_ANGLE_LIMIT = 36,
    DXL_MIN_ANGLE_LIMIT = 40,
    DXL_DRIVE_MODE = 11,
    DXL_LIMIT_TEMPERATURE = 21,
    DXL_DOWN_LIMIT_VOLTAGE = 24,
    DXL_UP_LIMIT_VOLTAGE = 22,
    DXL_ACCEL_LIMIT = 26,
    DXL_MAX_TORQUE = 30,
    DXL_RETURN_LEVEL = 16,
    DXL_OPERATING_MODE = 19,
    DXL_DOWN_CALIBRATION_L = 20,
    DXL_DOWN_CALIBRATION_H = 21,
    DXL_UP_CALIBRATION_L = 22,
    DXL_UP_CALIBRATION_H = 23,
    DXL_TORQUE_ENABLE = 562,
    DXL_LED = 25,
    DXL_CW_COMPLIANCE_MARGIN = 26,
    DXL_CCW_COMPLIANCE_MARGIN = 27,
    DXL_CW_COMPLIANCE_SLOPE = 28,
    DXL_CCW_COMPLIANCE_SLOPE = 29,
    DXL_GOAL_POSITION = 596,
    DXL_GOAL_POSITION_H = 31,
    DXL_GOAL_SPEED = 600,
    DXL_GOAL_SPEED_H = 33,
    DXL_PRESENT_POSITION = 611,
    DXL_PRESENT_POSITION_H = 37,
    DXL_PRESENT_SPEED = 615,
    DXL_PRESENT_SPEED_H = 39,
    DXL_PRESENT_CURRENT = 621,
    DXL_PRESENT_VOLTAGE = 623,
    DXL_PRESENT_TEMPERATURE = 625,
    DXL_REGISTERED_INSTRUCTION = 44,
    DXL_PAUSE_TIME = 45,
    DXL_MOVING = 46,
    DXL_LOCK = 47,
    DXL_PUNCH_L = 48,
    DXL_PUNCH_H = 49,
    DXL_SENSED_CURRENT_L = 56,
    DXL_SENSED_CURRENT_H = 57,

} DynamixelControl;

const uint16_t DXL_MODEL_NUMBER_SIZE = 4;
const uint16_t DXL_POSITION_LIMIT_SIZE = 4;


typedef enum DynamixelInstructionEnum
{
    DXL_PING = 1,
    DXL_READ_DATA = 2,
    DXL_WRITE_DATA = 3,
    DXL_REG_WRITE = 4,
    DXL_ACTION = 5,
    DXL_RESET = 6,
    DXL_SYNC_WRITE = 0x83,
    DXL_BROADCAST = 254,

} DynamixelInstruction;

typedef enum DynamixelErrorCodeEnum
{
    DXL_UNDOCUMENTED_ERROR2 = 64,
    DXL_UNDOCUMENTED_ERROR1 = 32,
    DXL_ELECTRIC_SHOCK_ERROR = 16,
    DXL_ENCODER_ERROR = 8,
    DXL_OVERHEATING_ERROR = 4,
    DXL_HALL_ERROR = 2,
    DXL_INPUT_VOLTAGE_ERROR = 1,
    DXL_NO_ERROR = 0,

} DynamixelErrorCode;

}

#endif  // DYNAMIXEL_CONST_H__
