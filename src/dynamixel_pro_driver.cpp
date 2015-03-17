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

#include <time.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

#include <sstream>

#include <ros/ros.h>

#include <dynamixel_pro_driver/dynamixel_const.h>
#include <dynamixel_pro_driver/dynamixel_pro_driver.h>


#define LOBYTE(w) ((uint8_t)(w))
#define HIBYTE(w) ((uint8_t)(((uint16_t)(w) >> 8) & 0xFF))
#define MAKEWORD(low, high) \
((uint16_t)((((uint16_t)(high)) << 8) | ((uint8_t)(low))))

#define PKT_LENGTH_L 5
#define PKT_LENGTH_H 6

using namespace std;



/*******************  IMPORTANT This code was written for little-endian cpus (forex intel)   ****************/
namespace dynamixel_pro_driver
{

DynamixelProDriver::DynamixelProDriver(const std::string &device,
                         const std::string &baud, uint32_t timeout_ms) :
    read_error_count(0),
    read_count(0),
    last_reset_sec(0.0),
    port_(new serial::Serial(device, atoi(baud.c_str()),
        serial::Timeout::simpleTimeout(timeout_ms))),
    serial_mutex_()
{
}

DynamixelProDriver::~DynamixelProDriver()
{
    port_->close();
    delete port_;
    pthread_mutex_destroy(&serial_mutex_);
}

bool DynamixelProDriver::ping(int servo_id)
{
    // Instruction, crcx2
    uint8_t length = 3;

    // packet: FF  FF FD 00 ID LEN_L LEN_H INSTRUCTION PARAM_1 ... CRC_L CRC_H
    uint8_t packet_length = 3;// length of instruction, params and data
    uint8_t packet[] = { 0xFF, 0xFF, 0xFD, 0x00,  servo_id, LOBYTE(length), HIBYTE(length), DXL_PING, 0x00, 0x00};

    packet_length += 7; //header size

    std::vector<uint8_t> response;

    pthread_mutex_lock(&serial_mutex_);
    bool success = writePacket(packet);
    if (success) { success = readResponse(response); }
    pthread_mutex_unlock(&serial_mutex_);

    return success;
}

bool DynamixelProDriver::getModelNumber(int servo_id, uint16_t& model_number)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_MODEL_NUMBER, DXL_MODEL_NUMBER_SIZE, response))
    {
        model_number = MAKEWORD(response[HEADER_SIZE + 2],  response[HEADER_SIZE + 3]);
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getModelInfo(int servo_id, uint32_t& model_info)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_MODEL_INFO, 4, response))
    {
        model_info = *((uint32_t *) (&response[REPLY_BEGIN_INDEX]));
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getFirmwareVersion(int servo_id, uint8_t& firmware_version)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_FIRMWARE_VERSION, 1, response))
    {
        firmware_version = response[HEADER_SIZE + 2];
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getBaudRate(int servo_id, uint8_t& baud_rate)
{
    std::vector<uint8_t> response;
    if (read(servo_id, DXL_BAUD_RATE, 1, response))
    {
        baud_rate = response[HEADER_SIZE + 2];
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getReturnDelayTime(int servo_id, uint8_t& return_delay_time)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_RETURN_DELAY_TIME, 1, response))
    {
        return_delay_time = response[HEADER_SIZE + 2];
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getOperatingMode(int servo_id, uint8_t& op_mode)
{
    std::vector<uint8_t> response;
    if (read(servo_id, DXL_DRIVE_MODE, 1, response))
    {
        op_mode = response[HEADER_SIZE + 2];
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getAngleLimits(int servo_id, uint32_t& min_angle_limit, uint32_t& max_angle_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_MAX_ANGLE_LIMIT, 2 * DXL_POSITION_LIMIT_SIZE, response))
    {
        max_angle_limit = *((uint32_t *) (&response[REPLY_BEGIN_INDEX]));
        min_angle_limit = *((uint32_t *) (&response[REPLY_BEGIN_INDEX + DXL_POSITION_LIMIT_SIZE]));
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getMaxAngleLimit(int servo_id, uint32_t& max_angle)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_MAX_ANGLE_LIMIT, DXL_POSITION_LIMIT_SIZE, response))
    {
        max_angle = *((uint32_t *) (&response[HEADER_SIZE + 2]));
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getMinAngleLimit(int servo_id, uint32_t& min_angle)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_MIN_ANGLE_LIMIT, DXL_POSITION_LIMIT_SIZE, response))
    {
        min_angle = *((uint32_t *) (&response[HEADER_SIZE + 2]));
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getVoltageLimits(int servo_id, float& min_voltage_limit, float& max_voltage_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_DOWN_LIMIT_VOLTAGE, 4, response))
    {
        min_voltage_limit = *((uint16_t*) &response[REPLY_BEGIN_INDEX]) / 10.0;
        max_voltage_limit = *((uint16_t*) &response[REPLY_BEGIN_INDEX + 2]) / 10.0;
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getMinVoltageLimit(int servo_id, float& min_voltage_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_DOWN_LIMIT_VOLTAGE, 2, response))
    {
        min_voltage_limit = *((uint16_t*) &response[REPLY_BEGIN_INDEX]) / 10.0;
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getMaxVoltageLimit(int servo_id, float& max_voltage_limit)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_UP_LIMIT_VOLTAGE, 2, response))
    {
        max_voltage_limit = *((uint16_t*) &response[REPLY_BEGIN_INDEX]) / 10.0;
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getTemperatureLimit(int servo_id, uint8_t& max_temperature)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_LIMIT_TEMPERATURE, 1, response))
    {
        max_temperature = response[REPLY_BEGIN_INDEX];
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getMaxTorque(int servo_id, uint16_t& max_torque)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_MAX_TORQUE, 2, response))
    {
        max_torque = response[REPLY_BEGIN_INDEX];
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getTorqueEnabled(int servo_id, bool& torque_enabled)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_TORQUE_ENABLE, 1, response))
    {
        torque_enabled = response[REPLY_BEGIN_INDEX];
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getTargetPosition(int servo_id, int32_t& target_position)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_GOAL_POSITION, 4, response))
    {
        target_position = *((int32_t*) &response[REPLY_BEGIN_INDEX]);
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getTargetVelocity(int servo_id, int32_t& target_velocity)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_GOAL_SPEED, 4, response))
    {
        target_velocity = *((int32_t*) &response[REPLY_BEGIN_INDEX]);
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getPosition(int servo_id, int32_t& position)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_POSITION, 4, response))
    {
        position = *((int32_t *)(&response[REPLY_BEGIN_INDEX]));
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getVelocity(int servo_id, int32_t& velocity)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_SPEED, 4, response))
    {
        velocity = *((int32_t *)(&response[REPLY_BEGIN_INDEX]));
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getCurrent(int servo_id, uint16_t& current)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_CURRENT, 2, response))
    {
        current = *((uint16_t *)(&response[REPLY_BEGIN_INDEX]));
        if (!validateNoErrors(servo_id, response[ERROR_INDEX], "getCurrent"))
            return false;

        if (current < 65000) //returns invalid readings sometimes
            return true;
        else
            return false;
    }

    return false;
}

bool DynamixelProDriver::getVoltage(int servo_id, float& voltage)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_VOLTAGE, 2, response))
    {
        uint16_t voltage_t = *((uint16_t *)(&response[REPLY_BEGIN_INDEX]));
        voltage = voltage_t / 10.0;
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

bool DynamixelProDriver::getTemperature(int servo_id, uint8_t& temperature)
{
    std::vector<uint8_t> response;

    if (read(servo_id, DXL_PRESENT_TEMPERATURE, 1, response))
    {
        temperature = *((uint8_t *)(&response[REPLY_BEGIN_INDEX]));
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);
    }

    return false;
}

/************************ SETTERS **************************/

bool DynamixelProDriver::setId(int servo_id, uint8_t id)
{
    std::vector<uint8_t> data;
    data.push_back(id);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_ID, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setBaudRate(int servo_id, uint8_t baud_rate)
{
    std::vector<uint8_t> data;
    data.push_back(baud_rate);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_BAUD_RATE, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setReturnDelayTime(int servo_id, uint8_t return_delay_time)
{
    std::vector<uint8_t> data;
    data.push_back(return_delay_time);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_RETURN_DELAY_TIME, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setOperatingMode(int servo_id, uint8_t op_mode)
{
    std::vector<uint8_t> data;
    data.push_back(op_mode);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_DRIVE_MODE, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK_PROTECTED(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setAngleLimits(int servo_id, int32_t min_angle, int32_t max_angle)
{
    std::vector<uint8_t> data;

    //expand the vector to be the right size
    for (int i = 0; i < 8; i++)
        data.push_back(0);

    *((uint32_t *)&data[0])= max_angle;
    *((uint32_t *)&data[4])= min_angle;


    std::vector<uint8_t> response;

    if (write(servo_id, DXL_MAX_ANGLE_LIMIT, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK_PROTECTED(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setMaxAngleLimit(int servo_id, int32_t max_angle)
{
    std::vector<uint8_t> data;

    //expand the vector to be the right size
    for (int i = 0; i < 4; i++)
        data.push_back(0);
    *((uint32_t *)&data[0])= max_angle;

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_MAX_ANGLE_LIMIT, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK_PROTECTED(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setMinAngleLimit(int servo_id, int32_t min_angle)
{
    std::vector<uint8_t> data;

    //expand the vector to be the right size
    for (int i = 0; i < 4; i++)
        data.push_back(0);
    *((uint32_t *)&data[0])= min_angle;

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_MIN_ANGLE_LIMIT, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK_PROTECTED(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setTemperatureLimit(int servo_id, uint8_t max_temperature)
{
    std::vector<uint8_t> data;
    data.push_back(max_temperature);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_LIMIT_TEMPERATURE, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK_PROTECTED(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setMaxTorque(int servo_id, uint16_t max_torque)
{
    std::vector<uint8_t> data;

    //expand the vector to be the right size
    for (int i = 0; i < 2; i++)
        data.push_back(0);

    *((uint16_t *)&data[0]) = max_torque;

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_MAX_TORQUE, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK_PROTECTED(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setTorqueEnabled(int servo_id, bool on)
{
    std::vector<uint8_t> data;
    data.push_back(on);

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_TORQUE_ENABLE, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setPosition(int servo_id, uint32_t position)
{
    std::vector<uint8_t> data;

    //expand the vector to be the right size
    for (int i = 0; i < 4; i++)
        data.push_back(0);
    *((uint32_t *)&data[0])=position;

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_GOAL_POSITION, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);

    return false;
}


bool DynamixelProDriver::setVelocity(int servo_id, int32_t velocity)
{
    std::vector<uint8_t> data;

    //expand the vector to be the right size
    for (int i = 0; i < 4; i++)
        data.push_back(0);
    *((uint32_t *)&data[0])=velocity;

    std::vector<uint8_t> response;

    if (write(servo_id, DXL_GOAL_SPEED, data, response))
        return DMX_PRO_DRIVER_ERROR_CHECK(servo_id, response[ERROR_INDEX]);

    return false;
}

bool DynamixelProDriver::setMultiPosition(const std::vector<std::vector<int> > &value_pairs)
{
    std::vector<std::vector<uint8_t> > data;

    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        uint8_t motor_id = value_pairs[i][0];
        int32_t position = value_pairs[i][1];

        std::vector<uint8_t> value_pair;
        value_pair.push_back(motor_id);                 // servo id

        //expand the vector to be the right size
        for (int i = 0; i < 4; i++)
            value_pair.push_back(0);

        *((int32_t * ) &value_pair[1]) = position;

        data.push_back(value_pair);
    }

    if (syncWrite(DXL_GOAL_POSITION, data))
        return true;
    else
        return false;
}

bool DynamixelProDriver::setMultiVelocity(const std::vector<std::vector<int> > &value_pairs)
{
    std::vector<std::vector<uint8_t> > data;

    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        int motor_id = value_pairs[i][0];
        int32_t velocity = value_pairs[i][1];

        std::vector<uint8_t> value_pair;
        value_pair.push_back(motor_id);             // servo id

        //expand the vector to be the right size
        for (int i = 0; i < 4; i++)
            value_pair.push_back(0);

        *((int32_t * ) &value_pair[1]) = velocity;

        data.push_back(value_pair);
    }

    if (syncWrite(DXL_GOAL_SPEED, data))
        return true;
    else
        return false;
}

bool DynamixelProDriver::setMultiPositionVelocity(const std::vector<std::vector<int> > &value_tuples)
{
    std::vector<std::vector<uint8_t> > data;

    for (size_t i = 0; i < value_tuples.size(); ++i)
    {
        uint8_t motor_id = value_tuples[i][0];
        int32_t position = value_tuples[i][1];
        int32_t velocity = value_tuples[i][2];

        std::vector<uint8_t> vals;

        vals.push_back(motor_id);     // servo id

        //expand the vector to be the right size
        for (int i = 0; i < 8; i++)
            vals.push_back(0);

        *((int32_t * ) &vals[1]) = position;
        *((int32_t * ) &vals[5]) = velocity;

        data.push_back(vals);
    }

    if (syncWrite(DXL_GOAL_POSITION, data))
        return true;
    else
        return false;
}

bool DynamixelProDriver::setMultiTorqueEnabled(const std::vector<std::vector<int> > &value_pairs)
{
    std::vector<std::vector<uint8_t> > data;

    cout << "num servos to enable " << value_pairs.size() << endl;
    for (size_t i = 0; i < value_pairs.size(); ++i)
    {
        int motor_id = value_pairs[i][0];
        bool torque_enabled = value_pairs[i][1];

        std::vector<uint8_t> value_pair;
        value_pair.push_back(motor_id);         // servo id
        value_pair.push_back(torque_enabled);   // torque_enabled

        data.push_back(value_pair);
    }

    if (syncWrite(DXL_TORQUE_ENABLE, data))
        return true;
    else
        return false;
}

bool DynamixelProDriver::validateNoErrorsProtected(int servo_id, uint8_t error_code, const std::string &method_name)
{
  if (validateNoErrors(servo_id, error_code, method_name))
  {
      return true;
  }
  else
  {
    if ((error_code & DXL_UNDOCUMENTED_ERROR2) != 0)
    {
      bool torque_enabled;
      getTorqueEnabled(servo_id, torque_enabled);
      if (torque_enabled != 0) //give a more userful error message. The error code it gives isn't documented as of now
                               // but they haven't officially released the servos either so it may be documented upon release
          cerr << "ERROR: You may not set the " << method_name << " on a servo when torque is enabled " << endl;
    }
  }
}

bool DynamixelProDriver::validateNoErrors(int servo_id, uint8_t error_code, const std::string &command_failed)
{
    if (error_code == DXL_NO_ERROR)
    {
        return true;
    }
    else
    {
        ROS_ERROR("you have a dynamixel comms error %d", (int) error_code);
    }

    std::vector<std::string> error_msgs;

    if ((error_code & DXL_INPUT_VOLTAGE_ERROR) != 0) { error_msgs.push_back("INPUT_VOLTAGE_ERROR"); }
    if ((error_code & DXL_HALL_ERROR) != 0) { error_msgs.push_back("HALL_ERROR"); }
    if ((error_code & DXL_OVERHEATING_ERROR) != 0) { error_msgs.push_back("OVERHEATING_ERROR"); }
    if ((error_code & DXL_ENCODER_ERROR) != 0)   { error_msgs.push_back("ENCODER_ERROR"); }
    if ((error_code & DXL_ELECTRIC_SHOCK_ERROR) != 0)         { error_msgs.push_back("ELECTRIC_SHOCK_ERROR"); }
    if ((error_code & DXL_UNDOCUMENTED_ERROR1) != 0)      { error_msgs.push_back("UNDOCUMENTED_ERROR1"); }
    if ((error_code & DXL_UNDOCUMENTED_ERROR2) != 0)   { error_msgs.push_back("UNDOCUMENTED_ERROR2"); }

    std::stringstream m;
    m << "Detected error condition [";

    for (size_t i = 0; i < error_msgs.size(); ++i)
    {
        m << error_msgs[i] << (i != error_msgs.size()-1 ? ", " : "");
    }

    m << "] during " << command_failed << " command on servo #" << servo_id;

    ROS_ERROR("%s", m.str().c_str());

    return false;
}

bool DynamixelProDriver::read(int servo_id,
                       int address,
                       int size,
                       std::vector<uint8_t>& response)
{
    // Number of bytes following standard header (0xFF, 0xFF, 0xFD, 0x00, id, length_l, length_h)
    // in our case this is instruction, add_l, add_h, len_l, len_h, crc_l, crc_h
    uint8_t length = 7;


    // header: FF  FF FD 00  ID LEN_L  LEN_H
    uint8_t packet[] = { 0xFF, 0xFF, 0xFD, 0x00, servo_id, LOBYTE(length), HIBYTE(length), //header
        DXL_READ_DATA, LOBYTE(address), HIBYTE(address), LOBYTE(size), HIBYTE(size), 0x00, 0x0}; //content
        // instruction, address_lo,      address_hi      len_to_read_lo, len_to_read_hi, crc_l, crc_h

    pthread_mutex_lock(&serial_mutex_);
    bool success = writePacket(packet);
    if (success) { success = readResponse(response); }
    pthread_mutex_unlock(&serial_mutex_);

    return success;
}

bool DynamixelProDriver::write(int servo_id,
                        int address,
                        const std::vector<uint8_t>& data,
                        std::vector<uint8_t>& response)
{
    // Number of bytes following standard header (0xFF, 0xFF, id, length)
    // instruction, addressx2, checksumx2
    uint8_t length = 5 + data.size();

    // packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
    uint8_t packetLength = 7 + length;
    uint8_t packet[packetLength];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = servo_id;
    packet[5] = LOBYTE(length);
    packet[6] = HIBYTE(length);//header

    packet[7] = DXL_WRITE_DATA;
    packet[8] = LOBYTE(address);
    packet[9] = HIBYTE(address);

    for (unsigned int i = 0; i < data.size(); ++i)
    {
        packet[10+i] = data[i];
    }

    pthread_mutex_lock(&serial_mutex_);
    bool success = writePacket(packet);
    if (success) { success = readResponse(response); }
    pthread_mutex_unlock(&serial_mutex_);

    return success;
}

bool DynamixelProDriver::syncWrite(int address,
                            const std::vector<std::vector<uint8_t> >& data)
{
    // data = ( (id, byte1, byte2... ), (id, byte1, byte2...), ... )

    uint8_t length = 7; //instruction, addressx2, field_lengthx2, crcx2
    int field_length = data[0].size();//includes the ID. Also the fields may NOT be different sizes

    int packet_length = 7 + length + data.size() * field_length;
    uint8_t packet[packet_length];

    //field_length--;//don't count the ID


    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00; // header

    packet[4] = DXL_BROADCAST; // ID, syncWrites are broadcasted to everyone, and the servos listen if they see their ID later on

    packet[5] = LOBYTE(length + data.size() * field_length);
    packet[6] = HIBYTE(length + data.size() * field_length);//length

    field_length--;

    packet[7] = DXL_SYNC_WRITE; //instruction

    packet[8] = LOBYTE(address);
    packet[9] = HIBYTE(address);
    packet[10] = LOBYTE(field_length);
    packet[11] = HIBYTE(field_length); //now we're ready to send over the tiem

    //cout << "sync writing  to " << data.size() << " servos" << endl;
    for (size_t i = 0; i < data.size(); ++i)
    {
        for (size_t j = 0; j < data[i].size(); ++j)
        {
            packet[12+i*data[i].size()+j] = data[i][j];
        }
    }

    pthread_mutex_lock(&serial_mutex_);
    bool success = writePacket(packet);
    pthread_mutex_unlock(&serial_mutex_);

    return success;
}

bool DynamixelProDriver::waitForBytes(ssize_t n_bytes, uint16_t timeout_ms)
{
    struct timespec ts_now;
    clock_gettime(CLOCK_REALTIME, &ts_now);
    double start_time_ms = ts_now.tv_sec * 1.0e3 + ts_now.tv_nsec / 1.0e6;

    // wait for response packet from the motor
    while (port_->available() < n_bytes)
    {
        clock_gettime(CLOCK_REALTIME, &ts_now);
        double current_time_ms = ts_now.tv_sec * 1.0e3 + ts_now.tv_nsec / 1.0e6;

        if (current_time_ms - start_time_ms > timeout_ms)
        {
            //printf("waitForBytes timed out trying to read %zd bytes in less than %dms\n", n_bytes, timeout_ms);
            return false;
        }
    }

    return true;
}

//#define PRINT_PACKET_BEFORE_SEND
bool DynamixelProDriver::writePacket(uint8_t *packet)
{
    int count = MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    port_->flush();

    count = MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);

    vector<uint8_t> proccessed_packet = stuff_packet(packet);

    uint16_t crc = calculate_crc(&proccessed_packet[0]);//don't count crc

    proccessed_packet.push_back(LOBYTE(crc));
    proccessed_packet.push_back(HIBYTE(crc));
    proccessed_packet[proccessed_packet.size() - 2] = LOBYTE(crc);
    proccessed_packet[proccessed_packet.size() - 1] = HIBYTE(crc);

#ifdef PRINT_PACKET_BEFORE_SEND
    for (int i = 0; i < count + 7; i++)
        cout <<(int)  proccessed_packet[i] << " ";
    cout << endl;
#endif

    return (port_->write(&proccessed_packet[0], count + 7) == (ssize_t) (count + 7));
}

bool DynamixelProDriver::readResponse(std::vector<uint8_t>& response)
{
    struct timespec ts_now;
    clock_gettime(CLOCK_REALTIME, &ts_now);
    double current_time_sec = ts_now.tv_sec + ts_now.tv_nsec / 1.0e9;

    if (current_time_sec - last_reset_sec > 20)
    {
        read_count = 0;
        read_error_count = 0;
        last_reset_sec = current_time_sec;
    }

    ++read_count;

    static const uint16_t timeout_ms = 50;

    uint8_t buffer[70000];
    response.clear();

    // wait until we receive the header bytes and read them
    if (!waitForBytes(7, timeout_ms) || port_->read(buffer, 7) != 7)
    {
        ++read_error_count;
        return false;
    }

    if (buffer[0] == 0xFF && buffer[1] == 0xFF && buffer[2] == 0xFD && buffer[3] != 0xFD)
    {
        response.push_back(buffer[0]);  // 0xFF
        response.push_back(buffer[1]);  // 0xFF
        response.push_back(buffer[2]);  // 0xFD
        response.push_back(buffer[3]);  // reserved (usually 0x00)
        response.push_back(buffer[4]);  // ID
        response.push_back(buffer[5]);  // packet length low
        response.push_back(buffer[6]);  // packet length high
        //you could also verify the instruction which is guaranteed to be 0x55

        uint8_t n_bytes = MAKEWORD(buffer[5], buffer[6]);    // Length
        //printf("recieved %d bytes\n", n_bytes);
        //response.push_back(n_bytes);

        // wait for and read the rest of response bytes
        if (!waitForBytes(n_bytes, timeout_ms) || port_->read(buffer, n_bytes) != n_bytes)
        {
            ++read_error_count;
            response.clear();
            return false;
        }

        for (int i = 0; i < n_bytes; ++i)
        {
            response.push_back(buffer[i]);
        }


        // verify the crc
        // spec guarantees that the data is stored as an array
        uint16_t crc = calculate_crc(&response[0]);
        uint16_t sent_crc = MAKEWORD(response[response.size() - 2], response[response.size() - 1]);

        if (crc != sent_crc)
            cout << "our crc = " << crc << " their crc = " << sent_crc << " our length is " << response.size() << endl;
        return crc == sent_crc;
    }

    return false;
}

uint16_t DynamixelProDriver::calculate_crc(uint8_t *data) const
{
    uint16_t size = MAKEWORD(data[PKT_LENGTH_L], data[PKT_LENGTH_H]) + 5;
    //cout << "size is" << size << endl;

    uint16_t i, j, crc_accum = 0;

    static uint16_t crc_table[256] = {0x0000,
                                0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                                0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                                0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                                0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                                0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                                0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                                0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                                0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                                0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                                0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                                0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                                0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                                0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                                0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                                0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                                0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                                0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                                0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                                0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                                0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                                0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                                0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                                0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                                0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                                0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                                0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                                0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                                0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                                0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                                0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                                0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                                0x820D, 0x8207, 0x0202 };

    for(j = 0; j < size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ *data++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

vector<uint8_t> DynamixelProDriver::stuff_packet(uint8_t *packet) const
{
   vector<uint8_t> stuffed_packet;

    int i = 0;
    int packet_length_in = MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;

    for (i = 0; i < 7; i++)
    {
        stuffed_packet.push_back(packet[i]);
    }

    int currSpot = 0;
    for( i = 0; i < packet_length_in - 2; i++) // don't copy over the crc, its not computed yet anyways
    {
        currSpot = i + 7;
        stuffed_packet.push_back(packet[currSpot ]);//copy the data over

        if(packet[currSpot] == 0xFD && packet[currSpot -1] == 0xFF && packet[currSpot -2] == 0xFF)
        {   // 0xFF 0xFF 0xFD, this means we have to stuff an extra 0xFD in so this packet doesn't look like a header
            stuffed_packet.push_back(0xFD);
            packet_length_out++;
        }
    }

   packet[PKT_LENGTH_L] = LOBYTE(packet_length_out);
   packet[PKT_LENGTH_H] = HIBYTE(packet_length_out);

   return stuffed_packet;
}



}
