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

#ifndef DYNAMIXEL_PRO_DRIVER_H__
#define DYNAMIXEL_PRO_DRIVER_H__

#include <pthread.h>
#include <stdint.h>

#include <set>
#include <map>
#include <string>
#include <vector>

#include <serial/serial.h>

/**
 * does the same thing as error_check but spits out a more intelligent error
 * message if you're having problems writing to protected
 * parameters on the dynamixel
*/
#define DMX_PRO_DRIVER_ERROR_CHECK_PROTECTED(servo_id, error_code) validateNoErrorsProtected((servo_id), (error_code), __PRETTY_FUNCTION__);

/**
 * Calls the validate no errors function with the current function name.
 */
#define DMX_PRO_DRIVER_ERROR_CHECK(servo_id, error_code) validateNoErrors((servo_id), (error_code), __PRETTY_FUNCTION__);

namespace dynamixel_pro_driver
{

/**
 * Provides the interface to control Dynamixel pro motors
 */
class DynamixelProDriver
{
public:
    DynamixelProDriver(const std::string &device = "/dev/ttyUSB0", const std::string &baud = "1000000",
        uint32_t timeout_ms = 1);
    ~DynamixelProDriver();

    long long unsigned int read_error_count;
    long long unsigned int read_count;
    double last_reset_sec;

    bool ping(int servo_id);

    // **************************** GETTERS ******************************** //
    bool getModelNumber(int servo_id, uint16_t& model_number);
    bool getModelInfo(int servo_id, uint32_t& model_info);
    bool getFirmwareVersion(int servo_id, uint8_t& firmware_version);
    bool getBaudRate(int servo_id, uint8_t& baud_rate);
    bool getReturnDelayTime(int servo_id, uint8_t& return_delay_time);

    bool getOperatingMode(int servo_id, uint8_t &operating_mode);

    bool getAngleLimits(int servo_id, uint32_t& min_angle_limit,
        uint32_t& max_angle_limit);
    bool getMaxAngleLimit(int servo_id, uint32_t& angle);
    bool getMinAngleLimit(int servo_id, uint32_t& angle);

    bool getVoltageLimits(int servo_id, float& min_voltage_limit,
        float& max_voltage_limit);
    bool getMinVoltageLimit(int servo_id, float& min_voltage_limit);
    bool getMaxVoltageLimit(int servo_id, float& max_voltage_limit);

    bool getTemperatureLimit(int servo_id, uint8_t& max_temperature);
    bool getMaxTorque(int servo_id, uint16_t& max_torque);
    bool getTorqueEnabled(int servo_id, bool& torque_enabled);

    bool getTargetPosition(int servo_id, int32_t& target_position);
    bool getTargetVelocity(int servo_id, int32_t& target_velocity);

    bool getPosition(int servo_id, int32_t& position);
    bool getVelocity(int servo_id, int32_t& velocity);
    bool getCurrent(int servo_id, uint16_t& current);
    bool getVoltage(int servo_id, float& voltage);
    bool getTemperature(int servo_id, uint8_t& temperature);

    // **************************** SETTERS ******************************** //
    bool setId(int servo_id, uint8_t id); //not full tested
    bool setBaudRate(int servo_id, uint8_t baud_rate);//not fully tested
    bool setReturnDelayTime(int servo_id, uint8_t return_delay_time);//not fully tested

    bool setOperatingMode(int servo_id, uint8_t operating_mode);

    bool setAngleLimits(int servo_id, int32_t min_angle, int32_t max_angle);
    bool setMinAngleLimit(int servo_id, int32_t angle);
    bool setMaxAngleLimit(int servo_id, int32_t angle);

    bool setTemperatureLimit(int servo_id, uint8_t max_temperature);
    bool setMaxTorque(int servo_id, uint16_t max_torque);
    bool setTorqueEnabled(int servo_id, bool on);

    bool setPosition(int servo_id, uint32_t position);
    bool setVelocity(int servo_id, int32_t velocity);

    // *********************** SYNC_WRITE METHODS *************************** //
    bool setMultiPosition(const std::vector<std::vector<int> > &value_pairs);
    bool setMultiVelocity(const std::vector<std::vector<int> > &value_pairs);

    //set position setpoint and velocity limit
    bool setMultiPositionVelocity(const std::vector<std::vector<int> > &value_tuples);
    bool setMultiTorqueEnabled(const std::vector<std::vector<int> > &value_pairs);

protected:
    /**
     * \brief Checks for errors in the error byte and prints out an error
     * description if neccessary. It gives a more usefull error message
     * in the case that you are writing to a protected field.
     */
    bool validateNoErrorsProtected(int servo_id,
        uint8_t error_code, const std::string &method_name); // returns true if no error

    /**
     * \brief Checks for errors in the error byte and prints out an error
     * description if neccessary.
     */
    bool validateNoErrors(int servo_id, uint8_t error_code,
        const std::string &command_failed);

    /**
     * Reads a value from the specified dynamixel at the specified address
     */
    bool read(int servo_id,
              int address,
              int size,
              std::vector<uint8_t>& response);

    /**
     * Writes a value to the specified dynamixel at the specified address
     */
    bool write(int servo_id,
               int address,
               const std::vector<uint8_t>& data,
               std::vector<uint8_t>& response);

    /**
     * Writes values to multiple dynamixels at a single, specified address
     */
    bool syncWrite(int address,
                   const std::vector<std::vector<uint8_t> >& data);

private:
    serial::Serial *port_;
    pthread_mutex_t serial_mutex_;

    bool waitForBytes(ssize_t n_bytes, uint16_t timeout_ms);

    bool writePacket(uint8_t *packet);
    bool readResponse(std::vector<uint8_t>& response);

    /**
     * Calculates the CRC of the packet as per Robotis's formula.
     * It will automatically ignore already present CRC bytes.
     */
    uint16_t calculate_crc(uint8_t *data) const;

    /**
     * Stuffs the packets to deal with restrictions on sending too many
     * Consecutive ones.
     */
    std::vector<uint8_t> stuff_packet(uint8_t *packet) const;
};

}

#endif
