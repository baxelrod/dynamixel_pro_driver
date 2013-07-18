#ifndef DYNAMIXEL_PRO_DRIVER_H__
#define 

#include <pthread.h>
#include <stdint.h>

#include <set>
#include <map>
#include <string>
#include <vector>

#include <serial/serial.h>
#include <clam/gearbox/flexiport/port.h>

#define ERROR_CHECK_PROTECTED(servo_id, error_code) return validateNoErrorsProtected((servo_id), (error_code), __PRETTY_FUNCTION__);
#define ERROR_CHECK(servo_id, error_code) return validateNoErrors((servo_id), (error_code), __PRETTY_FUNCTION__);

namespace dynamixel_pro_driver 
{

class DynamixelIO
{
public:
    DynamixelIO(std::string device, std::string baud);
    ~DynamixelIO();

    long long unsigned int read_error_count;
    long long unsigned int read_count;
    double last_reset_sec;

    bool ping(int servo_id);
    
    // ****************************** GETTERS ******************************** //
    bool getModelNumber(int servo_id, uint16_t& model_number);
    bool getFirmwareVersion(int servo_id, uint8_t& firmware_version);
    bool getBaudRate(int servo_id, uint8_t& baud_rate);
    bool getReturnDelayTime(int servo_id, uint8_t& return_delay_time);

    bool getOperatingMode(int servo_id, uint8_t &operating_mode);
    
    bool getAngleLimits(int servo_id, uint32_t& min_angle_limit, uint32_t& max_angle_limit);
    bool getMaxAngleLimit(int servo_id, uint32_t& angle);
    bool getMinAngleLimit(int servo_id, uint32_t& angle);
    
    bool getVoltageLimits(int servo_id, float& min_voltage_limit, float& max_voltage_limit);
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
    
    // ****************************** SETTERS ******************************** //
    bool setId(int servo_id, uint8_t id); //untested
    bool setBaudRate(int servo_id, uint8_t baud_rate);//untested
    bool setReturnDelayTime(int servo_id, uint8_t return_delay_time);//untested

    bool setOperatingMode(int servo_id, uint8_t operating_mode);
    
    bool setAngleLimits(int servo_id, int32_t min_angle, int32_t max_angle); 
    bool setMinAngleLimit(int servo_id, int32_t angle);
    bool setMaxAngleLimit(int servo_id, int32_t angle);
   
    bool setTemperatureLimit(int servo_id, uint8_t max_temperature);
    bool setMaxTorque(int servo_id, uint16_t max_torque); 
    bool setTorqueEnable(int servo_id, bool on);
    
    bool setPosition(int servo_id, uint32_t position);
    bool setVelocity(int servo_id, int32_t velocity);
    
    // ************************* SYNC_WRITE METHODS *************************** //
    bool setMultiPosition(std::vector<std::vector<int> > value_pairs);
    bool setMultiVelocity(std::vector<std::vector<int> > value_pairs);
    bool setMultiPositionVelocity(std::vector<std::vector<int> > value_tuples);
    bool setMultiTorqueEnabled(std::vector<std::vector<int> > value_pairs);
    
protected:
    bool validateNoErrorsProtected(int servo_id, uint8_t error_code, std::string method_name); // returns true if no error

    bool validateNoErrors(int servo_id, uint8_t error_code, std::string command_failed);

    bool read(int servo_id,
              int address,
              int size,
              std::vector<uint8_t>& response);

    bool write(int servo_id,
               int address,
               const std::vector<uint8_t>& data,
               std::vector<uint8_t>& response);

    bool syncWrite(int address,
                   const std::vector<std::vector<uint8_t> >& data);
    
private:
    serial::Serial *port_;
    pthread_mutex_t serial_mutex_;
    
    bool waitForBytes(ssize_t n_bytes, uint16_t timeout_ms);
    
    bool writePacket(uint8_t *packet);
    bool readResponse(std::vector<uint8_t>& response);

    uint16_t calculate_crc(uint8_t *data);
    std::vector<uint8_t> stuff_packet(uint8_t *packet);
};

}

#endif 
