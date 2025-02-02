#ifndef LSM6DSO32_HPP
#define LSM6DSO32_HPP

#include <memory>
#include <string>
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <queue>
#include "registers.hpp"

namespace lsm6dso32 {

/**
 * @brief Structure to hold IMU measurements
 */
struct ImuData {
    double timestamp;        // Time in seconds since epoch
    float accel[3];         // Acceleration in m/s^2 (x, y, z)
    float gyro[3];          // Angular velocity in rad/s (x, y, z)
    float temperature;      // Temperature in degrees Celsius
};

/**
 * @brief Main class for interfacing with LSM6DSO32 IMU
 */
class LSM6DSO32 {
public:
    /**
     * @brief Constructor
     * @param device_path Path to I2C device
     * @param address I2C address of the device (default: 0x6A)
     */
    explicit LSM6DSO32(const std::string& device_path, 
                       uint8_t address = DEFAULT_I2C_ADDRESS);

    /**
     * @brief Destructor
     */
    ~LSM6DSO32();

    /**
     * @brief Initialize the IMU
     * @param accel_odr Accelerometer output data rate
     * @param accel_scale Accelerometer full-scale selection
     * @param gyro_odr Gyroscope output data rate
     * @param gyro_scale Gyroscope full-scale selection
     * @return true if initialization successful
     */
    bool initialize(AccelODR accel_odr = AccelODR::Hz208,
                   AccelScale accel_scale = AccelScale::G4,
                   GyroODR gyro_odr = GyroODR::Hz208,
                   GyroScale gyro_scale = GyroScale::DPS2000);

    /**
     * @brief Start continuous reading at specified frequency
     * @param frequency Desired sampling frequency in Hz
     * @return true if successfully started
     */
    bool startContinuousReading(double frequency = 200.0);

    /**
     * @brief Stop continuous reading
     */
    void stopContinuousReading();

    /**
     * @brief Get latest IMU measurement
     * @return Latest IMU measurement
     */
    ImuData getLatestData();

    /**
     * @brief Get all available IMU measurements since last read
     * @return Vector of IMU measurements
     */
    std::vector<ImuData> getAllData();

    /**
     * @brief Perform self-test
     * @return true if self-test passed
     */
    bool performSelfTest();

    /**
     * @brief Reset device
     * @return true if reset successful
     */
    bool reset();

    /**
     * @brief Check if device is responding
     * @return true if device is responding
     */
    bool isResponding() const;

    /**
     * @brief Set accelerometer filter
     * @param filter Filter configuration
     * @return true if successful
     */
    bool setAccelFilter(AccelFilter filter);

    /**
     * @brief Configure interrupt settings
     * @param int1_config Configuration for INT1 pin
     * @param int2_config Configuration for INT2 pin
     * @return true if successful
     */
    bool configureInterrupts(uint8_t int1_config, uint8_t int2_config);

    /**
     * @brief Get device temperature
     * @return Temperature in degrees Celsius
     */
    float getTemperature();

    /**
     * @brief Get error string for last error
     * @return Error string
     */
    std::string getLastError() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    // Prevent copying
    LSM6DSO32(const LSM6DSO32&) = delete;
    LSM6DSO32& operator=(const LSM6DSO32&) = delete;

    // Allow moving
    LSM6DSO32(LSM6DSO32&&) = default;
    LSM6DSO32& operator=(LSM6DSO32&&) = default;
};

class ImuxException : public std::runtime_error {
public:
    explicit ImuxException(const std::string& msg) : std::runtime_error(msg) {}
};

class InitializationError : public ImuxException {
public:
    explicit InitializationError(const std::string& msg) : ImuxException(msg) {}
};

class CommunicationError : public ImuxException {
public:
    explicit CommunicationError(const std::string& msg) : ImuxException(msg) {}
};

class ConfigurationError : public ImuxException {
public:
    explicit ConfigurationError(const std::string& msg) : ImuxException(msg) {}
};

} // namespace lsm6dso32

#endif // LSM6DSO32_HPP
