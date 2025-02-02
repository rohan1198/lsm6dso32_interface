#include "lsm6dso32.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <system_error>
#include <chrono>
#include <thread>
#include <queue>
#include <iostream>
#include <cmath>

namespace lsm6dso32 {

// Implementation class (PIMPL)
class LSM6DSO32::Impl {
public:
    Impl(const std::string& device_path, uint8_t address) 
        : device_path_(device_path)
        , address_(address)
        , fd_(-1)
        , running_(false)
        , accel_scale_(AccelScale::G4)
        , gyro_scale_(GyroScale::DPS2000)
        , last_error_("")
        , data_queue_()
        , queue_mutex_()
    {
    }

    ~Impl() {
        stopContinuousReading();
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    bool openDevice() {
        fd_ = open(device_path_.c_str(), O_RDWR);
        if (fd_ < 0) {
            last_error_ = "Failed to open I2C device: " + std::string(strerror(errno));
            return false;
        }

        if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
            last_error_ = "Failed to set I2C slave address: " + std::string(strerror(errno));
            close(fd_);
            fd_ = -1;
            return false;
        }

        return true;
    }

    bool writeRegister(Register reg, uint8_t value) {
        uint8_t buffer[2] = {static_cast<uint8_t>(reg), value};
        if (write(fd_, buffer, 2) != 2) {
            last_error_ = "Failed to write register: " + std::string(strerror(errno));
            return false;
        }
        return true;
    }

    bool readRegister(Register reg, uint8_t& value) {
        uint8_t reg_addr = static_cast<uint8_t>(reg);
        if (write(fd_, &reg_addr, 1) != 1) {
            last_error_ = "Failed to write register address: " + std::string(strerror(errno));
            return false;
        }
        
        if (read(fd_, &value, 1) != 1) {
            last_error_ = "Failed to read register: " + std::string(strerror(errno));
            return false;
        }
        return true;
    }

    bool readRegisters(Register reg, uint8_t* buffer, size_t length) {
        uint8_t reg_addr = static_cast<uint8_t>(reg);
        if (write(fd_, &reg_addr, 1) != 1) {
            last_error_ = "Failed to write register address: " + std::string(strerror(errno));
            return false;
        }
        
        if (read(fd_, buffer, length) != static_cast<ssize_t>(length)) {
            last_error_ = "Failed to read registers: " + std::string(strerror(errno));
            return false;
        }
        return true;
    }

    bool initialize(AccelODR accel_odr, AccelScale accel_scale,
                   GyroODR gyro_odr, GyroScale gyro_scale) {
        std::cout << "Initializing IMU..." << std::endl;
        
        if (!openDevice()) {
            return false;
        }
        
        std::cout << "I2C device opened successfully" << std::endl;

        // Verify device ID
        uint8_t who_am_i;
        if (!readRegister(Register::WHO_AM_I, who_am_i) || who_am_i != WHO_AM_I_VALUE) {
            last_error_ = "Invalid WHO_AM_I value";
            return false;
        }

        // Reset device
        if (!reset()) {
            return false;
        }

        // Wait for reset to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Configure accelerometer
        uint8_t ctrl1_xl = (static_cast<uint8_t>(accel_odr) << 4) | 
                          (static_cast<uint8_t>(accel_scale) << 2);
        if (!writeRegister(Register::CTRL1_XL, ctrl1_xl)) {
            return false;
        }
        
        // Verify accelerometer configuration
        uint8_t verify_xl;
        if (!readRegister(Register::CTRL1_XL, verify_xl)) {
            return false;
        }
        std::cout << "CTRL1_XL set to: 0x" << std::hex << (int)verify_xl << std::dec 
                  << " (wanted 0x" << std::hex << (int)ctrl1_xl << std::dec << ")" << std::endl;

        // Configure gyroscope
        uint8_t ctrl2_g = (static_cast<uint8_t>(gyro_odr) << 4) | 
                         (static_cast<uint8_t>(gyro_scale) << 2);
        if (!writeRegister(Register::CTRL2_G, ctrl2_g)) {
            return false;
        }

        // Verify gyroscope configuration
        uint8_t verify_g;
        if (!readRegister(Register::CTRL2_G, verify_g)) {
            return false;
        }
        std::cout << "CTRL2_G set to: 0x" << std::hex << (int)verify_g << std::dec 
                  << " (wanted 0x" << std::hex << (int)ctrl2_g << std::dec << ")" << std::endl;

        // Enable Block Data Update
        uint8_t ctrl3_c;
        if (!readRegister(Register::CTRL3_C, ctrl3_c)) {
            return false;
        }
        ctrl3_c |= ControlBits::BDU;
        if (!writeRegister(Register::CTRL3_C, ctrl3_c)) {
            return false;
        }

        accel_scale_ = accel_scale;
        gyro_scale_ = gyro_scale;

        return true;
    }

    bool reset() {
        uint8_t ctrl3_c;
        if (!readRegister(Register::CTRL3_C, ctrl3_c)) {
            return false;
        }
        ctrl3_c |= ControlBits::SW_RESET;
        return writeRegister(Register::CTRL3_C, ctrl3_c);
    }

    bool readSensorData(ImuData& data) {

        uint8_t buffer[14];  // Temperature (2) + Gyro (6) + Accel (6)
        
        // Read all sensor data in one transaction
        if (!readRegisters(Register::OUT_TEMP_L, buffer, sizeof(buffer))) {
            return false;
        }

        // Get timestamp
        data.timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        // Process temperature
        int16_t temp_raw = static_cast<int16_t>(buffer[1] << 8 | buffer[0]);
        data.temperature = (temp_raw / ConversionFactors::TEMP_SCALE) + ConversionFactors::TEMP_OFFSET;

        // Process gyroscope data
        float gyro_scale = ConversionFactors::GYRO_250_SCALE;  // Default value
        switch (gyro_scale_) {
            case GyroScale::DPS250:  gyro_scale = ConversionFactors::GYRO_250_SCALE; break;
            case GyroScale::DPS500:  gyro_scale = ConversionFactors::GYRO_500_SCALE; break;
            case GyroScale::DPS1000: gyro_scale = ConversionFactors::GYRO_1000_SCALE; break;
            case GyroScale::DPS2000: gyro_scale = ConversionFactors::GYRO_2000_SCALE; break;
        }

        for (int i = 0; i < 3; i++) {
            int16_t gyro_raw = static_cast<int16_t>(buffer[3+2*i] << 8 | buffer[2+2*i]);
            // Convert from mdps to rad/s
            data.gyro[i] = (gyro_raw * gyro_scale) * (M_PI / 180000.0f);
        }

        // Process accelerometer data
        float accel_scale = ConversionFactors::ACCEL_G4_SCALE;  // Default value
        switch (accel_scale_) {
            case AccelScale::G4:  accel_scale = ConversionFactors::ACCEL_G4_SCALE; break;
            case AccelScale::G8:  accel_scale = ConversionFactors::ACCEL_G8_SCALE; break;
            case AccelScale::G16: accel_scale = ConversionFactors::ACCEL_G16_SCALE; break;
            case AccelScale::G32: accel_scale = ConversionFactors::ACCEL_G32_SCALE; break;
        }

        for (int i = 0; i < 3; i++) {
            int16_t accel_raw = static_cast<int16_t>(buffer[9+2*i] << 8 | buffer[8+2*i]);
            // Convert from mg to m/s²
            data.accel[i] = (accel_raw * accel_scale) * 0.00980665f;
        }

        return true;
    }

    void continuousReadingLoop(double frequency) {
        const std::chrono::duration<double> period(1.0 / frequency);
        auto next_reading = std::chrono::steady_clock::now();

        while (running_) {
            ImuData data;
            if (readSensorData(data)) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                data_queue_.push(data);
            }

            next_reading += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next_reading);
        }
    }

    bool startContinuousReading(double frequency) {
        if (running_) {
            return false;
        }

        running_ = true;
        read_thread_ = std::thread(&Impl::continuousReadingLoop, this, frequency);
        
        // Set thread priority (requires root privileges)
        sched_param sch_params;
        sch_params.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (pthread_setschedparam(read_thread_.native_handle(), SCHED_FIFO, &sch_params) != 0) {
            std::cerr << "Warning: Could not set thread priority" << std::endl;
        }

        return true;
    }

    void stopContinuousReading() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

    ImuData getLatestData() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (data_queue_.empty()) {
            // Instead of throwing, return the last known data
            if (last_known_data.timestamp == 0) {
                // Only throw if we've never received data
                throw std::runtime_error("No data available");
            }
            return last_known_data;
        }
        
        last_known_data = data_queue_.back();
        while (!data_queue_.empty()) data_queue_.pop();
        return last_known_data;
    }

    std::vector<ImuData> getAllData() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::vector<ImuData> all_data;
        while (!data_queue_.empty()) {
            all_data.push_back(data_queue_.front());
            data_queue_.pop();
        }
        return all_data;
    }

    std::string getLastError() const {
        return last_error_;
    }

private:
    std::string device_path_;
    uint8_t address_;
    int fd_;
    std::atomic<bool> running_;
    std::thread read_thread_;
    AccelScale accel_scale_;
    GyroScale gyro_scale_;
    std::string last_error_;
    std::queue<ImuData> data_queue_;
    std::mutex queue_mutex_;
    ImuData last_known_data{};  // Initialize all members to 0
};

// Main class implementation (delegates to PIMPL)
LSM6DSO32::LSM6DSO32(const std::string& device_path, uint8_t address)
    : pimpl_(std::make_unique<Impl>(device_path, address)) {
}

LSM6DSO32::~LSM6DSO32() = default;

bool LSM6DSO32::initialize(AccelODR accel_odr, AccelScale accel_scale,
                          GyroODR gyro_odr, GyroScale gyro_scale) {
    return pimpl_->initialize(accel_odr, accel_scale, gyro_odr, gyro_scale);
}

bool LSM6DSO32::startContinuousReading(double frequency) {
    return pimpl_->startContinuousReading(frequency);
}

void LSM6DSO32::stopContinuousReading() {
    pimpl_->stopContinuousReading();
}

ImuData LSM6DSO32::getLatestData() {
    return pimpl_->getLatestData();
}

std::vector<ImuData> LSM6DSO32::getAllData() {
    return pimpl_->getAllData();
}

bool LSM6DSO32::reset() {
    return pimpl_->reset();
}

std::string LSM6DSO32::getLastError() const {
    return pimpl_->getLastError();
}

} // namespace lsm6dso32
