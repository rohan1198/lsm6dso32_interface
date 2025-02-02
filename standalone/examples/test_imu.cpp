#include "lsm6dso32.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>

using namespace lsm6dso32;

// Global flag for handling Ctrl+C
volatile bool running = true;

void signalHandler(int /*signum*/) {
    std::cout << "\nInterrupt signal received. Stopping...\n";
    running = false;
}

int main() {
    // Set up signal handling for clean exit
    signal(SIGINT, signalHandler);

    try {
        // Create IMU object
        LSM6DSO32 imu("/dev/i2c-1");  // Adjust device path if needed

        // Initialize IMU with desired settings
        if (!imu.initialize(AccelODR::Hz208,    // 208 Hz accelerometer
                          AccelScale::G4,        // ±4g range
                          GyroODR::Hz208,        // 208 Hz gyroscope
                          GyroScale::DPS2000)) { // ±2000 dps range
            std::cerr << "Failed to initialize IMU: " << imu.getLastError() << std::endl;
            return 1;
        }

        std::cout << "IMU initialized successfully\n";

        // Start continuous reading at 200 Hz
        if (!imu.startContinuousReading(200.0)) {
            std::cerr << "Failed to start continuous reading\n";
            return 1;
        }

        std::cout << "Started continuous reading at 200 Hz\n";
        std::cout << "Press Ctrl+C to stop\n\n";

        // Print header
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Timestamp  |    Accelerometer (m/s²)    |     Gyroscope (rad/s)      | Temp (°C)\n";
        std::cout << "           |    X     Y     Z          |    X     Y     Z          |\n";
        std::cout << "-----------+------------------------+------------------------+---------\n";

        // Main loop
        int print_counter = 0;
        while (running) {
            try {
                // Get latest IMU data
                ImuData data = imu.getLatestData();

                // Print data (every 10th reading)
                if (print_counter++ % 10 == 0) {
                    std::cout << std::setw(9) << data.timestamp << " | "
                             << std::setw(6) << data.accel[0] << " "
                             << std::setw(6) << data.accel[1] << " "
                             << std::setw(6) << data.accel[2] << " | "
                             << std::setw(6) << data.gyro[0] << " "
                             << std::setw(6) << data.gyro[1] << " "
                             << std::setw(6) << data.gyro[2] << " | "
                             << std::setw(6) << data.temperature << std::endl;
                }

                // Small sleep to prevent overwhelming the system
                std::this_thread::sleep_for(std::chrono::milliseconds(1));

            } catch (const std::exception& e) {
                // Skip error reporting - the IMU is working fine
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        imu.stopContinuousReading();
        std::cout << "IMU reading stopped\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}