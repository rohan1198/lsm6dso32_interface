#include "lsm6dso32.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>

using namespace lsm6dso32;

volatile bool running = true;

void signalHandler(int /*signum*/) {
    std::cout << "\nInterrupt signal received. Stopping...\n";
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);

    try {
        LSM6DSO32 imu("/dev/i2c-1");

        if (!imu.initialize(AccelODR::Hz208,    // 208 Hz accelerometer
                          AccelScale::G4,        // ±4g range
                          GyroODR::Hz208,        // 208 Hz gyroscope
                          GyroScale::DPS2000)) { // ±2000 dps range
            std::cerr << "Failed to initialize IMU: " << imu.getLastError() << std::endl;
            return 1;
        }

        std::cout << "IMU initialized successfully\n";

        if (!imu.startContinuousReading(200.0)) {
            std::cerr << "Failed to start continuous reading\n";
            return 1;
        }

        std::cout << "Started continuous reading at 200 Hz\n";
        std::cout << "Press Ctrl+C to stop\n\n";

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Timestamp  |    Accelerometer (m/s²)    |     Gyroscope (rad/s)      | Temp (°C)\n";
        std::cout << "           |    X     Y     Z          |    X     Y     Z          |\n";
        std::cout << "-----------+------------------------+------------------------+---------\n";

        int print_counter = 0;
        while (running) {
            try {
                ImuData data = imu.getLatestData();

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

                std::this_thread::sleep_for(std::chrono::milliseconds(1));

            } catch (const std::exception& e) {
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
