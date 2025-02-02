# LSM6DSO32 IMU Driver

This repository contains a C++ driver for the LSM6DSO32 IMU (Inertial Measurement Unit), with both standalone and ROS 2 support. The driver provides a robust interface for reading accelerometer and gyroscope data, along with real-time visualization capabilities using RViz2.

## Features

- Standalone C++ driver with continuous reading support
- ROS 2 node
- Real-time 3D visualization using RViz2
- Configurable sampling rates and ranges
- Thread-safe implementation

## Hardware Requirements

- LSM6DSO32 IMU (e.g., Adafruit LSM6DSO32 breakout board)
- NVIDIA Jetson Orin Nano (or similar Linux system with I2C support)
- I2C connection to the IMU

## Software Requirements

### Standalone Version

- C++17 compiler
- CMake (>= 3.8)
- pthread library

### ROS 2 Version

- ROS 2 Humble
- sensor_msgs
- geometry_msgs
- tf2
- tf2_ros
- rviz2
- imu_tools

### Building the Driver

#### Standalone

```bash
# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make

# Install (optional)
sudo make install
```

#### ROS 2

```bash
# Create a ROS 2 workspace
mkdir -p ~/imu_ws/src
cd ~/imu_ws/src

# Clone the repository
git clone https://github.com/rohan1198/lsm6dso32_driver.git

# Install dependencies
sudo apt-get update
sudo apt-get install -y ros-humble-imu-tools ros-humble-rviz2

# Build the package
cd ~/imu_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Standalone

```bash
# Run the test program
./build/test_imu
```

### ROS 2

```bash
# Launch IMU node with visualization
ros2 launch lsm6dso32_driver imu_visualization.launch.py

# In another terminal, check the IMU data
ros2 topic echo /imu/data_raw

# Check publishing rate
ros2 topic hz /imu/data_raw
```

## Launch File Parameters

The ROS 2 launch file supports the following parameters:

- `device_path` (default: "/dev/i2c-1"): Path to I2C device
- `frame_id` (default: "imu_link"): Frame ID for IMU messages
- `publish_rate` (default: 200.0): Data publishing rate in Hz
- `publish_tf` (default: true): Whether to publish TF transforms

## Project Structure

```
lsm6dso32_driver/
├── CMakeLists.txt
├── package.xml
├── standalone/
│   ├── CMakeLists.txt
│   ├── examples/
│   │   └── test_imu.cpp
│   ├── include/
│   │   ├── lsm6dso32.hpp
│   │   └── registers.hpp
│   └── src/
│       └── lsm6dso32.cpp
└── ros2/
    ├── CMakeLists.txt
    ├── launch/
    │   └── imu_visualization.launch.py
    ├── config/
    │   └── imu_visualization.rviz
    ├── include/
    │   └── ros2_wrapper/
    │       └── imu_node.hpp
    └── src/
        └── ros2_wrapper/
            ├── imu_node.cpp
            └── imu_node_main.cpp
```

## IMU Data

The driver provides the following measurements:

- Acceleration in m/s² (3 axes)
- Angular velocity in rad/s (3 axes)
- Temperature in °C

Default configuration:

- Accelerometer: 208 Hz, ±4g range
- Gyroscope: 208 Hz, ±2000 dps range

## Troubleshooting

1. Check IMU connection:

```bash
i2cdetect -y 1  # Should show your device at address 0x6A
```

2. Verify permissions:

```bash
ls -l /dev/i2c-1  # Should show proper group and permissions
```

3. Common ROS 2 checks:

```bash
# Check if topics are published
ros2 topic list | grep imu

# Verify message contents
ros2 topic echo /imu/data_raw

# Check TF tree
ros2 run tf2_tools view_frames
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- Adafruit for their LSM6DSO32 breakout board
- ROS 2 community for their tools and support
