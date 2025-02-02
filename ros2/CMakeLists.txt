# Find required ROS packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/standalone/include
    ${PROJECT_SOURCE_DIR}/ros2/include
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

# Create ROS 2 node library
add_library(imu_node
    src/ros2_wrapper/imu_node.cpp
)

# Add dependencies to ensure message headers are built
ament_target_dependencies(imu_node
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
)

target_link_libraries(imu_node
    lsm6dso32
)

# Create ROS 2 node executable
add_executable(imu_node_exec
    src/ros2_wrapper/imu_node_main.cpp
)

ament_target_dependencies(imu_node_exec
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
)

target_link_libraries(imu_node_exec
    imu_node
)

# Install targets
install(TARGETS
    imu_node
    imu_node_exec
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)
