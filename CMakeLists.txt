cmake_minimum_required(VERSION 3.8)
project(lsm6dso32_driver VERSION 1.0.0 LANGUAGES CXX)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
    set(BUILD_ROS2 ON)
    message(STATUS "ROS 2 environment detected, building ROS 2 wrapper")
else()
    set(BUILD_ROS2 OFF)
    message(STATUS "No ROS 2 environment detected, building standalone only")
endif()

add_subdirectory(standalone)

if(BUILD_ROS2)
    # Find ROS 2 packages
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)

    add_subdirectory(ros2)

    ament_package()
endif()
