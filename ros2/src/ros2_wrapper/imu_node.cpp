#include "ros2_wrapper/imu_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <memory>

namespace lsm6dso32_ros2
{

ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: Node("lsm6dso32_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting LSM6DSO32 IMU node...");
    
    try {
        initializeParameters();
        initializeIMU();
        initializePublishers();
        
        // Create timer for publishing data
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&ImuNode::publishData, this));
        
        RCLCPP_INFO(this->get_logger(), "IMU node initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing node: %s", e.what());
        throw;
    }
}

ImuNode::~ImuNode()
{
    if (imu_) {
        imu_->stopContinuousReading();
    }
}

void ImuNode::initializeParameters()
{
    // Declare and get parameters
    this->declare_parameter("device_path", "/dev/i2c-1");
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("publish_rate", 200.0);
    this->declare_parameter("publish_tf", false);

    device_path_ = this->get_parameter("device_path").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    RCLCPP_INFO(this->get_logger(), "Using device: %s", device_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing at %f Hz", publish_rate_);
}

void ImuNode::initializePublishers()
{
    // Create IMU message publisher with reliable QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data_raw", 
        qos);

    // Create TF broadcaster if needed
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
}

void ImuNode::initializeIMU()
{
    imu_ = std::make_unique<lsm6dso32::LSM6DSO32>(device_path_);

    if (!imu_->initialize(
        lsm6dso32::AccelODR::Hz208,
        lsm6dso32::AccelScale::G4,
        lsm6dso32::GyroODR::Hz208,
        lsm6dso32::GyroScale::DPS2000)) {
        throw std::runtime_error("Failed to initialize IMU: " + imu_->getLastError());
    }

    if (!imu_->startContinuousReading(publish_rate_)) {
        throw std::runtime_error("Failed to start continuous reading");
    }
}

void ImuNode::publishData()
{
    try {
        auto data = imu_->getLatestData();
        
        // Create and publish IMU message
        auto imu_msg = createImuMessage(data);
        imu_pub_->publish(imu_msg);

        // Publish transform if enabled
        if (publish_tf_) {
            auto transform_msg = createTransformMessage(data);
            tf_broadcaster_->sendTransform(transform_msg);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000, // Throttle period in ms
            "Error getting IMU data: %s", e.what());
    }
}

sensor_msgs::msg::Imu ImuNode::createImuMessage(const lsm6dso32::ImuData& data)
{
    sensor_msgs::msg::Imu msg;
    
    // Set header
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    // Set linear acceleration (m/s^2)
    msg.linear_acceleration.x = data.accel[0];
    msg.linear_acceleration.y = data.accel[1];
    msg.linear_acceleration.z = data.accel[2];

    // Set angular velocity (rad/s)
    msg.angular_velocity.x = data.gyro[0];
    msg.angular_velocity.y = data.gyro[1];
    msg.angular_velocity.z = data.gyro[2];

    // Set covariance matrices (example values - should be calibrated)
    double linear_accel_stdev = 0.0003; // Example: 0.3 mg
    double angular_vel_stdev = 0.0001;  // Example: 0.1 deg/s
    
    for (int i = 0; i < 9; ++i) {
        msg.linear_acceleration_covariance[i] = 0.0;
        msg.angular_velocity_covariance[i] = 0.0;
    }
    
    msg.linear_acceleration_covariance[0] = 
    msg.linear_acceleration_covariance[4] = 
    msg.linear_acceleration_covariance[8] = linear_accel_stdev * linear_accel_stdev;

    msg.angular_velocity_covariance[0] = 
    msg.angular_velocity_covariance[4] = 
    msg.angular_velocity_covariance[8] = angular_vel_stdev * angular_vel_stdev;

    // Orientation not provided by this IMU
    msg.orientation_covariance[0] = -1;

    return msg;
}

geometry_msgs::msg::TransformStamped ImuNode::createTransformMessage(
    const lsm6dso32::ImuData& data)
{
    geometry_msgs::msg::TransformStamped transform;
    
    transform.header.stamp = this->now();
    transform.header.frame_id = "base_link";  // Parent frame
    transform.child_frame_id = frame_id_;     // IMU frame

    // Set translation (if mounted with offset from base_link)
    transform.transform.translation.x = 0.0;  // Adjust these values based on
    transform.transform.translation.y = 0.0;  // your IMU mounting position
    transform.transform.translation.z = 0.0;  // relative to base_link

    // Set rotation (identity quaternion since we don't have orientation)
    transform.transform.rotation.w = 1.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;

    return transform;
}

} // namespace lsm6dso32_ros2
