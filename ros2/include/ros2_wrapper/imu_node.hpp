#ifndef LSM6DSO32_ROS2_NODE_HPP
#define LSM6DSO32_ROS2_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "lsm6dso32.hpp"

namespace lsm6dso32_ros2
{

class ImuNode : public rclcpp::Node
{
public:
    explicit ImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~ImuNode();

private:
    std::string device_path_;
    std::string frame_id_;
    double publish_rate_;
    bool publish_tf_;

    std::unique_ptr<lsm6dso32::LSM6DSO32> imu_;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;

    void initializeParameters();
    void initializePublishers();
    void initializeIMU();
    void publishData();
    sensor_msgs::msg::Imu createImuMessage(const lsm6dso32::ImuData& data);
    geometry_msgs::msg::TransformStamped createTransformMessage(const lsm6dso32::ImuData& data);
};

} // namespace lsm6dso32_ros2

#endif // LSM6DSO32_ROS2_NODE_HPP
