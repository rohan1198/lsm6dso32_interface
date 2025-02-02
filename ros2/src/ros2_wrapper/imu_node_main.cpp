#include "ros2_wrapper/imu_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lsm6dso32_ros2::ImuNode>());
    rclcpp::shutdown();
    return 0;
}
