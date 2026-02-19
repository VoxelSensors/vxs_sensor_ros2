#include <rclcpp/rclcpp.hpp>
#include "obstacle_avoider.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vxs_ros::ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}