#include <rclcpp/rclcpp.hpp>
#include "safety_driver/safety_driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vxs_ros::VxsSafetyDriver>());
    rclcpp::shutdown();
    return 0;
}