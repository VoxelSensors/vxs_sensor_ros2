#include <vxs_node.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vxs_ros::VxsSensorPublisher>());
    rclcpp::shutdown();
    return 0;
}