#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class VxsPublisher : public rclcpp::Node
{

public:
    VxsPublisher()
        : Node("vxs_sensor")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("vxs_data", 10);
        timer_ = this->create_wall_timer( //
            500ms,                        //
            std::bind(                    //
                &VxsPublisher::TimerCB,   //
                this)                     //
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    void TimerCB();
};

void VxsPublisher::TimerCB()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VxsPublisher>());
    rclcpp::shutdown();
    return 0;
}