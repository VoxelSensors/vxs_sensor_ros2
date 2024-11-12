#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <SDK2.h>

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
    bool InitSensor(const std::string &cam_config_json, const std::string &cam_calib_json);
};

void VxsPublisher::TimerCB()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

bool InitSennsor(const std::string &cam_config_json, const std::string &cam_calib_json)
{
    int FPS = 25;
    // uint32_t bytes_to_fetch = 2 * 1024 * 1024;
    vxsdk::pipelineType pipeline_type = vxsdk::pipelineType::fbPointcloud; // @TODO: For now, need a pointcloud...

    // Set the frame rate - used only in the case of frame based rendering
    vxsdk::vxSetFPS(FPS);
    // Set the amount of bytes to fetch - used only in the case of data based rendering
    // vxsdk::vxSetBytesToFetch(bytesToFetch);

    // Start the SDK Engine.
    // @TODO: Capture/handle the return value (number of cameras)
    // @TODO: Now calibration file is **ONE**
    int cam_num = vxsdk::vxStartSystem( //
        cam_config_json.c_str(),        //
        cam_calib_json.c_str(),         //
        pipeline_type);

    return cam_num > 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VxsPublisher>());
    rclcpp::shutdown();
    return 0;
}