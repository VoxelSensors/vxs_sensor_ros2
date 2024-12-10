#include <chrono>
#include <functional>

#include "publisher/vxs_node.hpp"
#include "subscriber/vxs_subscriber.hpp"

namespace vxs_ros
{
    VxsSensorSubscriber::VxsSensorSubscriber() : Node("vxs_cpp_subscriber")
    {
        cam_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>( //
            "/sensor/camera_info",                                                      //
            10,                                                                         //
            std::bind(&VxsSensorSubscriber::CameraInfoCB, this, std::placeholders::_1)  //
        );
        depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>( //
            "/depth/image",                                                     //
            5,
            std::bind(&VxsSensorSubscriber::DepthImageCB, this, std::placeholders::_1) //
        );
    }

    VxsSensorSubscriber::~VxsSensorSubscriber()
    {
    }

    void VxsSensorSubscriber::CameraInfoCB(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg) const
    {
    }

    void VxsSensorSubscriber::DepthImageCB(const sensor_msgs::msg::Image::SharedPtr depth_img_msg) const
    {
        try
        {
            cv::Mat depth_img = cv_bridge::toCvShare(depth_img_msg, sensor_msgs::image_encodings::MONO16)->image;
            cv::imshow("view", depth_img);
            cv::waitKey(10);
        }
        catch (const cv_bridge::Exception &e)
        {
            auto logger = rclcpp::get_logger("my_subscriber");
            RCLCPP_ERROR(logger, "Could not convert from '%s' to 'mono16'.", depth_img_msg->encoding.c_str());
        }
    }

} // end namespace vxs_ros