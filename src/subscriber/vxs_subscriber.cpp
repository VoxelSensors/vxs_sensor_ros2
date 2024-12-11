#include <chrono>
#include <functional>

#include "common.hpp"
#include "subscriber/vxs_subscriber.hpp"
namespace vxs_ros
{
    VxsSensorSubscriber::VxsSensorSubscriber() : Node("vxs_cpp_subscriber"), cam_(nullptr)
    {
        cam_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>( //
            "/sensor/camera_info",                                                      //
            10,                                                                         //
            std::bind(&VxsSensorSubscriber::CameraInfoCB, this, std::placeholders::_1)  //
        );
        depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(        //
            "/depth/image",                                                            //
            5,                                                                         //
            std::bind(&VxsSensorSubscriber::DepthImageCB, this, std::placeholders::_1) //
        );
    }

    VxsSensorSubscriber::~VxsSensorSubscriber()
    {
    }

    void VxsSensorSubscriber::CameraInfoCB(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
    {
        if (!cam_)
        {
            cam_ = std::make_shared<CameraCalibration>();
            for (size_t r = 0; r < 3; r++)
            {
                for (size_t c = 0; c < 3; c++)
                {
                    cam_->K(r, c) = static_cast<float>(camera_info_msg->k[r * 3 + c]);
                    cam_->R(r, c) = static_cast<float>(camera_info_msg->r[r * 3 + c]);
                    cam_->P(r, c) = static_cast<float>(camera_info_msg->p[r * 4 + c]);
                }
                cam_->K(r, 3) = static_cast<float>(camera_info_msg->p[r * 4 + 3]);
            }

            for (size_t c = 0; c < 4; c++)
            {
                cam_->dist[c] = static_cast<float>(camera_info_msg->d[c]);
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Camera parameters acquired.");
        }
    }

    void VxsSensorSubscriber::DepthImageCB(const sensor_msgs::msg::Image::SharedPtr depth_img_msg)
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