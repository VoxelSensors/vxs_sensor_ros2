/**
 * @file vx_sensor.hpp
 * @author George Terzakis (george.terzakis.ext@voxelsensors.com)
 * @brief VXS sesnor data publishing node
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef VXS_SENSOR_HPP
#define VXS_SENSOR_HPP

#include <condition_variable>
#include <memory>
#include <thread>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <SDK2.h>

using namespace std::chrono_literals;

namespace vxs_ros
{
    struct CameraCalibration;
    class VxsSensorPublisher : public rclcpp::Node
    {

    public:
        //! Sensor dimensions here. @TODO: Should be able to get that from the SDK?
        static const int SENSOR_WIDTH = 300;
        static const int SENSOR_HEIGHT = 300;

        VxsSensorPublisher();
        ~VxsSensorPublisher();

    private:
        //! Frame publishing thread
        std::shared_ptr<std::thread> frame_publishing_thread_;
        //! Frame polling thread
        std::shared_ptr<std::thread> frame_polling_thread_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcloud_publisher_;

        //! FPS
        int fps_;
        //! Frame/streaming window in msec
        uint23_t period_;

        //! config json
        std::string config_json_;
        //! calibration json
        std::string calib_json_;

        //! Publish depth image
        bool publish_depth_image_;

        //! Publish pointcloud
        bool publish_pointcloud_;

        //! Publish events flag. This should override depth + simpple pointcloud publishers
        bool pubish_events_;

        //! Shut down request flag
        bool flag_shutdown_request_;
        //! Flag indicating execution is inside the polling loop.
        bool flag_in_polling_loop_;

        //! Camera #1 calibration
        std::vector<CameraCalibration> cams_;

        //! Initializae sensor
        bool InitSensor();
        //! The main loop of the frame ppolling thread
        void FramePollingLoop();
        //! Unpack sensor data into a cv::Mat and return 3D points
        cv::Mat UnpackFrameSensorData(float *frameXYZ, std::vector<cv::Vec3f> &points);

        //! Load calilbration from json (required for the formation of the depth map)
        void LoadCalibrationFromJson(const std::string &calib_json);
        //! Publish image and calibration
        void PublishDepthImage(const cv::Mat &depth_image);
        //! Publish a pointcloud
        void PublishPointcloud(const std::vector<cv::Vec3f> &points);
    };

} // end namespace vxs_ros

#endif