/**
 * @file vx_sensor.hpp
 * @author George Terzakis (george.terzakis.ext@voxelsensors.com)
 * @brief VXS sesnor data publishing node
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef VX_SENSOR_HPP
#define VX_SENSOR_HPP

#include <condition_variable>
#include <memory>
#include <thread>
#include <string>

#include <opencv2/core.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <SDK2.h>

using namespace std::chrono_literals;

namespace vxs_ros
{
    struct CameraCalibration
    {
        float fx, fy, cx, cy;
        cv::Vec<float, 5> dist = {0, 0, 0, 0, 0};
        cv::Vec3f t = {0, 0, 0, 0};
        cv::Matx<float, 3, 3> R = cv::Matx<float, 3, 3>::eye();
    };

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
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        //! FPS
        int fps_;
        //! config json
        std::string config_json_;
        //! calibration json
        std::string calib_json_;

        //! Return number of cameras by initialization
        int num_cams_;

        //! Sensor mutex
        std::mutex sensor_mutex_;

        //! Condition variable for the sensor polling thread
        std::condition_variable cvar_sensor_poll_;

        //! Shut down request flag
        bool flag_shutdown_request_;
        //! Flag indicating execution is inside the polling loop.
        bool flag_in_polling_loop_;

        //! Camera #1 calibration
        CameraCalibration cam1_;
        //! Camera #2 calibration
        CameraCalibration cam2_;

        bool InitSensor();
        void TimerCB();
        void FramePublisherLoop();
        void FramePollingLoop();
        cv::Mat UnpackSensorData(float *frameXYZ);
    };

} // end namespace vxs_ros

#endif