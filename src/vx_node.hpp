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

#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <SDK2.h>

using namespace std::chrono_literals;

namespace vxs_sensor
{
    class VxsSensorPublisher : public rclcpp::Node
    {

    public:
        VxsSensorPublisher();

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

        bool InitSensor();
        void TimerCB();
        void FramePublisherLoop();
        void FramePollingLoop();
    };

} // end namespace vxs_sensor

#endif