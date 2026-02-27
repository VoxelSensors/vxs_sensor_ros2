#ifndef VXS_SAFETY_DRIVER_HPP
#define VXS_SAFETY_DRIVER_HPP

#include <memory>
#include <thread>
#include <string>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <SDK2.h>

using namespace std::chrono_literals;

namespace vxs_ros
{
    struct FilteringParams
    {
        static const int DEFAULT_BINNING = 0;
        static const float DEFAULT_PREFILTERING_THRESH;
        static const int DEFAULT_POSTFILTERING_THRESH = 5;

        static const float DEFAULT_FILTERP1X;
        static const float DEFAULT_FILTERP1Y;
        static const int DEFAULT_TEMPORAL_THRESH = 4;
        static const int DEFAULT_SPATIAL_THRESH = 10;

        static const int DEFAULT_MEDIAN_REJECTION_THRESH = 5;

        int binning_amount = DEFAULT_BINNING;
        float prefiltering_threshold = DEFAULT_PREFILTERING_THRESH;
        int postfiltering_threshold = DEFAULT_POSTFILTERING_THRESH;

        float filterP1X = DEFAULT_FILTERP1X;
        float filterP1Y = DEFAULT_FILTERP1Y;
        int temporal_threshold = DEFAULT_TEMPORAL_THRESH;
        int spatial_threshold = DEFAULT_SPATIAL_THRESH;

        int median_rejection_threshold = DEFAULT_MEDIAN_REJECTION_THRESH;
    };

    class VxsSafetyDriver : public rclcpp::Node
    {
    public:
        VxsSafetyDriver();
        ~VxsSafetyDriver();

    private:
        std::shared_ptr<std::thread> frame_polling_thread_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr evcloud_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr reflex_pub_;

        int fps_;
        uint32_t period_;

        int on_time_;
        int period_time_;

        std::string config_json_;
        std::string calib_json_;

        std::atomic<bool> flag_shutdown_request_;
        FilteringParams filtering_params_;
        int sleep_time_ms_;
        bool publish_events_;

        bool InitSensor();
        void FramePollingLoop();
        void ProcessAndPublish(const int N, vxsdk::vxXYZT *eventsXYZT);

        // Your requested standalone debug function
        void PrintDebugPoints(const int N, vxsdk::vxXYZT *eventsXYZT);
    };
}

#endif // VXS_SAFETY_DRIVER_HPP