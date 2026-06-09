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
#include "sensor_msgs/msg/imu.hpp"
#include <cv_bridge/cv_bridge.h>

#include "vxs_sensor_ros2/srv/update_observation_window.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <SDK2.h>
#include <vxEmb.h>

using namespace std::chrono_literals;

namespace imu
{
    struct IMUSample;
}
namespace vxs_ros
{
    enum class TSensorFrame
    {
        EventsXYZT = 0,
        FrameXYZ
    };

    struct RawSensorFrame
    {
        //! number of bytes in the raw frame
        int N;
        //! Number of strruct/float entries
        int num_entries;
        //! Sensor frame type (events XYZT/frame XYZ)
        TSensorFrame frame_type;
        //! The frame data as a strteam of bytes
        std::shared_ptr<std::vector<uint8_t>> data;
        //! The global (ROS) stamp
        rclcpp::Time ros_stamp;
    };

    struct CameraCalibration;

    //! Filtering parameters
    struct FilteringParams
    {
        static const int DEFAULT_BINNING = 0;
        static const float DEFAULT_PREFILTERING_THRESH;    // = 2.0;
        static const int DEFAULT_POSTFILTERING_THRESH = 5; //

        static const float DEFAULT_FILTERP1X; // = 0.1;
        static const float DEFAULT_FILTERP1Y; // = 0.1;
        static const int DEFAULT_TEMPORAL_THRESH = 4;
        static const int DEFAULT_SPATIAL_THRESH = 10;

        static const int DEFAULT_MEDIAN_REJECTION_THRESH = 5; //

        int binning_amount = DEFAULT_BINNING;
        float prefiltering_threshold = DEFAULT_PREFILTERING_THRESH;
        int postfiltering_threshold = DEFAULT_POSTFILTERING_THRESH;

        float filterP1X = DEFAULT_FILTERP1X;
        float filterP1Y = DEFAULT_FILTERP1Y;
        int temporal_threshold = DEFAULT_TEMPORAL_THRESH;
        int spatial_threshold = DEFAULT_SPATIAL_THRESH;

        int median_rejection_threshold = DEFAULT_MEDIAN_REJECTION_THRESH;
    };

    class VxsSensorPublisher : public rclcpp::Node
    {

    public:
        //! Use this to convert long int to a double timestamp in seconds
        static constexpr double PERIOD_75_MHZ = 13.3333 * 1e-9;

        //! Default RGB dimensions
        // static const int DEFAULT_RGB_WIDTH = 640;
        // static const int DEFAULT_RGB_HEIGHT = 480;

        //! Maximum depth of pubishing queue
        static const int MAX_QUEUE_DEPTH = 100;

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
        //! The check-for-data timer thread that wakes up the polling condition variable
        std::shared_ptr<std::thread> timer_polling_thread_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcloud_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr evcloud_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

        //! Observation window service
        rclcpp::Service<vxs_sensor_ros2::srv::UpdateObservationWindow>::SharedPtr update_observation_window_service_;

        //! Embedded triangulation comms object
        std::shared_ptr<vxEmb> emb_comms_;

        //! FPS
        int fps_;
        //! Frame/streaming window in msec
        uint32_t period_;

        //! The latest HW depth stamp
        double latest_depth_stamp_;

        //! config json
        std::string config_json_;
        //! calibration json
        std::string calib_json_;

        //! Publish depth image
        bool publish_depth_image_;

        //! Publish pointcloud
        bool publish_pointcloud_;

        //! Publish events flag. This should override depth + simpple pointcloud publishers
        bool publish_events_;

        //! Publish imu samples (if available)
        bool publish_imu_;

        //! Enable embedded triangulation SDK mode
        bool embedded_triangulation_mode_;

        //! Embedded triangulation lookup tables
        std::string lookup_table1_ = "";
        std::string lookup_table2_ = "";

        //! Shut down request flag
        bool flag_shutdown_request_;
        //! Flag indicating execution is inside the polling loop.
        bool flag_in_polling_loop_;
        //! data-ready flag
        std::atomic<bool> flag_data_ready_;

        //! Camera #1 calibration
        std::vector<CameraCalibration> cams_;

        //! Filtering parameters
        FilteringParams filtering_params_;

        //! Queue of raw frames (wheth)
        std::queue<RawSensorFrame> frame_queue_;
        std::mutex frame_queue_mutex_;
        std::condition_variable frame_queue_cv_;
        //! Sensor condition variable and mutex
        std::mutex sensor_mutex_;
        std::condition_variable sensor_cv_;

        //! Reference ros Time for both frames and imu samples.
        rclcpp::Time ref_time_;
        //! Reference time in the sensor
        double sensor_ref_time_;
        //! Flag indicating that reference time is initialized
        bool flag_ref_time_initialized_;

        //! A flag forcing update of the observation window wit the cached values
        std::atomic<bool> flag_update_observation_window_;
        //! observation window parameters
        int on_time_, period_time_;
        //! Mainloop sleep time
        int sleep_time_ms_;

        //! Mutex for reference time members
        std::shared_timed_mutex ref_time_mutex_;

        //! Initializae sensor
        bool InitSensor();
        //! Get the available sensor data (frame or streaming based)
        void *GetNextSensorFrame(int &N);
        //! The main loop of the frame ppolling thread
        void FramePollingLoop();
        //! Asynchronous publishing
        void SensorPublishingLoop();

        //! Unpack sensor data into a cv::Mat and return 3D points
        cv::Mat UnpackFrameSensorData(float *frameXYZ, std::vector<cv::Vec3f> &points);

        //! Load calilbration from json (required for the formation of the depth map)
        void LoadCalibrationFromJson(const std::string &calib_json);
        //! Publish image and calibration
        void PublishDepthImage(const cv::Mat &depth_image, const rclcpp::Time &stamp);
        //! Publish a pointcloud
        void PublishPointcloud(const std::vector<cv::Vec3f> &points, const rclcpp::Time &stamp);
        //! Pubish stamped pointcloud
        void PublishStampedPointcloud(const int N, vxsdk::vxXYZT *eventsXYZT, const rclcpp::Time &cloud_stamp);
        //! Publish an IMU sample
        void PublishIMUSample(const imu::IMUSample &sample, const rclcpp::Time &stamp);
        //! Timer with internal sleep wake-up the frame-polling condition variable
        void TimerPollingLoop();
    };

} // end namespace vxs_ros

#endif