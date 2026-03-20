#include "safety_driver/safety_driver.hpp"
#include <chrono>
#include <functional>

namespace vxs_ros
{
    const float FilteringParams::DEFAULT_PREFILTERING_THRESH = 2.0;
    const float FilteringParams::DEFAULT_FILTERP1X = 0.1;
    const float FilteringParams::DEFAULT_FILTERP1Y = 0.1;

    VxsSafetyDriver::VxsSafetyDriver() : Node("vxs_safety_driver"),
                                         frame_polling_thread_(nullptr),
                                         flag_shutdown_request_(false)
    {
        this->declare_parameter("fps", 30);
        this->declare_parameter("config_json", "config/and2_median_golden.json");
        this->declare_parameter("calib_json", "config/default_calib.json");
        this->declare_parameter("sleep_time_ms", 1);
        this->declare_parameter("publish_events", false);
        this->declare_parameter("observation_window_on_time", 40);
        this->declare_parameter("observation_window_period_time", 200);

        // Declare the dynamic bounding box (using your current hardcoded values as defaults)
        this->declare_parameter("x_min", -10.0);
        this->declare_parameter("x_max", 10.0);
        this->declare_parameter("y_min", -20000.0);
        this->declare_parameter("y_max", 20000.0);
        this->declare_parameter("z_min", -10000.0);
        this->declare_parameter("z_max", 10000.0);

        this->declare_parameter("binning_amount", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("prefiltering_threshold", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("postfiltering_threshold", rclcpp::PARAMETER_INTEGER);

        fps_ = this->get_parameter("fps").as_int();
        config_json_ = this->get_parameter("config_json").as_string();
        calib_json_ = this->get_parameter("calib_json").as_string();
        sleep_time_ms_ = this->get_parameter("sleep_time_ms").as_int();
        publish_events_ = this->get_parameter("publish_events").as_bool();
        on_time_ = this->get_parameter("observation_window_on_time").as_int();
        period_time_ = this->get_parameter("observation_window_period_time").as_int();

        // Fetch the bounding box limits (ROS 2 uses doubles for floating point parameters)
        x_min_ = static_cast<float>(this->get_parameter("x_min").as_double());
        x_max_ = static_cast<float>(this->get_parameter("x_max").as_double());
        y_min_ = static_cast<float>(this->get_parameter("y_min").as_double());
        y_max_ = static_cast<float>(this->get_parameter("y_max").as_double());
        z_min_ = static_cast<float>(this->get_parameter("z_min").as_double());
        z_max_ = static_cast<float>(this->get_parameter("z_max").as_double());

        rclcpp::Parameter binning_amount_param;
        if (!this->get_parameter("binning_amount", binning_amount_param))
        {
            filtering_params_.binning_amount = FilteringParams::DEFAULT_BINNING;
        }
        else
        {
            filtering_params_.binning_amount = binning_amount_param.as_int();
        }

        rclcpp::Parameter prefiltering_threshold_param;
        if (!this->get_parameter("prefiltering_threshold", prefiltering_threshold_param))
        {
            filtering_params_.prefiltering_threshold = FilteringParams::DEFAULT_PREFILTERING_THRESH;
        }
        else
        {
            filtering_params_.prefiltering_threshold = prefiltering_threshold_param.as_double();
        }

        rclcpp::Parameter postfiltering_threshold_param;
        if (!this->get_parameter("postfiltering_threshold", postfiltering_threshold_param))
        {
            filtering_params_.postfiltering_threshold = FilteringParams::DEFAULT_POSTFILTERING_THRESH;
        }
        else
        {
            filtering_params_.postfiltering_threshold = postfiltering_threshold_param.as_int();
        }

        period_ = std::lround(1000.0f / fps_);

        if (!InitSensor())
        {
            RCLCPP_ERROR(this->get_logger(), "Sensor initialization failed!");
            rclcpp::shutdown();
            return;
        }

        reflex_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_reflex", 1);

        if (publish_events_)
        {
            evcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcloud/events", 10);
            RCLCPP_INFO(this->get_logger(), "Event pointcloud publishing is ENABLED. Box: X(%.1f to %.1f) Y(%.1f to %.1f) Z(%.1f to %.1f)",
                        x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
        }

        RCLCPP_INFO(this->get_logger(), "Starting Safety Driver Thread...");
        frame_polling_thread_ = std::make_shared<std::thread>(std::bind(&VxsSafetyDriver::FramePollingLoop, this));
    }

    VxsSafetyDriver::~VxsSafetyDriver()
    {
        flag_shutdown_request_ = true;
        if (frame_polling_thread_ && frame_polling_thread_->joinable())
        {
            frame_polling_thread_->join();
        }
        vxsdk::vxStopSystem();
    }

    bool VxsSafetyDriver::InitSensor()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing VoxelSensors SDK...");
        vxsdk::vxSetStreamingDuration(period_);
        vxsdk::vxSetBinningAmount(filtering_params_.binning_amount);

        int cam_num = vxsdk::vxStartSystem(config_json_.c_str(), calib_json_.c_str(), vxsdk::pipelineType::all);

        return cam_num > 0;
    }

    void VxsSafetyDriver::FramePollingLoop()
    {
        vxsdk::vxSetObservationWindow(on_time_, period_time_);

        while (!flag_shutdown_request_)
        {
            if (!vxsdk::vxCheckForData())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms_));
                continue;
            }

            int N;
            vxsdk::vxXYZT *eventsXYZT = vxsdk::vxGetXYZT(N);

            if (N > 0)
            {
                ProcessAndPublish(N, eventsXYZT);
            }
        }
    }

    void VxsSafetyDriver::PrintDebugPoints(const int N, vxsdk::vxXYZT *eventsXYZT)
    {
        static int frame_counter = 0;
        if (frame_counter++ % 20 != 0 || N <= 0)
            return;

        RCLCPP_INFO(this->get_logger(), "--- RAW SDK SIGNS (Showing 1 in 50, max 5) ---");
        int printed = 0;
        for (int i = 0; i < N; i += 50)
        {
            RCLCPP_INFO(this->get_logger(), "Index [%d] -> X: %.1f | Y: %.1f | Z: %.1f",
                        i, eventsXYZT[i].x, eventsXYZT[i].y, eventsXYZT[i].z);
            printed++;
            if (printed >= 5)
                break;
        }
    }

    void VxsSafetyDriver::ProcessAndPublish(const int N, vxsdk::vxXYZT *eventsXYZT)
    {
        int danger_points = 0;
        const int PANIC_THRESHOLD = 5;
        bool reflex_fired = false;

        std::vector<int> danger_indices;
        if (publish_events_)
        {
            danger_indices.reserve(N);
        }

        for (int i = 0; i < N; ++i)
        {
            // Fully dynamic Kill Zone!
            if (eventsXYZT[i].z > z_min_ && eventsXYZT[i].z < z_max_ &&
                eventsXYZT[i].x > x_min_ && eventsXYZT[i].x < x_max_ &&
                eventsXYZT[i].y > y_min_ && eventsXYZT[i].y < y_max_)
            {
                danger_points++;

                if (publish_events_)
                {
                    danger_indices.push_back(i);
                }
            }

            if (danger_points > PANIC_THRESHOLD && !reflex_fired)
            {
                geometry_msgs::msg::Twist reflex_cmd;
                reflex_cmd.linear.x = 0.0;
                reflex_cmd.linear.y = 0.0;
                reflex_cmd.linear.z = 0.0;
                reflex_pub_->publish(reflex_cmd);

                reflex_fired = true;

                if (!publish_events_)
                {
                    break;
                }
            }
        }

        if (publish_events_ && danger_points > 0)
        {
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            msg->header.stamp = this->get_clock()->now();
            msg->header.frame_id = "sensor";
            msg->height = 1;
            msg->width = danger_points;

            static const std::vector<sensor_msgs::msg::PointField> fields = []()
            {
                std::vector<sensor_msgs::msg::PointField> f(4);
                f[0].name = "x";
                f[0].offset = 0;
                f[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
                f[0].count = 1;
                f[1].name = "y";
                f[1].offset = 4;
                f[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
                f[1].count = 1;
                f[2].name = "z";
                f[2].offset = 8;
                f[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
                f[2].count = 1;
                f[3].name = "t";
                f[3].offset = 12;
                f[3].datatype = sensor_msgs::msg::PointField::FLOAT64;
                f[3].count = 1;
                return f;
            }();
            msg->fields = fields;

            msg->point_step = 20;
            msg->row_step = msg->point_step * msg->width;
            msg->data.resize(msg->row_step * msg->height);

            uint8_t *ptr = &msg->data[0];

            for (int idx : danger_indices)
            {
                float *point = reinterpret_cast<float *>(ptr);
                point[0] = eventsXYZT[idx].x * 0.001f;
                point[1] = eventsXYZT[idx].y * 0.001f;
                point[2] = eventsXYZT[idx].z * 0.001f;
                *(double *)(ptr + 12) = eventsXYZT[idx].timestamp;
                ptr += msg->point_step;
            }

            evcloud_publisher_->publish(std::move(msg));
        }
    }
} // end namespace vxs_ros