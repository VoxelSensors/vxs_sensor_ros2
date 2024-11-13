#include <chrono>
#include <functional>

#include <vxs_node.hpp>

namespace vxs_ros
{
    VxsSensorPublisher::VxsSensorPublisher() :                                 //
                                               Node("vxs_sensor"),             //
                                               num_cams_(0),                   //
                                               frame_polling_thread_(nullptr), //
                                               flag_shutdown_request_(false)
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("vxs_sensor");
        RCLCPP_INFO_STREAM(this->get_logger(), "Package share directory: " << package_share_directory);
        // Declare & Get parameters
        this->declare_parameter("fps", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("config_json", rclcpp::PARAMETER_STRING);
        this->declare_parameter("calib_json", rclcpp::PARAMETER_STRING);

        // Retrieve params
        rclcpp::Parameter fps_param;
        if (!this->get_parameter("fps", fps_param))
        {
            fps_ = 30;
            RCLCPP_INFO_STREAM(this->get_logger(), "Fps not specified. Using " << fps_ << " frames per second.");
        }
        else
        {
            fps_ = fps_param.as_int();
            RCLCPP_INFO_STREAM(this->get_logger(), "Fps set to " << fps_);
        }

        rclcpp::Parameter config_json_param;
        if (!this->get_parameter("config_json", config_json_param))
        {
            config_json_ = "config/and2_median_golden.json";
            RCLCPP_INFO_STREAM(this->get_logger(), "Config JSON not specified. Using default: " << config_json_);
        }
        else
        {
            config_json_ = config_json_param.as_string();
            RCLCPP_INFO_STREAM(this->get_logger(), "Config JSON is " << config_json_);
        }

        rclcpp::Parameter calib_json_param;
        if (!this->get_parameter("calib_json", calib_json_param))
        {
            calib_json_ = "config/default_calib.json";
            RCLCPP_INFO_STREAM(this->get_logger(), "Calibration JSON not specified. Using default: " << calib_json_);
        }
        else
        {
            calib_json_ = calib_json_param.as_string();
            RCLCPP_INFO_STREAM(this->get_logger(), "Calibration JSON is " << calib_json_);
        }

        // Initialize Sensor
        if (!InitSensor())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Sensor initialization failed!");
            rclcpp::shutdown();
        }

        // Initialize & start polling thread
        frame_polling_thread_ = std::make_shared<std::thread>(std::bind(&VxsSensorPublisher::FramePollingLoop, this));

        publisher_ = this->create_publisher<std_msgs::msg::String>("vxs_data", 10);
        timer_ = this->create_wall_timer(     //
            500ms,                            //
            std::bind(                        //
                &VxsSensorPublisher::TimerCB, //
                this)                         //
        );
    }

    VxsSensorPublisher::~VxsSensorPublisher()
    {
        if (frame_polling_thread_)
        {
            if (frame_polling_thread_->joinable())
            {
                frame_polling_thread_->join();
            }
        }
    }

    void VxsSensorPublisher::TimerCB()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! "; // + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    bool VxsSensorPublisher::InitSensor()
    {
        vxsdk::pipelineType pipeline_type = vxsdk::pipelineType::fbPointcloud;

        // Set the frame rate
        vxsdk::vxSetFPS(fps_);

        // Start the SDK Engine.
        int cam_num = vxsdk::vxStartSystem( //
            config_json_.c_str(),           //
            calib_json_.c_str(),            //
            pipeline_type);

        return cam_num > 0;
    }

    void VxsSensorPublisher::FramePublisherLoop()
    {
        while (!flag_shutdown_request_)
        {
            // @TODO: Publish depth images and sensor params here...
        }
    }

    void VxsSensorPublisher::FramePollingLoop()
    {
        flag_in_polling_loop_ = true;
        while (!flag_shutdown_request_)
        {
            // @TODO: Poll sensor and and hold back...

            std::unique_lock<std::mutex> sensor_lock(sensor_mutex_);
            // Block the thread if no data is available from the sensor
            cvar_sensor_poll_.wait(
                sensor_lock,
                [this]()
                { return vxsdk::vxCheckForData() || flag_shutdown_request_; });
            if (flag_shutdown_request_)
            {
                sensor_lock.unlock();
                continue;
            }

            // Get data from the sensor
            float *frameXYZ = vxsdk::vxGetFrameXYZ();

            // Unlock the queue now/ for subsequent processing
            sensor_lock.unlock();

            // @TODO: Maybe add data to queue or simply publish here...
        }
        flag_in_polling_loop_ = false;
    }

} // end namespace vxs_ros