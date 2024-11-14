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
        flag_shutdown_request_ = true;
        cvar_sensor_poll_.notify_all();
        if (frame_polling_thread_)
        {
            if (frame_polling_thread_->joinable())
            {
                frame_polling_thread_->join();
            }
        }
        frame_polling_thread_ = nullptr;
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

    /*
    void VxsSensorPublisher::FramePollingLoop()
    {
        flag_in_polling_loop_ = true;
        int counter = 0;
        while (!flag_shutdown_request_)
        {
            // @TODO: Poll sensor and and hold back...

            std::unique_lock<std::mutex> sensor_lock(sensor_mutex_);
            //  Block the thread if no data is available from the sensor
            bool data_ready = false;

            // while (!(data_ready = vxsdk::vxCheckForData()))
            //{
            // }
            counter++;
            RCLCPP_INFO_STREAM(this->get_logger(), "Data ready: " << (data_ready ? "YES" : "NO"));
            RCLCPP_INFO_STREAM(this->get_logger(), "Counter: " << counter);

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
            RCLCPP_INFO_STREAM(this->get_logger(), "Received frame!");
            // Unlock the queue now/ for subsequent processing
            sensor_lock.unlock();

            // @TODO: Maybe add data to queue or simply publish here...
        }
        flag_in_polling_loop_ = false;
    }
    */
    void VxsSensorPublisher::FramePollingLoop()
    {
        flag_in_polling_loop_ = true;
        int counter = 0;
        while (!flag_shutdown_request_)
        {
            // @TODO: Poll sensor and and hold back...

            //  Wait until data ready
            while (!vxsdk::vxCheckForData())
            {
            }

            // Get data from the sensor
            float *frameXYZ = vxsdk::vxGetFrameXYZ();
            counter++;

            RCLCPP_INFO_STREAM(this->get_logger(), "Received frame!");

            // @TODO: Maybe add data to queue or simply publish here...
        }
        flag_in_polling_loop_ = false;
    }

    cv::Mat VxsSensorPublisher::UnpackSensorData(float *frameXYZ)
    {
        // Use cam #1 intrinsics for the depth image sensor
        const float &fx = cam1_.fx;
        const float &fy = cam1_.fy;
        const float &cx = cam1_.cx;
        const float &cy = cam1_.cy;

        cv::Mat depth(SENSOR_HEIGHT, SENSOR_WIDTH, CV_16U);
        depth *= 0;
        for (size_t r = 0; r < SENSOR_HEIGHT; r++)
        {
            for (size_t c = 0; c < SENSOR_WIDTH; c++)
            {
                if ((float &Z = frameXYZ[(r * SENSOR_WIDTH + c) * 3 + 2]) > 1e-5)
                {
                    const float &X = frameXYZ[(r * SENSOR_WIDTH + c) * 3];
                    const float &Y = frameXYZ[(r * SENSOR_WIDTH + c) * 3 + 1];
                    const int x = std::lround(X / Z * fx + cx);
                    const int y = std::lround(Y / Z * fy + cy);
                    //  Check for negatives and out-of-bounds
                    if (y < 0 || y > SENSOR_HEIGHT - 1 || //
                        x < 0 || x > SENSOR_WIDTH - 1)
                    {
                        // @TODO: Get a 16-bit approximation and save at x, y location
                        continue;
                    }
                }
            }
        }
        return depth;
    }

} // end namespace vxs_ros