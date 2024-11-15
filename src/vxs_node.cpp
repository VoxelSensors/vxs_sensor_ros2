#include <chrono>
#include <functional>

#include <vxs_node.hpp>

namespace vxs_ros
{
    VxsSensorPublisher::VxsSensorPublisher() :                                 //
                                               Node("vxs_sensor"),             //
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

        // Load calibration into members
        LoadCalibrationFromJson(calib_json_);

        // Initialize Sensor
        if (!InitSensor())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Sensor initialization failed!");
            rclcpp::shutdown();
        }

        // Create depth image puiblisher
        depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);

        // Initialize & start polling thread
        RCLCPP_INFO_STREAM(this->get_logger(), "Starting publisher thread...");
        frame_polling_thread_ = std::make_shared<std::thread>(std::bind(&VxsSensorPublisher::FramePollingLoop, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Done!");

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

    void VxsSensorPublisher::FramePollingLoop()
    {
        flag_in_polling_loop_ = true;
        int counter = 0;
        while (!flag_shutdown_request_)
        {
            // Wait until data ready
            while (!vxsdk::vxCheckForData())
            {
            }
            // Get data from the sensor
            float *frameXYZ = vxsdk::vxGetFrameXYZ();
            counter++;
            // Extract frame
            // RCLCPP_INFO_STREAM(this->get_logger(), "Unpacking frame...");
            cv::Mat frame = UnpackSensorData(frameXYZ);
            // RCLCPP_INFO_STREAM(this->get_logger(), "DONE");
            //  Publish sensor data as a depth image
            PublishDepthImage(frame);
        }
        flag_in_polling_loop_ = false;
    }

    cv::Mat VxsSensorPublisher::UnpackSensorData(float *frameXYZ)
    {
        // Use cam #1 intrinsics for the depth image sensor
        const float &fx = cams_[0].K(0, 0);
        const float &fy = cams_[0].K(1, 1);
        const float &cx = cams_[0].K(0, 2);
        const float &cy = cams_[0].K(1, 2);
        cv::Mat depth(SENSOR_HEIGHT, SENSOR_WIDTH, CV_16U);
        depth = 0;
        for (size_t r = 0; r < SENSOR_HEIGHT; r++)
        {
            for (size_t c = 0; c < SENSOR_WIDTH; c++)
            {
                const float &Z = frameXYZ[(r * SENSOR_WIDTH + c) * 3 + 2];
                if (Z > 1e-5)
                {
                    const float &X = frameXYZ[(r * SENSOR_WIDTH + c) * 3];
                    const float &Y = frameXYZ[(r * SENSOR_WIDTH + c) * 3 + 1];
                    const int x = std::lround(X / Z * fx + cx);
                    const int y = std::lround(Y / Z * fy + cy);
                    //  Check for negatives and out-of-bounds
                    if (y < 0 || y > SENSOR_HEIGHT - 1 || //
                        x < 0 || x > SENSOR_WIDTH - 1)
                    {
                        continue;
                    }

                    // Get a 16-bit approximation and save at x, y location
                    uint16_t iZ = std::lround(Z);
                    depth.at<uint16_t>(y, x) = iZ;
                }
            }
        }
        return depth;
    }

    void VxsSensorPublisher::LoadCalibrationFromJson(const std::string &calib_json)
    {
        // @TODO: Read the config to acquire number of cameras! Assuming stereo for now....
        cams_.resize(2);
        cv::FileStorage fs(calib_json, 0);
        cv::FileNode root = fs["Cameras"];
        cv::FileNode cam1 = root[0];
        cams_[0].t = cv::Vec3f({cam1["Translation"][0], cam1["Translation"][1], cam1["Translation"][2]});           //
        cams_[0].R = cv::Matx<float, 3, 3>({cam1["Rotation"][0][0], cam1["Rotation"][0][1], cam1["Rotation"][0][2], //
                                            cam1["Rotation"][1][0], cam1["Rotation"][1][1], cam1["Rotation"][1][2], //
                                            cam1["Rotation"][2][0], cam1["Rotation"][2][1], cam1["Rotation"][2][2]});
        cams_[0].dist = cv::Vec<float, 5>({cam1["Distortion"][0], cam1["Distortion"][1], cam1["Distortion"][2], cam1["Distortion"][3], cam1["Distortion"][4]});
        cams_[0].K = cv::Matx<float, 3, 3>({cam1["Intrinsic"][0][0], cam1["Intrinsic"][0][1], cam1["Intrinsic"][0][2], //
                                            cam1["Intrinsic"][1][0], cam1["Intrinsic"][1][1], cam1["Intrinsic"][1][2], //
                                            cam1["Intrinsic"][2][0], cam1["Intrinsic"][2][1], cam1["Intrinsic"][2][2]});
        cams_[0].image_size = cv::Size_<int>(cam1["SensorSize"]["Width"], cam1["SensorSize"]["Height"]);

        cv::FileNode cam2 = root[1];
        cams_[1].t = cv::Vec3f({cam2["Translation"][0], cam2["Translation"][1], cam2["Translation"][2]});           //
        cams_[1].R = cv::Matx<float, 3, 3>({cam2["Rotation"][0][0], cam2["Rotation"][0][1], cam2["Rotation"][0][2], //
                                            cam2["Rotation"][1][0], cam2["Rotation"][1][1], cam2["Rotation"][1][2], //
                                            cam2["Rotation"][2][0], cam2["Rotation"][2][1], cam2["Rotation"][2][2]});
        cams_[1].dist = cv::Vec<float, 5>({cam2["Distortion"][0], cam2["Distortion"][1], cam2["Distortion"][2], cam2["Distortion"][3], cam1["Distortion"][4]});
        cams_[1].K = cv::Matx<float, 3, 3>({cam2["Intrinsic"][0][0], cam2["Intrinsic"][0][1], cam2["Intrinsic"][0][2], //
                                            cam2["Intrinsic"][1][0], cam2["Intrinsic"][1][1], cam2["Intrinsic"][1][2], //
                                            cam2["Intrinsic"][2][0], cam2["Intrinsic"][2][1], cam2["Intrinsic"][2][2]});
        cams_[1].image_size = cv::Size_<int>(cam2["SensorSize"]["Width"], cam2["SensorSize"]["Height"]);

        RCLCPP_INFO_STREAM(this->get_logger(), "cam1 K: " << cams_[0].K);
    }

    void VxsSensorPublisher::PublishDepthImage(const cv::Mat &depth_image)
    {
        // cv_bridge::CvImagePtr cv_ptr;
        //  NOTE: See http://docs.ros.org/en/lunar/api/cv_bridge/html/c++/cv__bridge_8cpp_source.html
        //        for image encoding constants in cv_bridge
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(                       //
                std_msgs::msg::Header(),              //
                sensor_msgs::image_encodings::MONO16, //
                depth_image)
                .toImageMsg();
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing image...");
        depth_publisher_->publish(*msg.get());
    }

} // end namespace vxs_ros