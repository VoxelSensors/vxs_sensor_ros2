#include <chrono>
#include <functional>

#include "publisher/vxs_node.hpp"
#include "common.hpp"
#include "imu.hpp"

namespace vxs_ros
{
    const float FilteringParams::DEFAULT_PREFILTERING_THRESH = 2.0;
    const float FilteringParams::DEFAULT_FILTERP1X = 0.1;
    const float FilteringParams::DEFAULT_FILTERP1Y = 0.1;

    VxsSensorPublisher::VxsSensorPublisher() :                                 //
                                               Node("vxs_sensor"),             //
                                               frame_polling_thread_(nullptr), //
                                               emb_comms_(nullptr),            //
                                               flag_shutdown_request_(false),  //
                                               flag_ref_time_initialized_(false)

    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("vxs_sensor_ros2");
        RCLCPP_INFO_STREAM(this->get_logger(), "Package share directory: " << package_share_directory);
        // Declare & Get parameters
        this->declare_parameter("embedded_triangulation", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("lookup_table1", rclcpp::PARAMETER_STRING);
        this->declare_parameter("lookup_table2", rclcpp::PARAMETER_STRING);

        this->declare_parameter("publish_imu", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("publish_depth_image", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("publish_pointcloud", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("publish_events", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("fps", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("config_json", rclcpp::PARAMETER_STRING);
        this->declare_parameter("calib_json", rclcpp::PARAMETER_STRING);

        // Filtering parameters
        this->declare_parameter("binning_amount", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("prefiltering_threshold", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("filterP1X", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("filterP1Y", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("temporal_threshold", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("spatial_threshold", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("median_rejection_threshold", rclcpp::PARAMETER_INTEGER);

        this->declare_parameter("observation_window_on_time", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("observation_window_period_time", rclcpp::PARAMETER_INTEGER);

        this->declare_parameter("sleep_time_ms", rclcpp::PARAMETER_INTEGER);

        // Retrieve params
        // Publish depth image
        rclcpp::Parameter embedded_triangulation_param;
        if (!this->get_parameter("embedded_triangulation", embedded_triangulation_param))
        {
            embedded_triangulation_mode_ = false;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Embedded triangulation mode enabled.");
            embedded_triangulation_mode_ = embedded_triangulation_param.as_bool();
            rclcpp::Parameter lookup_table1_param;
            if (!this->get_parameter("lookup_table1", lookup_table1_param))
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Embedded triangulation mode requires lookup tables! Please specify lookup table #1.");
                rclcpp::shutdown();
            }
            lookup_table1_ = lookup_table1_param.as_string();

            rclcpp::Parameter lookup_table2_param;
            if (!this->get_parameter("lookup_table2", lookup_table2_param))
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Embedded triangulation mode requires lookup tables! Please specify lookup table #2.");
                rclcpp::shutdown();
            }
            lookup_table2_ = lookup_table2_param.as_string();
        }

        // Publish events (XYZT)
        rclcpp::Parameter publish_events_param;
        if (!this->get_parameter("publish_events", publish_events_param))
        {
            // Support frame-based mode by default
            publish_events_ = false;
        }
        else
        {
            publish_events_ = publish_events_param.as_bool();
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "Publishing stamped point cloud (events): " << (publish_events_ ? "YES." : "NO."));

        // Publish depth image
        rclcpp::Parameter publish_depth_param;
        if (!this->get_parameter("publish_depth_image", publish_depth_param))
        {
            publish_depth_image_ = false;
        }
        else
        {
            publish_depth_image_ = publish_depth_param.as_bool();
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "Publishing depth image " << (publish_depth_image_ ? "YES." : "NO."));

        rclcpp::Parameter publish_pcloud_param;
        if (!this->get_parameter("publish_pointcloud", publish_pcloud_param))
        {
            publish_pointcloud_ = false;
        }
        else
        {
            publish_pointcloud_ = publish_pcloud_param.as_bool();
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "Publishing point cloud: " << (publish_pointcloud_ ? "YES." : "NO."));

        // FPS
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
        period_ = std::lround(1000.0f / fps_); // period in ms (will be used in initialization if streaming events)

        // Config json file
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

        // Calibration json file
        rclcpp::Parameter calib_json_param;
        if (!this->get_parameter("calib_json", calib_json_param))
        {
            calib_json_ = "config/and2_106.json";
            RCLCPP_INFO_STREAM(this->get_logger(), "Calibration JSON not specified. Using default: " << calib_json_);
        }
        else
        {
            calib_json_ = calib_json_param.as_string();
            RCLCPP_INFO_STREAM(this->get_logger(), "Calibration JSON is " << calib_json_);
        }

        rclcpp::Parameter binning_amount_param;
        if (!this->get_parameter("binning_amount", binning_amount_param))
        {
            filtering_params_.binning_amount = FilteringParams::DEFAULT_BINNING;
        }
        else
        {
            filtering_params_.binning_amount = binning_amount_param.as_int();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtering: --- Binning amount: " << filtering_params_.binning_amount);

        rclcpp::Parameter prefiltering_threshold_param;
        if (!this->get_parameter("prefiltering_threshold", prefiltering_threshold_param))
        {
            filtering_params_.prefiltering_threshold = FilteringParams::DEFAULT_PREFILTERING_THRESH;
        }
        else
        {
            filtering_params_.prefiltering_threshold = prefiltering_threshold_param.as_double();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtering: --- Prefiltering threshold: " << filtering_params_.prefiltering_threshold);

        rclcpp::Parameter postfiltering_threshold_param;
        if (!this->get_parameter("postfiltering_threshold", postfiltering_threshold_param))
        {
            filtering_params_.postfiltering_threshold = FilteringParams::DEFAULT_POSTFILTERING_THRESH;
        }
        else
        {
            filtering_params_.postfiltering_threshold = postfiltering_threshold_param.as_int();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtering: --- Postfiltering threshold: " << filtering_params_.postfiltering_threshold);

        rclcpp::Parameter filterP1X_param;
        if (!this->get_parameter("filterP1X", filterP1X_param))
        {
            filtering_params_.filterP1X = FilteringParams::DEFAULT_FILTERP1X;
        }
        else
        {
            filtering_params_.filterP1X = filterP1X_param.as_double();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtering: --- FilterP1X: " << filtering_params_.filterP1X);

        rclcpp::Parameter filterP1Y_param;
        if (!this->get_parameter("filterP1Y", filterP1Y_param))
        {
            filtering_params_.filterP1Y = FilteringParams::DEFAULT_FILTERP1Y;
        }
        else
        {
            filtering_params_.filterP1Y = filterP1Y_param.as_double();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtering: --- FilterP1Y: " << filtering_params_.filterP1Y);

        rclcpp::Parameter temporal_threshold_param;
        if (!this->get_parameter("temporal_threshold", temporal_threshold_param))
        {
            filtering_params_.temporal_threshold = FilteringParams::DEFAULT_TEMPORAL_THRESH;
        }
        else
        {
            filtering_params_.temporal_threshold = temporal_threshold_param.as_int();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtering: --- Temporal threshold: " << filtering_params_.temporal_threshold);

        rclcpp::Parameter spatial_threshold_param;
        if (!this->get_parameter("spatial_threshold", spatial_threshold_param))
        {
            filtering_params_.spatial_threshold = FilteringParams::DEFAULT_SPATIAL_THRESH;
        }
        else
        {
            filtering_params_.spatial_threshold = spatial_threshold_param.as_int();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtering: --- Spatial threshold: " << filtering_params_.spatial_threshold);

        rclcpp::Parameter median_rejection_threshold_param;
        if (!this->get_parameter("median_rejection_threshold", median_rejection_threshold_param))
        {
            filtering_params_.median_rejection_threshold = FilteringParams::DEFAULT_MEDIAN_REJECTION_THRESH;
        }
        else
        {
            filtering_params_.median_rejection_threshold = median_rejection_threshold_param.as_int();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Filtering: --- Median rejection threshold: " << filtering_params_.median_rejection_threshold);

        rclcpp::Parameter publish_imu_param;
        if (!this->get_parameter("publish_imu", publish_imu_param))
        {
            publish_imu_ = false;
        }
        else
        {
            publish_imu_ = publish_imu_param.as_bool();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Publish IMU samples: " << (publish_imu_ ? "YES" : "NO"));

        rclcpp::Parameter observation_window_on_time_param, observation_window_period_time_param;
        if (this->get_parameter("observation_window_on_time", observation_window_on_time_param))
        {
            if (this->get_parameter("observation_window_period_time", observation_window_period_time_param))
            {
                on_time_ = observation_window_on_time_param.as_int();
                period_time_ = observation_window_period_time_param.as_int();
                flag_update_observation_window_ = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "Observation window SET(on_time, period_time) = (" << on_time_ << ", " << period_time_ << ").");
            }
        }
        else
        {
            flag_update_observation_window_ = false;
            RCLCPP_INFO_STREAM(this->get_logger(), "Observation window set to DEFAULT. ");
        }

        rclcpp::Parameter sleep_time_ms_param;
        if (!this->get_parameter("sleep_time_ms", sleep_time_ms_param))
        {
            sleep_time_ms_ = 1;
        }
        else
        {
            sleep_time_ms_ = sleep_time_ms_param.as_int();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Thread sleep time set to " << sleep_time_ms_ << "ms.");

        // Do some logic to resolve conflicting flags regarding frame-based and/or event/streaming/timestamped mode
        if (publish_events_)
        {
            // Disable both standard pointcloud and depth image publishing
            publish_depth_image_ = publish_pointcloud_ = false;
            RCLCPP_INFO_STREAM(this->get_logger(), "Streaming mode (event based) enabled. Disabling depth and standard pointcloud poublishers.");
        }
        else
        {
            // Force pointcloud publishing by default if running frame based mode
            if (!publish_depth_image_ && !publish_pointcloud_)
            {
                publish_depth_image_ = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "Running frame based mode. Enabling pointcloud publisher...");
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Pointcloud publisher: " << (publish_pointcloud_ ? "ENABLED." : "DISABLED."));
            RCLCPP_INFO_STREAM(this->get_logger(), "Depth image publisher: " << (publish_depth_image_ ? "ENABLED." : "DISABLED."));

            if (publish_imu_)
            {
                publish_imu_ = false;
                RCLCPP_INFO_STREAM(this->get_logger(), "IMU sample will **NOT** be published in frame mode... ");
            }
        }

        // Load calibration into members
        LoadCalibrationFromJson(calib_json_);

        // Initialize Sensor
        if (!InitSensor())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Sensor initialization failed!");
            rclcpp::shutdown();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Done.");

        // Create publishers
        depth_publisher_ = publish_depth_image_ ? this->create_publisher<sensor_msgs::msg::Image>("depth/image", 10) : nullptr;

        pcloud_publisher_ = publish_pointcloud_ ? this->create_publisher<sensor_msgs::msg::PointCloud2>("pcloud/cloud", 10) : nullptr;

        evcloud_publisher_ = publish_events_ ? this->create_publisher<sensor_msgs::msg::PointCloud2>("pcloud/events", 10) : nullptr;

        cam_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("sensor/camera_info", 10);

        imu_publisher_ = publish_imu_ ? this->create_publisher<sensor_msgs::msg::Imu>("imu", 10) : nullptr;

        // The observation window service
        // Create the freeze service
        update_observation_window_service_ = this->create_service<vxs_sensor_ros2::srv::UpdateObservationWindow>( //
            "update_observation_window",                                                                          //
            [this](const std::shared_ptr<vxs_sensor_ros2::srv::UpdateObservationWindow::Request> req,             //
                   std::shared_ptr<vxs_sensor_ros2::srv::UpdateObservationWindow::Response> res)
            {
                //! No boundary checking until values have been confirmed in SDK
                on_time_ = req->on_time;
                period_time_ = req->period_time;
                res->status_message = "vxs_node: Updating observation window...";
                res->success = true;
                flag_update_observation_window_ = true;
            });

        // Initialize & start polling thread
        RCLCPP_INFO_STREAM(this->get_logger(), "Starting publisher thread...");
        frame_polling_thread_ = std::make_shared<std::thread>(std::bind(&VxsSensorPublisher::FramePollingLoop, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Done!");
    }

    VxsSensorPublisher::~VxsSensorPublisher()
    {
        flag_shutdown_request_ = true;
        if (frame_polling_thread_)
        {
            if (frame_polling_thread_->joinable())
            {
                frame_polling_thread_->join();
            }
        }
        frame_polling_thread_ = nullptr;
        vxsdk::vxStopSystem();
    }

    bool VxsSensorPublisher::InitSensor()
    {
        static constexpr uint32_t transfer_size = 2 * 1024 * 1024;
        if (embedded_triangulation_mode_)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Initializing embedded triamgulation comms mode.");
            emb_comms_ = std::make_shared<vxEmb>();
            return emb_comms_->startSystem(    //
                       config_json_.c_str(),   //
                       lookup_table1_.c_str(), //
                       lookup_table2_.c_str(), //
                       transfer_size) > 0;
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Initializing standard SDK comms.");

        // Set the frame rate (or time window)
        vxsdk::pipelineType pipeline_type;
        if (publish_events_)
        {
            pipeline_type = vxsdk::pipelineType::all; // Get everything out XYT-XYT pairs and XYZT
            vxsdk::vxSetStreamingDuration(period_);
        }
        else
        {
            pipeline_type = vxsdk::pipelineType::fbPointcloud;
            vxsdk::vxSetFPS(fps_);
        }

        // Set filtering parameters
        vxsdk::vxSetBinningAmount(filtering_params_.binning_amount);
        vxsdk::vxSetFilteringParameters(                  //
            filtering_params_.prefiltering_threshold,     //
            filtering_params_.postfiltering_threshold,    //
            filtering_params_.median_rejection_threshold, //
            filtering_params_.filterP1X,                  //
            filtering_params_.filterP1Y,                  //
            filtering_params_.temporal_threshold,         //
            filtering_params_.spatial_threshold);

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
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms_));
            }
            if (publish_events_) // streaming based publishing
            {
                int N;
                vxsdk::vxXYZT *eventsXYZT = vxsdk::vxGetXYZT(N);
                PublishStampedPointcloud(N, eventsXYZT);
            }
            else // Frame based data
            {
                // Get data from the sensor
                float *frameXYZ = vxsdk::vxGetFrameXYZ();
                counter++;
                // Extract frame
                std::vector<cv::Vec3f> points;
                cv::Mat frame = UnpackFrameSensorData(frameXYZ, points);
                //   Publish sensor data as a depth image
                if (publish_depth_image_)
                {
                    PublishDepthImage(frame);
                }
                if (publish_pointcloud_)
                {
                    PublishPointcloud(points);
                }
            }

            // Check for imu samples
            if (publish_imu_)
            {
                std::vector<imu::IMUSample> imu_samples;
                int num_samples;
                vxsdk::vxIMU *sample_ptr = vxsdk::vxGetIMU(num_samples);
                for (int i = 0; i < num_samples; i++)
                {
                    imu_samples.emplace_back(*sample_ptr);
                    sample_ptr++;
                }
                if (num_samples > 0)
                {
                    // Check if reference time is initialized. @TODO: It should not jappen because IMU is available only in streaming mode
                    {
                        std::unique_lock<std::shared_timed_mutex> lock(ref_time_mutex_);
                        if (!flag_ref_time_initialized_)
                        {
                            const double ref_time_secs = this->get_clock()->now().seconds() - (imu_samples.rbegin()->stamp_seconds - imu_samples[0].stamp_seconds);
                            const int32_t ref_time_sec_part = static_cast<int32_t>(ref_time_secs);
                            const int64_t ref_time_nsec_part = static_cast<int64_t>((ref_time_secs - ref_time_sec_part) * 1e9);
                            ref_time_ = rclcpp::Time(ref_time_sec_part, ref_time_nsec_part);
                            sensor_ref_time_ = imu_samples[0].stamp_seconds;
                            flag_ref_time_initialized_ = true;
                        }
                    }
                    // Now publish imu readings
                    for (int i = 0; i < num_samples; i++)
                    {
                        PublishIMUSample(imu_samples[i]);
                    }
                }
            }
            // Check for observation window update
            if (flag_update_observation_window_)
            {
                vxsdk::vxSetObservationWindow(on_time_, period_time_);
                flag_update_observation_window_ = false;
            }
        }
        flag_in_polling_loop_ = false;
    }

    cv::Mat VxsSensorPublisher::UnpackFrameSensorData(float *frameXYZ, std::vector<cv::Vec3f> &points)
    {
        // Use cam #1 intrinsics for the depth image sensor
        const float &fx = cams_[0].K(0, 0);
        const float &fy = cams_[0].K(1, 1);
        const float &cx = cams_[0].K(0, 2);
        const float &cy = cams_[0].K(1, 2);
        cv::Mat depth(SENSOR_HEIGHT, SENSOR_WIDTH, CV_16U);
        depth = 0;
        points.clear();
        for (size_t r = 0; r < SENSOR_HEIGHT; r++)
        {
            for (size_t c = 0; c < SENSOR_WIDTH; c++)
            {
                const float &Z = frameXYZ[(r * SENSOR_WIDTH + c) * 3 + 2];

                if (Z > 1e-5)
                {

                    const float &X = frameXYZ[(r * SENSOR_WIDTH + c) * 3];
                    const float &Y = frameXYZ[(r * SENSOR_WIDTH + c) * 3 + 1];

                    // Keep the point, irrespective of visibility on sensor (it shouldn't be happening though...)
                    points.emplace_back(X, Y, Z);

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
    }

    void VxsSensorPublisher::PublishDepthImage(const cv::Mat &depth_image)
    {
        // cv_bridge::CvImagePtr cv_ptr;
        //  NOTE: See http://docs.ros.org/en/lunar/api/cv_bridge/html/c++/cv__bridge_8cpp_source.html
        //        for image encoding constants in cv_bridge
        auto depth_header = std_msgs::msg::Header();
        depth_header.stamp = this->get_clock()->now();
        depth_header.frame_id = "sensor";
        sensor_msgs::msg::Image::SharedPtr depth_image_msg =
            cv_bridge::CvImage(                       //
                depth_header,                         //
                sensor_msgs::image_encodings::MONO16, //
                depth_image)
                .toImageMsg();

        // Create camera info message
        sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();

        cam_info_msg->header.stamp = depth_image_msg->header.stamp;
        cam_info_msg->header = std_msgs::msg::Header();
        cam_info_msg->header.stamp = depth_image_msg->header.stamp;

        cam_info_msg->header.frame_id = depth_header.frame_id;
        cam_info_msg->width = depth_image.cols;
        cam_info_msg->height = depth_image.rows;
        cam_info_msg->distortion_model = "plumn_bob";

        cam_info_msg->d = {cams_[0].dist[0], cams_[0].dist[1], cams_[0].dist[2], cams_[0].dist[3], cams_[0].dist[4]};
        cam_info_msg->k = {                                                      //
                           cams_[0].K(0, 0), cams_[0].K(0, 1), cams_[0].K(0, 2), //
                           cams_[0].K(1, 0), cams_[0].K(1, 1), cams_[0].K(1, 2), //
                           cams_[0].K(2, 0), cams_[0].K(2, 1), cams_[0].K(2, 2)};
        cam_info_msg->r = {                                                      //
                           cams_[0].R(0, 0), cams_[0].R(0, 1), cams_[0].R(0, 2), //
                           cams_[0].R(1, 0), cams_[0].R(1, 1), cams_[0].R(1, 2), //
                           cams_[0].R(2, 0), cams_[0].R(2, 1), cams_[0].R(2, 2)};

        cam_info_msg->p = {                                                         //
                           cams_[0].K(0, 0), cams_[0].K(0, 1), cams_[0].K(0, 2), 0, //
                           cams_[0].K(1, 0), cams_[0].K(1, 1), cams_[0].K(1, 2), 0, //
                           cams_[0].K(2, 0), cams_[0].K(2, 1), cams_[0].K(2, 2)};
        // publish depth image and camera info

        depth_publisher_->publish(*depth_image_msg.get());
        cam_info_publisher_->publish(*cam_info_msg.get());
    }

    void VxsSensorPublisher::PublishPointcloud(const std::vector<cv::Vec3f> &points)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Set the header
        auto pcloud_header = std_msgs::msg::Header();
        pcloud_header.stamp = this->get_clock()->now();
        pcloud_header.frame_id = "sensor";
        msg->header = pcloud_header;
        // Unordered pointcloud. Height is 1 and Width is the size (N)
        const size_t N = points.size();
        msg->height = 1;
        msg->width = N;

        // Define the point cloud fields
        sensor_msgs::msg::PointField x, y, z;
        x.name = "x";
        x.offset = 0;
        x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x.count = 1;
        y.name = "y";
        y.offset = 4;
        y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        y.count = 1;
        z.name = "z";
        z.offset = 8;
        z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        z.count = 1;

        msg->fields.push_back(x);
        msg->fields.push_back(y);
        msg->fields.push_back(z);

        msg->point_step = 12; // Size of a point in bytes
        msg->row_step = msg->point_step * msg->width;

        // Allocate memory for the point cloud data
        msg->data.resize(msg->row_step * msg->height);

        // Populate the point cloud data
        uint8_t *ptr = &msg->data[0];
        for (size_t i = 0; i < msg->width; ++i)
        {
            float *point = reinterpret_cast<float *>(ptr);
            point[0] = points[i][0] / 1000.0f;  // X coordinate
            point[1] = points[i][1] / 1000.0f;  // Y coordinate
            point[2] = -points[i][2] / 1000.0f; // Z coordinate
            ptr += msg->point_step;
        }
        pcloud_publisher_->publish(*msg.get());
    }

    void VxsSensorPublisher::PublishStampedPointcloud(const int N, vxsdk::vxXYZT *eventsXYZT)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Set the header
        auto evcloud_header = std_msgs::msg::Header();
        evcloud_header.stamp = this->get_clock()->now();
        evcloud_header.frame_id = "sensor";
        msg->header = evcloud_header;
        // Unordered pointcloud. Height is 1 and Width is the size (N)
        msg->height = 1;
        msg->width = N;

        // Define the point cloud fields
        sensor_msgs::msg::PointField x, y, z, t;
        x.name = "x";
        x.offset = 0;
        x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x.count = 1;
        y.name = "y";
        y.offset = 4;
        y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        y.count = 1;
        z.name = "z";
        z.offset = 8;
        z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        z.count = 1;
        t.name = "t";
        t.offset = 12;
        t.datatype = sensor_msgs::msg::PointField::FLOAT64;
        t.count = 1;

        msg->fields.push_back(x);
        msg->fields.push_back(y);
        msg->fields.push_back(z);
        msg->fields.push_back(t);

        msg->point_step = sizeof(float) * 3 + sizeof(double); // Size of a point in bytes
        msg->row_step = msg->point_step * msg->width;

        // Allocate memory for the point cloud data
        msg->data.resize(msg->row_step * msg->height);

        // Populate the point cloud data
        uint8_t *ptr = &msg->data[0];
        for (size_t i = 0; i < msg->width; ++i)
        {
            float *point = reinterpret_cast<float *>(ptr);
            point[0] = eventsXYZT[i].x / 1000.0f;  // X coordinate
            point[1] = eventsXYZT[i].y / 1000.0f;  // Y coordinate
            point[2] = -eventsXYZT[i].z / 1000.0f; // Z coordinate
            *(double *)(ptr + t.offset) = *(double *)&(eventsXYZT[i].timestamp);
            ptr += msg->point_step;
        }
        evcloud_publisher_->publish(*msg.get());
    }

    void VxsSensorPublisher::PublishIMUSample(const imu::IMUSample &sample)
    {
        sensor_msgs::msg::Imu imu_msg;
        // 1. Work out time in seconds
        const double time_in_secs = ref_time_.seconds() + sample.stamp_seconds - sensor_ref_time_;
        // 2. Extract integer part of seconds
        const int32_t second_part = static_cast<int32_t>(time_in_secs);
        // 3. get the nanosecond part as in64
        const int64_t nanosecond_part = static_cast<int64_t>((time_in_secs - second_part) * 1e9);
        imu_msg.header.stamp = rclcpp::Time(second_part, nanosecond_part);
        imu_msg.header.frame_id = "IMU";

        //@TODO: Find covariance values from IMU manufacturer
        imu_msg.orientation_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        imu_msg.angular_velocity_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        imu_msg.linear_acceleration_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};

        // !TODO: Assign calibrated orientation if necessary
        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 1;

        imu_msg.angular_velocity.x = sample.omegaX;
        imu_msg.angular_velocity.y = sample.omegaY;
        imu_msg.angular_velocity.z = sample.omegaZ;

        imu_msg.linear_acceleration.x = sample.aX;
        imu_msg.linear_acceleration.y = sample.aY;
        imu_msg.linear_acceleration.z = sample.aZ;

        imu_publisher_->publish(imu_msg);
    }

} // end namespace vxs_ros
