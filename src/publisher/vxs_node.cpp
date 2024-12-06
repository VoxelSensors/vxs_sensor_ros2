#include <chrono>
#include <functional>

#include "publisher/vxs_node.hpp"

namespace vxs_ros
{
    VxsSensorPublisher::VxsSensorPublisher() :                                 //
                                               Node("vxs_sensor"),             //
                                               frame_polling_thread_(nullptr), //
                                               flag_shutdown_request_(false)
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("vxs_sensor_ros2");
        RCLCPP_INFO_STREAM(this->get_logger(), "Package share directory: " << package_share_directory);
        // Declare & Get parameters
        this->declare_parameter("publish_depth_image", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("publish_pcloud", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("fps", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("config_json", rclcpp::PARAMETER_STRING);
        this->declare_parameter("calib_json", rclcpp::PARAMETER_STRING);

        // Retrieve params

        // Publish depth image
        rclcpp::Parameter publish_depth_param;
        if (!this->get_parameter("publish_depth_image", publish_depth_param))
        {
            publish_depth_image_ = true;
        }
        else
        {
            publish_depth_image_ = publish_depth_param.as_bool();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing depth image " << (publish_depth_image_ ? "YES." : "NO."));

        // Publish point cloud
        rclcpp::Parameter publish_pcloud_param;
        if (!this->get_parameter("publish_pointcloud", publish_pcloud_param))
        {
            publish_pointcloud_ = true;
        }
        else
        {
            publish_pointcloud_ = publish_pcloud_param.as_bool();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing point cloud: " << (publish_pointcloud_ ? "YES." : "NO."));

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

        // By default publish depth image
        if (!publish_pointcloud_ && !publish_depth_image_)
        {
            publish_depth_image_ = true;
        }

        // Create publishers
        depth_publisher_ = publish_depth_image_ ? this->create_publisher<sensor_msgs::msg::Image>("depth/image", 10) : nullptr;
        pcloud_publisher_ = publish_pointcloud_ ? this->create_publisher<sensor_msgs::msg::PointCloud2>("pcloud/cloud", 10) : nullptr;

        cam_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("sensor/camera_info", 10);
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
            std::vector<cv::Vec3f> points;
            cv::Mat frame = UnpackSensorData(frameXYZ, points);
            // RCLCPP_INFO_STREAM(this->get_logger(), "DONE");
            //  Publish sensor data as a depth image
            if (publish_depth_image_)
            {
                PublishDepthImage(frame);
            }
            if (publish_pointcloud_)
            {
                PublishPointcloud(points);
            }
        }
        flag_in_polling_loop_ = false;
    }

    cv::Mat VxsSensorPublisher::UnpackSensorData(float *frameXYZ, std::vector<cv::Vec3f> &points)
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
            point[0] = points[i][0]; // X coordinate
            point[1] = points[i][1]; // Y coordinate
            point[2] = points[i][2]; // Z coordinate
            ptr += msg->point_step;
        }
        pcloud_publisher_->publish(*msg.get());
    }

} // end namespace vxs_ros