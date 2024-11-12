#include <chrono>
#include <functional>

#include <vxs_node.hpp>

namespace vxs_ros
{
    VvxsSensorPublisher::VxsSensorPublisher() : Node("vxs_sensor"), num_cams_(0)
    {
        // Declare & Get parameters
        this->declare_parameter("fps", rclcpp::PARAMETER_INTEGGER);
        this->declare_parameter("config_json", rclcpp::PARAMETER_STRING);
        this->declare_parameter("calib_json", rclcpp::PARAMETER_STRING);

        // Retrieve params
        rclcpp::Parameter fps_param = this->get_parameter("fps");
        rclcpp::Parameter config_json_param = this->get_parameter("config_json");
        rclcpp::Parameter calib_json_param = this->get_parameter("calib_json");

        fps_ = fps_param.as_int();
        config_json_ = config_json_param.as_string();
        calib_json_ = calib_json_param.as_string();

        // Initialize Sensor
        InitSensor();

        // Initialize publishing thread
        publishing_thread_ = std::make_shared<std::thread>(&PublisherLoop);

        publisher_ = this->create_publisher<std_msgs::msg::String>("vxs_data", 10);
        timer_ = this->create_wall_timer( //
            500ms,                        //
            std::bind(                    //
                &VxsPublisher::TimerCB,   //
                this)                     //
        );
    }

    void VxsSensorPublisher::TimerCB()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
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
        while (!shutdown_)
        {
            // @TODO: Publish depth images and sensor params here...
        }
    }

    void VxsSensorPublisher::FramePollinmgLoop()
    {
        while (!shutdown_request_)
        {
            // @TODO: Poll sensor and and hold back...

            std::unique_lock<std::mutex> sensor_lock(sensor_mutex_);
            // Block the thread if no data is available from the sensor
            cv_process_item.wait(
                sensor_lock,
                [this]()
                { return vxsdk::vxCheckForData() || shutdown_request_; });
            if (shutdown_request_)
            {
                rgbd_data_lock.unlock();
                continue;
            }

            // Get data from the sensor
            float *frameXYZ = vxsdk::vxGetFrameXYZ();

            // Unlock the queue now/ for subsequent processing
            sensor_lock.unlock();

            // @TODO: Maybe add data to queue or simply publish here...
        }
    }

} // end namespace vxs_ros