#ifndef OBSTACLE_AVOIDER_HPP_
#define OBSTACLE_AVOIDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace vxs_ros
{
    class ObstacleAvoider : public rclcpp::Node
    {
    public:
        ObstacleAvoider();
        virtual ~ObstacleAvoider();

    private:
        // Subscriber for the streaming events
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Helper to send the arrow to RViz
        void publish_arrow(float x, float y, float z, const std::string &frame_id);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    };
}
#endif // OBSTACLE_AVOIDER_HPP_