#include "obstacle_avoider.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

namespace vxs_ros
{
    ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider_node")
    {
        // UPDATED: Correct topic name for streaming mode
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pcloud/events", 10,
            std::bind(&ObstacleAvoider::cloud_callback, this, std::placeholders::_1));

        // Publisher for the visualization arrow
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("avoidance_vector", 10);
        RCLCPP_INFO(this->get_logger(), "Obstacle Avoider Started. Listening on /pcloud/events");
    }

    ObstacleAvoider::~ObstacleAvoider() {}

    void ObstacleAvoider::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        float rep_x = 0.0, rep_y = 0.0, rep_z = 0.0;
        int valid_pts = 0;

        // 1. Robot position in the map frame
        const float robot_x = 0.06f;
        const float robot_y = 0.0f;
        const float robot_z = 0.0f;

        // 2. Iterate through the event stream
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // Basic Filter: Check for NaN
            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                continue;

            // Calculate point position relative to the ROBOT
            float rel_x = x - robot_x;
            float rel_y = y - robot_y;
            float rel_z = z - robot_z;

            // Distance calc (using relative coordinates)
            float dist_sq = rel_x * rel_x + rel_y * rel_y + rel_z * rel_z;

            // Filter: Ignore noise (<5cm) and far background (>1.5m)
            if (dist_sq < 0.0025)
                continue;

            if (dist_sq > 0.12)
                continue;

            float dist = std::sqrt(dist_sq);

            // Repulsion Logic: Vector pointing AWAY from the obstacle towards the robot
            float weight = 1.0f / dist_sq;
            rep_x += (-rel_x / dist) * weight;
            rep_y += (-rel_y / dist) * weight;
            rep_z += (-rel_z / dist) * weight;
            valid_pts++;
        }

        // 3. Vector Math: Scaled down for a 10cm grid
        float goal_x = 0.0;
        float goal_y = 0.0;
        float goal_z = -0.2; // Default: Go forward (reduced to 10cm)

        if (valid_pts > 20)
        {
            float rep_mag = std::sqrt(rep_x * rep_x + rep_y * rep_y + rep_z * rep_z);
            if (rep_mag > 0.001)
            {
                float gain = 0.1; // Tunable "Fear Factor" (reduced for smaller scale)
                goal_x += (rep_x / rep_mag) * gain;
                goal_y += (rep_y / rep_mag) * gain;
                goal_z += (rep_z / rep_mag) * gain;
            }
        }

        publish_arrow(goal_x, goal_y, goal_z, msg->header.frame_id);
    }

        void ObstacleAvoider::publish_arrow(float x, float y, float z, const std::string &frame_id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->now();
        marker.ns = "avoidance_logic";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;

        // 4. Arrow starts at the robot's actual position
        start.x = 0.06;
        start.y = 0.0;
        start.z = 0.0;

        // 5. End position is the start position + the calculated vector
        end.x = start.x + x;
        end.y = start.y + y;
        end.z = start.z + z;

        marker.points.push_back(start);
        marker.points.push_back(end);

        // 6. Arrow sizing dramatically reduced for 10cm grid visibility
        marker.scale.x = 0.005; // Shaft width (1 cm)
        marker.scale.y = 0.015; // Head width (2.5 cm)
        marker.scale.z = 0.015; // Head length (2.5 cm)

        // Color (Bright Green)
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);
    }
}