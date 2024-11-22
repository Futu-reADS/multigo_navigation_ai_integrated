#ifndef NAV_GOAL_H
#define NAV_GOAL_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <regex>
#include <vector>


namespace nav_goal
{
    class Nav_goal : public rclcpp::Node
    {
    public:
        Nav_goal();

    private:

        void arucoPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        int extractMarkerIds(const geometry_msgs::msg::PoseArray::SharedPtr& pose_array_msg);
        // TF buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub;

        // Publisher for the navigation goal
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;

        std::string map_frame = "map";
        std::string camera_frame = "camera_rgb_frame";
        std::string marker_topic_front = "aruco_detect/markers_front";
        int desired_aruco_marker_id;
        int found_aruco_marker_id;
        float aruco_distance_offset = -0.5;
        float aruco_left_right_offset = 0;
    };

} // namespace nav_goal

#endif // NAV_GOAL_H
