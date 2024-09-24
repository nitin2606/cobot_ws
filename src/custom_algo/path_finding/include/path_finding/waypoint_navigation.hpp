#ifndef WAYPOINT_NAVIGATION_HPP

#define WAYPOINT_NAVIGATION_HPP

#include <vector>
#include <utility>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>


class WaypointNavigation{

    public:
        WaypointNavigation(rclcpp::Node::SharedPtr node);
        void set_path(const std::vector<std::pair<int, int>>& path);
        geometry_msgs::msg::Twist get_next_cmd(double current_x, double current_y, double theta);
        bool is_path_completed() const;
    
    private:
        std::vector<std::pair<int, int>> path_;
        rclcpp::Node::SharedPtr node_;
        bool path_completed_ = false;
};

#endif