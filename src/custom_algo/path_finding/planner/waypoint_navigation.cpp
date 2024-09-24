#include "path_finding/waypoint_navigation.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>

WaypointNavigation::WaypointNavigation(rclcpp::Node::SharedPtr node): node_(node) {}

void WaypointNavigation::set_path(const std::vector<std::pair<int, int>>& path){

    path_ = path;
    path_completed_ = path_.empty();

}


bool WaypointNavigation::is_path_completed() const{

    return path_completed_;
}


geometry_msgs::msg::Twist WaypointNavigation::get_next_cmd(double current_x, double current_y, double theta){
   
   
    geometry_msgs::msg::Twist cmd;

    if(path_.empty()){
        path_completed_ = true;
        return cmd;
    }


    std::pair<int, int> next_waypoint = path_.front();

    double dx = next_waypoint.first - current_x;
    double dy = next_waypoint.second - current_y;
    double distance = std::sqrt(dx*dx + dy*dy);
    double target_angle = std::atan2(dy, dx);

    double angle_diff = target_angle - theta;

    if(distance < 0.1){
        RCLCPP_INFO(node_->get_logger(), "Waypoint reached: (%d, %d)", next_waypoint.first, next_waypoint.second);
        path_.erase(path_.begin());
        if(path_.empty()){

            path_completed_ = true;
            RCLCPP_INFO(node_->get_logger(), "All waypoints completed.");
        }
    }

    cmd.linear.x = std::min(distance, 0.5);
    cmd.angular.z = angle_diff;

    return cmd;
}