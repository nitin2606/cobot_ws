#include "path_finding/dijkstra.hpp"
#include "path_finding/waypoint_navigation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>

class MapFollower : public rclcpp::Node
{
public:
    MapFollower() : Node("map_follower")
    {
        // Subscribe to /map topic
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapFollower::map_callback, this, std::placeholders::_1));

        // Subscribe to /odom topic for robot position
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/the_bot/odom", 10, std::bind(&MapFollower::odom_callback, this, std::placeholders::_1));

        // Publisher for velocity commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/the_bot/cmd_vel", 10);
    }

    void initialize(){
        waypoint_nav_ = std::make_shared<WaypointNavigation>(this->shared_from_this());
        
    }

    

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    std::shared_ptr<WaypointNavigation> waypoint_nav_;  // Instance of WaypointNavigation
    bool path_available_ = false;

    void printGrid(const std::vector<std::vector<int>>& grid){
        for(const auto& row : grid){
            for(const auto& cell : row){
                std::cout << cell << " ";
            }
            std::cout << std::endl;
        }
    }

    // Callback for /map topic
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received a map!");

        int width = msg->info.width;
        int height = msg->info.height;

        std::vector<int8_t> map_data = msg->data;
        std::vector<std::vector<int>> grid(height, std::vector<int>(width));

        // Convert 1D occupancy grid into 2D grid
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = x + y * width;
                grid[y][x] = map_data[index];
                
            }
        }

        // Create a Dijkstra object and find the path

        printGrid(grid);
        std::cout << std::endl;


        // Dijkstra dijkstra(grid);
        // std::pair<int, int> start = {0, 0};
        // std::pair<int, int> goal = {width - 1, height - 1};

        // std::vector<std::pair<int, int>> path = dijkstra.find_path(start, goal);

      
        // if (!path.empty())
        // {
        //     RCLCPP_INFO(this->get_logger(), "Path found!");
        //     waypoint_nav_->set_path(path);  // Pass the path to the WaypointNavigation class
        //     path_available_ = true;
        // }
        // else
        // {
        //     RCLCPP_WARN(this->get_logger(), "No valid path found!");
        //     path_available_ = false;
        // }
    }

    // Callback for /odom topic
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!path_available_ || waypoint_nav_->is_path_completed())
        {
            return;
        }

        // Get the robot's current position and orientation
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // Convert quaternion to yaw (theta)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, theta;
        m.getRPY(roll, pitch, theta);

        // Get the next velocity command from WaypointNavigation
        geometry_msgs::msg::Twist cmd = waypoint_nav_->get_next_cmd(x, y, theta);

        // Publish the command
        cmd_vel_publisher_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapFollower>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
