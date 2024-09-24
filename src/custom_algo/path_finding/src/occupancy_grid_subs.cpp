#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include<sstream>


class MapSubscriber : public rclcpp::Node
{
public:
   
    MapSubscriber() : Node("map_subscriber")
    {
        // Subscribe to /map topic
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapSubscriber::map_callback, this, std::placeholders::_1));
    }

private:
    // Subscription object for /map topic
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;


    void printMapData(const std::vector<int8_t>& map_data, int width, int height) {
        std::stringstream ss;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int index = x + y * width;
                ss << static_cast<int>(map_data[index]) << " ";  // Cast int8_t to int to display correctly
            }
            ss << "\n";  // Newline after each row
        }
        RCLCPP_INFO(rclcpp::get_logger("map_logger"), "\n%s", ss.str().c_str());
    }


    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received a map!");

        int width = msg->info.width;
        int height = msg->info.height;
        double resolution = msg->info.resolution;

        std::vector<int8_t> map_data = msg->data; // This contains the occupancy grid data
        std::vector<std::vector<int>> grid(height, std::vector<int>(width));


        printMapData(map_data, width, height);
        // Convert 1D occupancy grid into 2D grid
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = x + y * width;
                grid[y][x] = map_data[index];
            }
        }


        // Start and goal points (hardcoded for this example)
        std::pair<int, int> start = {0, 0};
        std::pair<int, int> goal = {width - 1, height - 1};

        // Run Dijkstra's algorithm
        // std::vector<std::pair<int, int>> path = dijkstra(grid, start, goal);

        // if (!path.empty())
        // {
        //     RCLCPP_INFO(this->get_logger(), "Path found!");
        // }
        // else
        // {
        //     RCLCPP_WARN(this->get_logger(), "No valid path found!");
        // }
    }

    // Dijkstra's algorithm implementation
    // std::vector<std::pair<int, int>> dijkstra(const std::vector<std::vector<int>> &grid, 
    //                                           const std::pair<int, int> &start, 
    //                                           const std::pair<int, int> &goal)
    // {
    //     int height = grid.size();
    //     int width = grid[0].size();

    //     std::vector<std::vector<int>> dist(height, std::vector<int>(width, std::numeric_limits<int>::max()));
    //     std::vector<std::vector<std::pair<int, int>>> prev(height, std::vector<std::pair<int, int>>(width, {-1, -1}));

    //     // Lambda function to check if a cell is within the grid bounds
    //     auto is_valid = [&](int x, int y) {
    //         return x >= 0 && x < width && y >= 0 && y < height && grid[y][x] == 0;
    //     };

    //     std::priority_queue<std::pair<int, std::pair<int, int>>, std::vector<std::pair<int, std::pair<int, int>>>, std::greater<>> pq;
    //     pq.push({0, start});
    //     dist[start.second][start.first] = 0;

    //     std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}; // 4-way connectivity

    //     while (!pq.empty())
    //     {
    //         auto [current_dist, current] = pq.top();
    //         pq.pop();

    //         int x = current.first;
    //         int y = current.second;

    //         if (current == goal)
    //             break;

    //         for (auto &dir : directions)
    //         {
    //             int new_x = x + dir.first;
    //             int new_y = y + dir.second;

    //             if (is_valid(new_x, new_y))
    //             {
    //                 int alt = current_dist + 1;
    //                 if (alt < dist[new_y][new_x])
    //                 {
    //                     dist[new_y][new_x] = alt;
    //                     prev[new_y][new_x] = {x, y};
    //                     pq.push({alt, {new_x, new_y}});
    //                 }
    //             }
    //         }
    //     }

    //     // Reconstruct the path from goal to start
    //     std::vector<std::pair<int, int>> path;
    //     std::pair<int, int> current = goal;

    //     while (current != start)
    //     {
    //         path.push_back(current);
    //         current = prev[current.second][current.first];
    //         if (current == std::make_pair(-1, -1)) // No path found
    //             return {};
    //     }
    //     path.push_back(start);
    //     std::reverse(path.begin(), path.end());

    //     return path;
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSubscriber>());
    rclcpp::shutdown();
    return 0;
}