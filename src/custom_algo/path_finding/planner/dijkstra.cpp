#include <path_finding/dijkstra.hpp>
#include <queue>
#include <limits>
#include <algorithm>


Dijkstra::Dijkstra(const std::vector<std::vector<int>>& grid) : grid_(grid), height_(grid.size()), width_(grid[0].size()){}

bool Dijkstra::isvalid(int x, int y){
    return x>=0 && x < width_ && y >= 0 && y < height_ && grid_[y][x] == 0;
}

std::vector<std::pair<int, int>> Dijkstra::find_path(const std::pair<int, int>& start, const std::pair<int, int>& goal){

    std::vector<std::vector<int>> dist(height_, std::vector<int>(width_, std::numeric_limits<int>::max()));
    std::vector<std::vector<std::pair<int, int>>> prev(height_, std::vector<std::pair<int, int>>(width_, {-1, -1}));

    std::priority_queue<std::pair<int, std::pair<int, int>>, std::vector<std::pair<int, std::pair<int, int>>>, std::greater<>> pq;

    pq.push({0, start});
    dist[start.second][start.first] = 0;

    std::vector<std::pair<int, int>> directtions = {{0,1}, {1, 0}, {0,-1}, {-1,0}};

    while (!pq.empty())
    {
        auto [current_dist, current] = pq.top();
        pq.pop();

        int x = current.first;
        int y = current.second;
        
        if(current == goal){
            break;
        }

        for (auto& dir: directtions){
            int new_x = x + dir.first;
            int new_y = y + dir.second;

            if(isvalid(new_x, new_y)){
                int alt = current_dist + 1;

                if(alt < dist[new_y][new_x]){
                    dist[new_y][new_x] = alt;
                    prev[new_y][new_x] = {x, y};
                    pq.push({alt, {new_x, new_y}});

                }
            }
        }
    }

    std::vector<std::pair<int, int>> path;
    std::pair<int, int> current = goal;

    while(current != start){
        path.push_back(current);
        current = prev[current.second][current.first];
        if(current == std::make_pair(-1, -1)){
            return {};
        }
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}