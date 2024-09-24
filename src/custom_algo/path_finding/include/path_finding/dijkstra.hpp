#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <vector>
#include <utility>


class Dijkstra{

    public:
        Dijkstra(const std::vector<std::vector<int>>& grid);
        std::vector<std::pair<int, int>> find_path(const std::pair<int, int>& start, const std::pair<int, int>& goal);

    private:
        std::vector<std::vector<int>> grid_;
        int height_;
        int width_;

        bool isvalid(int x, int y);
};


#endif // DIJKSTRA_HPP
