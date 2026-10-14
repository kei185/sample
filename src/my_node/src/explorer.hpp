#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <queue>
#include <vector>

namespace explorer
{

struct Coordinate
{
        int x;
        int y;
};

struct Mark
{
        bool mapOpen;
        bool mapClose;
        bool frontierOpen;
        bool frontierClose;
};

class WFD
{
      public:
        // gridを所有する。std::moveで渡せばコピーせずに受け取れる。
        explicit WFD(nav_msgs::msg::OccupancyGrid grid);
        void run(const geometry_msgs::msg::Pose& currentPose);

      private:
        nav_msgs::msg::OccupancyGrid         grid;
        std::vector<std::vector<Mark>>       markList;
        std::vector<Coordinate>              newFrontier;
        std::vector<std::vector<Coordinate>> frontierGroup;
        std::queue<Coordinate>               m;
        std::queue<Coordinate>               f;

        bool                    isInside(const Coordinate& coord) const;
        bool                    isMarkedAs(int flag, const Coordinate& coord) const;
        void                    markAs(int flag, const Coordinate& coord);
        void                    markAs(int flag, const std::vector<Coordinate>& coords);
        bool                    isFrontier(const Coordinate& coord) const;
        bool                    hasOpenSpaceNeighbor(const Coordinate& coord) const;
        Coordinate              toCoordinate(const geometry_msgs::msg::Point& point) const;
        std::vector<Coordinate> getAdjacent(const Coordinate& coord) const;
};

} // namespace explorer
