#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <expected>
#include <optional>
#include <queue>
#include <string>
#include <vector>

namespace explorer
{

enum class Error
{
        INVALID_GRID,
        INVALID_POSITION,
        START_CELL_NOT_FREE,
};

const char* toString(Error error);

struct Coordinate
{
        int x;
        int y;
};

struct FrontierGroup
{
        std::vector<Coordinate> frontiers;
        Coordinate              representative;
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
        [[nodiscard]] std::expected<void, Error> run(const geometry_msgs::msg::Pose& currentPose);
        std::string describeStart(const geometry_msgs::msg::Pose& currentPose) const;
        const std::vector<FrontierGroup>&        getFrontierGroups() const;
        std::optional<Coordinate>
        getNearestRepresentative(const geometry_msgs::msg::Pose& currentPose) const;

      private:
        nav_msgs::msg::OccupancyGrid   grid;
        std::vector<std::vector<Mark>> markList;
        std::vector<Coordinate>        newFrontier;
        std::vector<FrontierGroup>     frontierGroup;
        std::queue<Coordinate>         m;
        std::queue<Coordinate>         f;

        bool                    isInside(const Coordinate& coord) const;
        bool                    isMarkedAs(int flag, const Coordinate& coord) const;
        void                    markAs(int flag, const Coordinate& coord);
        void                    markAs(int flag, const std::vector<Coordinate>& coords);
        bool                    isFrontier(const Coordinate& coord) const;
        bool                    hasOpenSpaceNeighbor(const Coordinate& coord) const;
        Coordinate              toCoordinate(const geometry_msgs::msg::Point& point) const;
        Coordinate              getRepresentative(const std::vector<Coordinate>& frontiers) const;
        std::vector<Coordinate> getAdjacent(const Coordinate& coord) const;
};

} // namespace explorer
