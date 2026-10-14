#include "explorer.hpp"

#include <utility>

namespace explorer
{

#define MARKED     true
#define NOT_MARKED false

#define MAP_OPEN       0b0001
#define MAP_CLOSE      0b0010
#define FRONTIER_OPEN  0b0100
#define FRONTIER_CLOSE 0b1000

WFD::WFD(nav_msgs::msg::OccupancyGrid grid) : grid(std::move(grid)) {}

bool WFD::isMarkedAs(const int flag, const Coordinate& coord) const
{

        (void)flag;
        (void)coord;
        return false;
};

void WFD::markAs(const int flag, const Coordinate& coord)
{
        (void)flag;
        (void)coord;
}

void WFD::markAs(const int flag, const std::vector<Coordinate>& coords)
{
        (void)flag;
        (void)coords;
}

bool WFD::isFrontier(const Coordinate& coord) const
{
        (void)coord;
        return false;
}

bool WFD::hasOpenSpaceNeighbor(const Coordinate& coord) const
{
        (void)coord;
        return false;
}

Coordinate WFD::toCoordinate(const geometry_msgs::msg::Point& point) const
{
        (void)point;

        return {};
}

std::vector<Coordinate> WFD::getAdjacent(const Coordinate& coord) const
{
        (void)coord;
        return {};
};

void WFD::run(const geometry_msgs::msg::Pose& currentPose)
{
        markList = std::vector<std::vector<Mark>>{};
        markList.assign(grid.info.width,
                        std::vector<Mark>(grid.info.height,
                                          Mark{
                                                  .mapOpen       = NOT_MARKED,
                                                  .mapClose      = NOT_MARKED,
                                                  .frontierOpen  = NOT_MARKED,
                                                  .frontierClose = NOT_MARKED,
                                          }));

        m             = {};
        f             = {};
        newFrontier   = {};
        frontierGroup = {};

        auto currentCoordinate = toCoordinate(currentPose.position);
        m.push(currentCoordinate);
        markAs(MAP_OPEN, currentCoordinate);

        while (!m.empty()) {

                auto p = m.front();
                m.pop();
                if (isMarkedAs(MAP_CLOSE, p))
                        continue;

                if (isFrontier(p)) {
                        f           = {};
                        newFrontier = {};
                        f.push(p);
                        markAs(FRONTIER_OPEN, p);

                        while (!f.empty()) {
                                auto q = f.front();
                                f.pop();
                                if (isMarkedAs(MAP_CLOSE | FRONTIER_CLOSE, q))
                                        continue;

                                if (isFrontier(q)) {
                                        newFrontier.push_back(q);

                                        for (auto adj : getAdjacent(q))
                                                if (!isMarkedAs(MAP_CLOSE | FRONTIER_CLOSE |
                                                                        FRONTIER_OPEN,
                                                                q)) {
                                                        f.push(adj);
                                                        markAs(FRONTIER_OPEN, adj);
                                                }
                                }

                                markAs(FRONTIER_CLOSE, q);
                        }

                        frontierGroup.push_back(newFrontier);
                        markAs(MAP_CLOSE, newFrontier);
                }

                for (auto adj : getAdjacent(p))
                        if (!isMarkedAs(MAP_OPEN | MAP_CLOSE, p) && hasOpenSpaceNeighbor(p)) {
                                m.push(adj);
                                markAs(MAP_OPEN, p);
                        }

                markAs(MAP_CLOSE, p);
        }
}

} // namespace explorer
