#include "explorer.hpp"

#include <tf2/LinearMath/Transform.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
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

const std::vector<FrontierGroup>& WFD::getFrontierGroups() const { return frontierGroup; }

Coordinate WFD::getRepresentative(const std::vector<Coordinate>& frontiers) const
{
        double centroidX = 0.0;
        double centroidY = 0.0;
        for (const auto& frontier : frontiers) {
                centroidX += frontier.x;
                centroidY += frontier.y;
        }
        centroidX /= frontiers.size();
        centroidY /= frontiers.size();

        Coordinate representative         = frontiers.front();
        double     nearestDistanceSquared = std::numeric_limits<double>::infinity();
        for (const auto& frontier : frontiers) {
                const double dx              = frontier.x - centroidX;
                const double dy              = frontier.y - centroidY;
                const double distanceSquared = dx * dx + dy * dy;
                if (distanceSquared < nearestDistanceSquared) {
                        nearestDistanceSquared = distanceSquared;
                        representative         = frontier;
                }
        }
        return representative;
}

std::optional<Coordinate>
WFD::getNearestRepresentative(const geometry_msgs::msg::Pose& currentPose) const
{
        if (frontierGroup.empty())
                return std::nullopt;

        const auto current = toCoordinate(currentPose.position);
        if (!isInside(current))
                return std::nullopt;

        std::optional<Coordinate> nearest;
        auto                      nearestDistanceSquared = std::numeric_limits<int64_t>::max();
        for (const auto& group : frontierGroup) {
                const auto dx = static_cast<int64_t>(group.representative.x) - current.x;
                const auto dy = static_cast<int64_t>(group.representative.y) - current.y;
                const auto distanceSquared = dx * dx + dy * dy;
                if (distanceSquared < nearestDistanceSquared) {
                        nearestDistanceSquared = distanceSquared;
                        nearest                = group.representative;
                }
        }
        return nearest;
}

// 失敗時だけ呼び、開始セルと周囲5×5マスを調べる。地図全体は走査しない。
std::string WFD::describeStart(const geometry_msgs::msg::Pose& currentPose) const
{
        const auto& position = currentPose.position;
        const auto& info     = grid.info;
        const auto& origin   = info.origin;
        std::ostringstream detail;
        detail << "frame=" << grid.header.frame_id
               << ", robot=(" << position.x << ", " << position.y << ", " << position.z << ")"
               << ", grid=" << info.width << "x" << info.height
               << ", resolution=" << info.resolution
               << ", origin=(" << origin.position.x << ", " << origin.position.y << ", "
               << origin.position.z << "), origin_q=(" << origin.orientation.x << ", "
               << origin.orientation.y << ", " << origin.orientation.z << ", "
               << origin.orientation.w << "), data_size=" << grid.data.size();

        // INVALID_GRIDの場合も、無効な座標変換や配列アクセスをせずに診断情報を返す。
        if (info.width == 0u || info.height == 0u ||
            info.width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
            info.height > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
            !std::isfinite(info.resolution) || info.resolution <= 0.0f ||
            grid.data.size() != static_cast<size_t>(info.width) * info.height)
                return detail.str() + ", cell=unavailable (invalid grid)";

        const auto current = toCoordinate(position);
        detail << ", cell=(" << current.x << ", " << current.y << ")";
        if (!isInside(current))
                return detail.str() + ", value=out_of_bounds";

        const auto index = static_cast<size_t>(current.y) * info.width + current.x;
        const int  value = grid.data[index];
        detail << ", value=" << value << " ("
               << (value == -1 ? "unknown" : value == 0 ? "free" : "occupied") << ")";

        // 上の行ほどgridのyが大きく、各行はxの小さい順。中央が開始セル、outは地図の外。
        detail << ", neighbors_5x5[y+2..y-2,x-2..x+2]=[";
        for (int dy = 2; dy >= -2; --dy) {
                if (dy != 2)
                        detail << "; ";
                for (int dx = -2; dx <= 2; ++dx) {
                        if (dx != -2)
                                detail << ",";
                        const int64_t x = static_cast<int64_t>(current.x) + dx;
                        const int64_t y = static_cast<int64_t>(current.y) + dy;
                        if (x < 0 || y < 0 || x >= info.width || y >= info.height)
                                detail << "out";
                        else
                                detail << static_cast<int>(
                                        grid.data[static_cast<size_t>(y) * info.width + x]);
                }
        }
        detail << "]";
        return detail.str();
}

bool WFD::isInside(const Coordinate& coord) const
{
        return coord.x >= 0 && coord.y >= 0 && static_cast<unsigned>(coord.x) < grid.info.width &&
               static_cast<unsigned>(coord.y) < grid.info.height;
}

bool WFD::isMarkedAs(const int flag, const Coordinate& coord) const
{
        const Mark& mark = markList[coord.x][coord.y];
        // 複数のflagが指定されたら、どれか1つでも付いているかを調べる。
        return ((flag & MAP_OPEN) && mark.mapOpen) || ((flag & MAP_CLOSE) && mark.mapClose) ||
               ((flag & FRONTIER_OPEN) && mark.frontierOpen) ||
               ((flag & FRONTIER_CLOSE) && mark.frontierClose);
}

void WFD::markAs(const int flag, const Coordinate& coord)
{
        Mark& mark = markList[coord.x][coord.y];
        if (flag & MAP_OPEN)
                mark.mapOpen = MARKED;
        if (flag & MAP_CLOSE)
                mark.mapClose = MARKED;
        if (flag & FRONTIER_OPEN)
                mark.frontierOpen = MARKED;
        if (flag & FRONTIER_CLOSE)
                mark.frontierClose = MARKED;
}

void WFD::markAs(const int flag, const std::vector<Coordinate>& coords)
{
        for (const auto& coord : coords)
                markAs(flag, coord);
}

bool WFD::isFrontier(const Coordinate& coord) const
{
        // 未知セルで、8近傍に空きセルがあるものをfrontierとする。
        const auto index = static_cast<size_t>(coord.y) * grid.info.width + coord.x;
        return grid.data[index] == -1 && hasOpenSpaceNeighbor(coord);
}

bool WFD::hasOpenSpaceNeighbor(const Coordinate& coord) const
{
        for (const auto& neighbor : getAdjacent(coord)) {
                // OccupancyGridのdataは、x方向を先に並べる1次元配列。
                const auto index = static_cast<size_t>(neighbor.y) * grid.info.width + neighbor.x;
                if (grid.data[index] == 0)
                        return true;
        }
        return false;
}

Coordinate WFD::toCoordinate(const geometry_msgs::msg::Point& point) const
{
        const auto& origin = grid.info.origin;
        const auto& q      = origin.orientation;

        const tf2::Transform gridOrigin(
                tf2::Quaternion(q.x, q.y, q.z, q.w),
                tf2::Vector3(origin.position.x, origin.position.y, origin.position.z));

        // 地図原点の姿勢を逆変換して、位置をgridの座標系へ戻す。
        const tf2::Vector3 local = gridOrigin.inverse() * tf2::Vector3(point.x, point.y, point.z);

        const double x = std::floor(local.x() / grid.info.resolution);
        const double y = std::floor(local.y() / grid.info.resolution);

        if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0 || x >= grid.info.width ||
            y >= grid.info.height)
                return {-1, -1};

        return {static_cast<int>(x), static_cast<int>(y)};
}

std::vector<Coordinate> WFD::getAdjacent(const Coordinate& coord) const
{
        std::vector<Coordinate> adjacent;
        if (!isInside(coord))
                return adjacent;

        adjacent.reserve(8u);
        for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0)
                                continue;
                        Coordinate neighbor{coord.x + dx, coord.y + dy};
                        if (isInside(neighbor))
                                adjacent.push_back(neighbor);
                }
        return adjacent;
}

std::expected<void, Error> WFD::run(const geometry_msgs::msg::Pose& currentPose)
{
        markList      = {};
        m             = {};
        f             = {};
        newFrontier   = {};
        frontierGroup = {};

        // gridの形と開始位置をここで確認し、補助関数には有効な座標を渡す。
        if (grid.info.width == 0u || grid.info.height == 0u ||
            grid.info.width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
            grid.info.height > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
            !std::isfinite(grid.info.resolution) || grid.info.resolution <= 0.0f ||
            grid.data.size() != static_cast<size_t>(grid.info.width) * grid.info.height)
                return std::unexpected(Error::INVALID_GRID);

        auto currentCoordinate = toCoordinate(currentPose.position);
        if (!isInside(currentCoordinate))
                return std::unexpected(Error::INVALID_POSITION);
        const size_t startIndex =
                static_cast<size_t>(currentCoordinate.y) * grid.info.width + currentCoordinate.x;
        if (grid.data[startIndex] != 0)
                return std::unexpected(Error::START_CELL_NOT_FREE);

        markList.assign(grid.info.width,
                        std::vector<Mark>(grid.info.height,
                                          Mark{
                                                  .mapOpen       = NOT_MARKED,
                                                  .mapClose      = NOT_MARKED,
                                                  .frontierOpen  = NOT_MARKED,
                                                  .frontierClose = NOT_MARKED,
                                          }));

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
                                                                adj)) {
                                                        f.push(adj);
                                                        markAs(FRONTIER_OPEN, adj);
                                                }
                                }

                                markAs(FRONTIER_CLOSE, q);
                        }

                        if (!newFrontier.empty())
                                frontierGroup.push_back({
                                        .frontiers      = newFrontier,
                                        .representative = getRepresentative(newFrontier),
                                });
                        markAs(MAP_CLOSE, newFrontier);
                }

                for (auto adj : getAdjacent(p)) {
                        const auto index = static_cast<size_t>(adj.y) * grid.info.width + adj.x;
                        // 障害物を経由して、反対側の空き領域へ探索を広げない。
                        if (grid.data[index] > 0)
                                continue;
                        if (!isMarkedAs(MAP_OPEN | MAP_CLOSE, adj) && hasOpenSpaceNeighbor(adj)) {
                                m.push(adj);
                                markAs(MAP_OPEN, adj);
                        }
                }

                markAs(MAP_CLOSE, p);
        }
        return {};
}

const char* toString(Error error)
{
        switch (error) {
                case Error::INVALID_GRID:
                        return "INVALID_GRID";
                case Error::INVALID_POSITION:
                        return "INVALID_POSITION";
                case Error::START_CELL_NOT_FREE:
                        return "START_CELL_NOT_FREE";
                default:
                        return "UNKNOWN";
        }
}

} // namespace explorer
