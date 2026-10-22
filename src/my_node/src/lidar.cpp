#include "lidar.hpp"
#include "device.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace lidar
{

namespace
{
// 4000点/秒 ÷ 6周/秒を四捨五入し、ROSへ渡す一周の配列を667点に固定する。
constexpr size_t SCAN_POINTS =
        static_cast<size_t>(device::LIDAR_SAMPLE_RATE_HZ / device::LIDAR_SCAN_RATE_HZ + 0.5);
} // namespace

Lidar::Lidar(xqueue::Queue<frame::LidarPoint>& parsedQueue)
    : Node("lidar"), parsedQueue(parsedQueue)
{
        this->publisher =
                this->create_publisher<sensor_msgs::msg::LaserScan>("scan",
                                                                    rclcpp::SensorDataQoS());

        // 実機の測距・回転周波数から決めた点数で一周を等分し、SLAMへ渡す角度範囲を固定する。
        this->scan.header.frame_id = device::LIDAR_FRAME_ID;
        this->scan.range_min       = device::LIDAR_RANGE_MIN_M;
        this->scan.range_max       = device::LIDAR_RANGE_MAX_M;
        this->scan.angle_min       = 0.0f;
        this->scan.angle_increment = 360.0 / SCAN_POINTS * device::RADIAN_PER_DEGREE;
        this->scan.angle_max =
                this->scan.angle_min + this->scan.angle_increment * (SCAN_POINTS - 1u);
        this->scan.ranges.assign(SCAN_POINTS, std::numeric_limits<float>::quiet_NaN());
}

void Lidar::run(std::stop_token st)
{
        RCLCPP_INFO(this->get_logger(), "node lidar running");
        uint64_t published = 0u;
        while (!st.stop_requested()) {
                if (this->parsedQueue.empty())
                        continue;

                auto data = this->parsedQueue.pop();
                auto msg  = this->transform(data);
                if (!msg)
                        continue;
                const auto isAllFinite =
                        std::count_if(msg->ranges.begin(), msg->ranges.end(), [](float range) {
                                return std::isfinite(range);
                        });
                if (isAllFinite == 0) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(),
                                             *this->get_clock(),
                                             5000,
                                             "LiDAR rotation has no valid ranges");
                        continue;
                }
                this->publisher->publish(*msg);
                ++published;
                RCLCPP_INFO_THROTTLE(this->get_logger(),
                                     *this->get_clock(),
                                     5000,
                                     "scan published: count=%llu, points=%zu, valid=%zu, "
                                     "angle=(%.3f, %.3f), scan_time=%.3f s, stamp=%d.%09u",
                                     static_cast<unsigned long long>(published),
                                     msg->ranges.size(),
                                     static_cast<size_t>(isAllFinite),
                                     msg->angle_min,
                                     msg->angle_max,
                                     msg->scan_time,
                                     msg->header.stamp.sec,
                                     msg->header.stamp.nanosec);
        }
}

std::optional<sensor_msgs::msg::LaserScan> Lidar::transform(const frame::LidarPoint& lidar)
{
        using namespace device;

        if (lidar.point.empty())
                return std::nullopt;

        const double span = lidar.angleDiff * (lidar.point.size() - 1u);
        if (!std::isfinite(lidar.angleStart) || !std::isfinite(lidar.angleDiff) ||
            lidar.angleDiff < 0.0 || (lidar.point.size() > 1u && lidar.angleDiff == 0.0) ||
            !std::isfinite(span) || span >= 360.0) {
                RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        5000,
                        "Invalid LiDAR packet angles: start=%.3f, step=%.3f, points=%zu",
                        lidar.angleStart,
                        lidar.angleDiff,
                        lidar.point.size());
                return std::nullopt;
        }

        std::optional<sensor_msgs::msg::LaserScan> completed;
        for (size_t i = 0u; i < lidar.point.size(); ++i) {
                double angle = std::fmod(lidar.angleStart + lidar.angleDiff * i, 360.0);
                if (angle < 0.0)
                        angle += 360.0;

                // 0度をまたぐ点で区切る。最初の不完全な一周は配信しない。
                if (this->previousAngle >= 0.0 && this->previousAngle - angle > 180.0) {
                        if (this->collecting) {
                                this->scan.scan_time =
                                        lidar.timestamp >= this->scanStartTimestamp
                                                ? (lidar.timestamp - this->scanStartTimestamp) /
                                                          1000.0f
                                                : 0.0f;
                                completed = this->scan;
                        }

                        // 点ごとの時刻はないため、区切りを含むパケットの時刻を使う。
                        // time_incrementは0のままにし、欠落した角度の距離はNaNで残す。
                        this->scanStartTimestamp    = lidar.timestamp;
                        this->scan.header.stamp.sec = static_cast<int32_t>(lidar.timestamp / 1000u);
                        this->scan.header.stamp.nanosec =
                                static_cast<uint32_t>((lidar.timestamp % 1000u) * 1'000'000u);
                        std::fill(this->scan.ranges.begin(),
                                  this->scan.ranges.end(),
                                  std::numeric_limits<float>::quiet_NaN());
                        this->collecting = true;
                }
                this->previousAngle = angle;
                if (!this->collecting)
                        continue;

                const float distance = lidar.point[i] * LIDAR_METERS_PER_MM;
                if (!std::isfinite(distance) || distance < this->scan.range_min ||
                    distance > this->scan.range_max)
                        continue;

                // 時計回りの実測角を、反時計回りの固定配列へ入れる。
                const size_t index =
                        static_cast<size_t>(std::lround((360.0 - angle) * SCAN_POINTS / 360.0)) %
                        SCAN_POINTS;
                // 同じ角度のマスへ複数点が入った場合は、近い障害物を残す。
                if (!std::isfinite(this->scan.ranges[index]) || distance < this->scan.ranges[index])
                        this->scan.ranges[index] = distance;
        }
        return completed;
}

} // namespace lidar
