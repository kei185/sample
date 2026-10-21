#include "lidar.hpp"
#include "device.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

namespace lidar
{

Lidar::Lidar(xqueue::Queue<frame::LidarPoint>& parsedQueue)
    : Node("lidar"), parsedQueue(parsedQueue)
{
        this->publisher =
                this->create_publisher<sensor_msgs::msg::LaserScan>("scan",
                                                                    rclcpp::SensorDataQoS());
}

void Lidar::run(std::stop_token st)
{
        RCLCPP_INFO(this->get_logger(), "node lidar running");
        uint64_t published = 0u;
        while (!st.stop_requested()) {
                if (this->parsedQueue.empty())
                        continue;

                auto data = this->parsedQueue.pop();
                auto msg  = transform(data);
                this->publisher->publish(msg);
                ++published;
                RCLCPP_INFO_THROTTLE(this->get_logger(),
                                     *this->get_clock(),
                                     5000,
                                     "scan published: count=%llu, points=%zu, stamp=%d.%09u",
                                     static_cast<unsigned long long>(published),
                                     msg.ranges.size(),
                                     msg.header.stamp.sec,
                                     msg.header.stamp.nanosec);
        }
}

sensor_msgs::msg::LaserScan Lidar::transform(const frame::LidarPoint& lidar)
{
        using namespace device;

        auto msg = sensor_msgs::msg::LaserScan();

        msg.header.stamp.sec     = static_cast<int32_t>(lidar.timestamp / 1000);
        msg.header.stamp.nanosec = static_cast<uint32_t>((lidar.timestamp % 1000) * 1'000'000);
        msg.header.frame_id      = LIDAR_FRAME_ID;
        msg.range_min            = LIDAR_RANGE_MIN_M;
        msg.range_max            = LIDAR_RANGE_MAX_M;

        if (lidar.point.empty())
                return msg;

        // 終端角を基準にすれば、350度から10度の区間も連続した角度になる。
        // clockwise to anti clockwise
        msg.angle_min       = std::remainder(-lidar.angleEnd, 360.0) * RADIAN_PER_DEGREE;
        msg.angle_increment = lidar.angleDiff * RADIAN_PER_DEGREE;
        msg.angle_max       = msg.angle_min + msg.angle_increment * (lidar.point.size() - 1);

        msg.ranges.reserve(lidar.point.size());

        for (const double& point : lidar.point) {
                const float distance = point * LIDAR_METERS_PER_MM;
                msg.ranges.push_back(std::isfinite(distance) && distance >= msg.range_min &&
                                                     distance <= msg.range_max
                                             ? distance
                                             : std::numeric_limits<float>::quiet_NaN());
        }

        return msg;
}

} // namespace lidar
