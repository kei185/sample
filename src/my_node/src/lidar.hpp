#pragma once

#include <optional>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "core/frame.hpp"
#include "core/xqueue.hpp"
#include "worker/processor.hpp"

namespace lidar
{

struct Lidar : public processor::Processor, public rclcpp::Node
{

        xqueue::Queue<frame::LidarPoint>&                         parsedQueue;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;

        // queueの所有者は、ノードより長く生存すること。
        explicit Lidar(xqueue::Queue<frame::LidarPoint>& parsedQueue);
        void                                       run(std::stop_token) override;
        std::optional<sensor_msgs::msg::LaserScan> transform(const frame::LidarPoint&);

      private:
        sensor_msgs::msg::LaserScan scan;
        uint64_t                    scanStartTimestamp = 0u;
        double                      previousAngle      = -1.0;
        bool                        collecting         = false;
};

} // namespace lidar
