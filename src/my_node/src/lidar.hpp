#pragma once

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
        void                        run(std::stop_token) override;
        sensor_msgs::msg::LaserScan transform(const frame::LidarPoint&);
};

} // namespace lidar
