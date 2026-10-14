#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "core/frame.hpp"
#include "core/xqueue.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "worker/processor.hpp"

namespace imu
{

struct Imu : public processor::Processor, public rclcpp::Node
{

        xqueue::Queue<frame::Imu>&                          parsedQueue;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;

        // queueの所有者は、ノードより長く生存すること。
        explicit Imu(xqueue::Queue<frame::Imu>& parsedQueue);
        void                  run(std::stop_token) override;
        sensor_msgs::msg::Imu transform(const frame::Imu&);
};

} // namespace imu
