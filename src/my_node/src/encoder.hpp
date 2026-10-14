#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "core/frame.hpp"
#include "core/xqueue.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "worker/processor.hpp"

namespace encoder
{

struct Encoder : public processor::Processor, public rclcpp::Node
{

        xqueue::Queue<frame::Encoder>&                        parsedQueue;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;

        // queueの所有者は、ノードより長く生存すること。
        explicit Encoder(xqueue::Queue<frame::Encoder>& parsedQueue);

        void                    run(std::stop_token) override;
        nav_msgs::msg::Odometry transform(const frame::Encoder&);
};

} // namespace encoder
