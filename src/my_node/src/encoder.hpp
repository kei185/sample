#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "core/frame.hpp"
#include "core/xqueue.hpp"
#include "worker/processor.hpp"

namespace encoder
{

struct Encoder : public processor::Processor, public rclcpp::Node
{

        xqueue::Queue<frame::Encoder>&                             parsedQueue;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;

        // queueの所有者は、ノードより長く生存すること。
        explicit Encoder(xqueue::Queue<frame::Encoder>& parsedQueue);

        void run(std::stop_token) override;
        void process();
};

} // namespace encoder
