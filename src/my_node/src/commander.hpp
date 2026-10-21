#pragma once

#include "core/frame.hpp"
#include "core/xqueue.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/node.hpp>
#include <stop_token>

namespace commander
{

struct Commander : public rclcpp::Node
{
        xqueue::Queue<frame::MotorCommand>&                        directiveQueue;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVel;

        explicit Commander(xqueue::Queue<frame::MotorCommand>&);
        void                run(std::stop_token);
        void                onMessage(const geometry_msgs::msg::Twist::SharedPtr);
        frame::MotorCommand transform(geometry_msgs::msg::Twist&);
};

} // namespace commander
