#include "commander.hpp"
#include "core/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

namespace commander
{

Commander::Commander(xqueue::Queue<frame::MotorCommand>& directiveQueue)
    : Node("commnder"), directiveQueue(directiveQueue)
{
        this->cmdVel = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel",
                rclcpp::SensorDataQoS(),
                std::bind(&Commander::onMessage, this, std::placeholders::_1));
}

void Commander::onMessage(const geometry_msgs::msg::Twist::SharedPtr twist)
{
        auto cmd = transform(*twist);
        RCLCPP_INFO(this->get_logger(), "a");
        this->directiveQueue.push(std::move(cmd));
}

frame::MotorCommand Commander::transform(geometry_msgs::msg::Twist&)
{
        return frame::MotorCommand{};
}

} // namespace commander
