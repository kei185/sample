#include "encoder.hpp"
#include "core/frame.hpp"
#include "device.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace encoder
{

Encoder::Encoder(xqueue::Queue<frame::Encoder>& parsedQueue)
    : Node("encoder"), parsedQueue(parsedQueue)
{
        this->publisher =
                this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
}

void Encoder::run(std::stop_token st)
{
        while (this->parsedQueue.waitData(st)) {
                auto data = this->parsedQueue.pop();
                auto msg  = transform(data);
                this->publisher->publish(msg);
        }
}

nav_msgs::msg::Odometry Encoder::transform(const frame::Encoder& encoder)
{
        (void)encoder;
        auto msg = nav_msgs::msg::Odometry();

        return msg;
}

} // namespace encoder
