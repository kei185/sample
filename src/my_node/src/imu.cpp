#include "imu.hpp"
#include "core/frame.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <stop_token>

namespace imu
{

Imu::Imu(xqueue::Queue<frame::Imu>& parsedQueue) : Node("imu"), parsedQueue(parsedQueue)
{
        this->publisher =
                this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
}

void Imu::run(std::stop_token st)
{
        while (this->parsedQueue.waitData(st)) {
                auto data = this->parsedQueue.pop();
                auto msg  = transform(data);
                this->publisher->publish(msg);
        }
}

sensor_msgs::msg::Imu Imu::transform(const frame::Imu& imu)
{
        (void)imu;
        auto msg = sensor_msgs::msg::Imu();

        return msg;
}

} // namespace imu
