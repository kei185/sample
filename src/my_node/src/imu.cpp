#include "imu.hpp"
#include "core/frame.hpp"
#include "device.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <stop_token>
#include <cstdint>

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
        auto msg                 = sensor_msgs::msg::Imu();
        msg.header.stamp.sec     = static_cast<int32_t>(imu.timestamp / 1000);
        msg.header.stamp.nanosec = static_cast<uint32_t>((imu.timestamp % 1000) * 1'000'000);
        msg.header.frame_id      = device::IMU_FRAME_ID;

        msg.orientation_covariance[0] = -1.0;
        msg.angular_velocity.x        = imu.rot.x * device::IMU_ANGULAR_VELOCITY_RADPS_PER_LSB;
        msg.angular_velocity.y        = imu.rot.y * device::IMU_ANGULAR_VELOCITY_RADPS_PER_LSB;
        msg.angular_velocity.z        = imu.rot.z * device::IMU_ANGULAR_VELOCITY_RADPS_PER_LSB;
        msg.linear_acceleration.x     = imu.trans.x * device::IMU_ACCELERATION_MPS2_PER_LSB;
        msg.linear_acceleration.y     = imu.trans.y * device::IMU_ACCELERATION_MPS2_PER_LSB;
        msg.linear_acceleration.z     = imu.trans.z * device::IMU_ACCELERATION_MPS2_PER_LSB;

        return msg;
}

} // namespace imu
