#include "encoder.hpp"
#include "core/frame.hpp"
#include "device.hpp"
#include "utility/numecric.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <cstdint>

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
        RCLCPP_INFO(this->get_logger(), "node encoder running");
        uint64_t published = 0u;
        while (this->parsedQueue.waitData(st)) {
                auto data = this->parsedQueue.pop();
                auto msg  = transform(data);
                this->publisher->publish(msg);
                ++published;
                RCLCPP_INFO_THROTTLE(this->get_logger(),
                                     *this->get_clock(),
                                     5000,
                                     "odom published: count=%llu, stamp=%d.%09u, "
                                     "linear.x=%.3f m/s, angular.z=%.3f rad/s",
                                     static_cast<unsigned long long>(published),
                                     msg.header.stamp.sec,
                                     msg.header.stamp.nanosec,
                                     msg.twist.twist.linear.x,
                                     msg.twist.twist.angular.z);
        }
}

nav_msgs::msg::Odometry Encoder::transform(const frame::Encoder& encoder)
{
        using namespace device;
        // 入力は0.1秒間の回転量。車輪の円周と採取周波数を掛けてm/sへ直す。
        // 負の回転量は後退として、そのまま符号を残す。
        const double leftVelocity  = encoder.left * VELOCITY_PER_ROTATION;
        const double rightVelocity = encoder.right * VELOCITY_PER_ROTATION;

        auto msg = nav_msgs::msg::Odometry();

        msg.header.stamp.sec     = static_cast<int32_t>(encoder.timestamp / 1000);
        msg.header.stamp.nanosec = static_cast<uint32_t>((encoder.timestamp % 1000) * 1'000'000);
        msg.header.frame_id      = ODOM_FRAME_ID;
        msg.child_frame_id       = BASE_FRAME_ID;

        // 位置・姿勢の積算は行わず、EKFへ渡す車体座標系の速度だけを設定する。
        msg.pose.pose.orientation.w = 1.0;
        msg.twist.twist.linear.x    = translation(leftVelocity, rightVelocity);
        msg.twist.twist.angular.z   = rotation(leftVelocity, rightVelocity, WHEEL_SEPARATION);
        // EKFが融合する前進速度・横速度0・旋回角速度の分散を設定する。
        msg.twist.covariance[0]  = ENCODER_LINEAR_VELOCITY_VARIANCE;
        msg.twist.covariance[7]  = ENCODER_LATERAL_VELOCITY_VARIANCE;
        msg.twist.covariance[35] = ENCODER_ANGULAR_VELOCITY_VARIANCE;

        return msg;
}

} // namespace encoder
