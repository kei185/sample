#include "imu.hpp"
#include "device.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace imu
{

Imu::Imu(xqueue::Queue<frame::Imu>& parsedQueue) : Node("imu"), parsedQueue(parsedQueue)
{
        this->publisher =
                this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
        this->timer = this->create_wall_timer(device::IMU_INTERVAL, [this] { this->process(); });
}

void Imu::run(std::stop_token st)
{
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(this->get_node_base_interface());

        // Workerのunique_ptrでも使える。停止要求を確認しながらtimerを動かす。
        while (rclcpp::ok() && !st.stop_requested())
                executor.spin_once(device::EXECUTOR_TIMEOUT);
}

void Imu::process()
{
        if (this->parsedQueue.empty())
                return;

        // TODO: parsedQueueから値を取得し、ROSの単位に変換してpublisherへ渡す。
        // 変換を実装するまではqueueから取り出さず、データを残しておく。
}

} // namespace imu
