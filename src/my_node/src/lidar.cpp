#include "lidar.hpp"
#include "device.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace lidar
{

Lidar::Lidar(xqueue::Queue<frame::LidarPoint>& parsedQueue)
    : Node("lidar"), parsedQueue(parsedQueue)
{
        this->publisher =
                this->create_publisher<sensor_msgs::msg::LaserScan>("scan",
                                                                    rclcpp::SensorDataQoS());
        this->timer = this->create_wall_timer(device::LIDAR_INTERVAL, [this] { this->process(); });
}

void Lidar::run(std::stop_token st)
{
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(this->get_node_base_interface());

        // Workerのunique_ptrでも使える。停止要求を確認しながらtimerを動かす。
        while (rclcpp::ok() && !st.stop_requested())
                executor.spin_once(device::EXECUTOR_TIMEOUT);
}

void Lidar::process()
{
        if (this->parsedQueue.empty())
                return;

        // TODO: parsedQueueから点を取得し、LaserScanにまとめてpublisherへ渡す。
        // 変換を実装するまではqueueから取り出さず、データを残しておく。
}

} // namespace lidar
