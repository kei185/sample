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
}

void Lidar::run(std::stop_token st) { (void)st; }

void Lidar::process()
{
        if (this->parsedQueue.empty())
                return;

        // TODO: parsedQueueから点を取得し、LaserScanにまとめてpublisherへ渡す。
        // 変換を実装するまではqueueから取り出さず、データを残しておく。
}

} // namespace lidar
