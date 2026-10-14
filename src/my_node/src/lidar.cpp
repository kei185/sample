#include "lidar.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

void Lidar::run(std::stop_token st)
{
        while (st.stop_requested()) {
                if (!this->parsedQueue.empty())
                        continue;

                auto data = this->parsedQueue.pop();
                auto msg  = transform(data);
                this->publisher->publish(msg);
        }
}

sensor_msgs::msg::LaserScan transform(const frame::LidarPoint& lidar)
{
        (void)lidar;
        auto msg = sensor_msgs::msg::LaserScan();

        return msg;
}

} // namespace lidar
