#include "lidar.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <bits/chrono.h>
#include <rclcpp/qos.hpp>

namespace lidar
{

using namespace sensor_msgs;

Lidar::Lidar() : Node("lidar")
{
        this->publisher =
                this->create_publisher<msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepAll()));

        // this->processor = this->create_wall_timer(std::chrono::milliseconds(50),
        // std::bind(&Lidar::process, this));
}

void Lidar::process()
{

        while (1) {
        }
}

} // namespace lidar
