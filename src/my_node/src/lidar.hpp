
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/publisher.hpp>

namespace lidar
{

struct Lidar : public rclcpp::Node
{

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
        // rclcpp::TimerBase::ConstSharedPtr                         processor;

        Lidar();
        void process();
};
} // namespace lidar
