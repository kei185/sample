
// TODO cmd_velを流す
#include "core/frame.hpp"
#include "core/xqueue.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/node.hpp>

namespace commander
{

struct Commander : public rclcpp::Node
{
        xqueue::Queue<frame::MotorCommand>&                        directiveQueue;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVel;

        explicit Commander(xqueue::Queue<frame::MotorCommand>&);
        void                onMessage(const geometry_msgs::msg::Twist::SharedPtr);
        frame::MotorCommand transform(geometry_msgs::msg::Twist&);
};

} // namespace commander
