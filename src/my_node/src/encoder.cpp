#include "encoder.hpp"
#include "device.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace encoder
{

Encoder::Encoder(xqueue::Queue<frame::Encoder>& parsedQueue)
    : Node("encoder"), parsedQueue(parsedQueue)
{
        this->publisher =
                this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                                     rclcpp::SensorDataQoS());
}

void Encoder::run(std::stop_token st)
{
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(this->get_node_base_interface());

        // Workerのunique_ptrでも使える。停止要求を確認しながらtimerを動かす。
        while (rclcpp::ok() && !st.stop_requested())
                executor.spin_once(device::EXECUTOR_TIMEOUT);
}

void Encoder::process()
{
        if (this->parsedQueue.empty())
                return;
}

} // namespace encoder
