#include "governor.hpp"
#include "explorer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <action_msgs/msg/goal_status.hpp>
#include <chrono>
#include <cstdint>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/time.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <functional>
#include <future>
#include <stop_token>
#include <thread>
#include <utility>

namespace governor
{

using namespace std::placeholders;
using namespace std::chrono_literals;

Governor::Governor() : Node("governor")
{
        this->client =
                rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,
                                                                                "navigate_to_pose");

        this->map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "map",
                rclcpp::QoS(1),
                std::bind(&Governor::mapOnMessage, this, _1));
}

void Governor::run(std::stop_token st)
{
        RCLCPP_INFO(this->get_logger(), "node governor running");
        tf2_ros::Buffer            tfBuffer(this->get_clock());
        tf2_ros::TransformListener tfListener(tfBuffer);

        while (rclcpp::ok() && !st.stop_requested()) {
                rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(100ms);

                if (!this->grid) {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for map");
                        continue;
                }
                if (!this->client->action_server_is_ready()) {
                        RCLCPP_INFO_THROTTLE(get_logger(),
                                             *get_clock(),
                                             5000,
                                             "Waiting for navigate_to_pose action server");
                        continue;
                }

                const auto frameId =
                        this->grid->header.frame_id.empty() ? "map" : this->grid->header.frame_id;

                auto tf = getTf(tfBuffer, frameId);
                if (!tf)
                        continue;

                this->pose.pose.position.x  = tf.value().transform.translation.x;
                this->pose.pose.position.y  = tf.value().transform.translation.y;
                this->pose.pose.position.z  = tf.value().transform.translation.z;
                this->pose.pose.orientation = tf.value().transform.rotation;

                const auto info   = this->grid->info;
                auto       wfd    = explorer::WFD(std::move(this->grid.value()));
                RCLCPP_INFO_THROTTLE(get_logger(),
                                     *get_clock(),
                                     5000,
                                     "WFD start: frame=%s, grid=%u x %u, robot=(%.3f, %.3f)",
                                     frameId.c_str(),
                                     info.width,
                                     info.height,
                                     this->pose.pose.position.x,
                                     this->pose.pose.position.y);
                const auto result = wfd.run(this->pose.pose);

                this->grid.reset();
                // 計算済みの地図を無効化して、次に届く地図を待つ。
                if (!result) {
                        RCLCPP_WARN_THROTTLE(get_logger(),
                                             *get_clock(),
                                             5000,
                                             "WFD failed: %s; %s",
                                             explorer::toString(result.error()),
                                             wfd.describeStart(this->pose.pose).c_str());
                        continue;
                }

                const auto representative = wfd.getNearestRepresentative(this->pose.pose);
                if (!representative) {
                        RCLCPP_INFO_THROTTLE(get_logger(),
                                             *get_clock(),
                                             5000,
                                             "No frontier representative found");
                        continue;
                }
                if (st.stop_requested())
                        continue;

                const auto& origin = info.origin;
                const auto& q      = origin.orientation;

                const tf2::Transform gridOrigin(
                        tf2::Quaternion(q.x, q.y, q.z, q.w),
                        tf2::Vector3(origin.position.x, origin.position.y, origin.position.z));

                const auto target =
                        gridOrigin * tf2::Vector3((representative->x + 0.5) * info.resolution,
                                                  (representative->y + 0.5) * info.resolution,
                                                  0.0);
                // 代表セルの中心を、マップ座標のメートル値へ戻す。

                nav2_msgs::action::NavigateToPose::Goal goal;
                goal.pose.header.frame_id  = frameId;
                goal.pose.header.stamp     = this->now();
                goal.pose.pose.position.x  = target.x();
                goal.pose.pose.position.y  = target.y();
                goal.pose.pose.position.z  = target.z();
                goal.pose.pose.orientation = this->pose.pose.orientation;
                const auto status          = this->sendGoal(std::move(goal), st);
                RCLCPP_INFO_THROTTLE(get_logger(),
                                     *get_clock(),
                                     5000,
                                     "Navigation goal status: %s",
                                     toString(status));
        }
}

std::optional<geometry_msgs::msg::TransformStamped> Governor::getTf(tf2_ros::Buffer& tfBuffer,
                                                                    std::string      mapId)
{
        try {
                return tfBuffer.lookupTransform(mapId, "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& error) {
                RCLCPP_INFO_THROTTLE(get_logger(),
                                     *get_clock(),
                                     5000,
                                     "Waiting for TF %s <- base_link: %s",
                                     mapId.c_str(),
                                     error.what());
                return {};
        }
}

uint8_t Governor::sendGoal(nav2_msgs::action::NavigateToPose::Goal goal, std::stop_token st)
{
        if (st.stop_requested())
                return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;

        auto sendGoalOptions =
                rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        sendGoalOptions.feedback_callback = std::bind(&Governor::feedbackCallback, this, _1, _2);
        sendGoalOptions.result_callback   = std::bind(&Governor::resultCallback, this, _1);

        RCLCPP_INFO_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Sending navigation goal: frame=%s, position=(%.3f, %.3f)",
                             goal.pose.header.frame_id.c_str(),
                             goal.pose.pose.position.x,
                             goal.pose.pose.position.y);
        auto future = this->client->async_send_goal(goal, sendGoalOptions);

        while (rclcpp::ok() && !st.stop_requested()) {
                rclcpp::spin_some(this->get_node_base_interface());

                if (future.wait_for(0ms) == std::future_status::ready)
                        break;

                std::this_thread::sleep_for(100ms);
        }

        if (!rclcpp::ok() || st.stop_requested())
                return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;

        const auto handle = future.get();
        if (handle)
                return handle->get_status();
        return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
}

void Governor::feedbackCallback(

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
        RCLCPP_INFO_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "remaining distance: %.3f m",
                             feedback->distance_remaining);
}

void Governor::resultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult&
                result)
{
        switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Success!!!");
                        break;
                case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(get_logger(), "Goal was aborted");
                        return;
                case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(get_logger(), "Goal was canceled");
                        return;
                default:
                        RCLCPP_ERROR(get_logger(), "Unknown result code");
                        return;
        }
}

void Governor::mapOnMessage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
{
        this->grid = *grid;
        RCLCPP_INFO_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "map received: frame=%s, grid=%u x %u, resolution=%.3f m",
                             grid->header.frame_id.c_str(),
                             grid->info.width,
                             grid->info.height,
                             grid->info.resolution);
}

const char* toString(uint8_t status)
{
        using Status = action_msgs::msg::GoalStatus;

        switch (status) {
                case Status::STATUS_ACCEPTED:
                        return "ACCEPTED";
                case Status::STATUS_EXECUTING:
                        return "EXECUTING";
                case Status::STATUS_CANCELING:
                        return "CANCELING";
                case Status::STATUS_SUCCEEDED:
                        return "SUCCEEDED";
                case Status::STATUS_CANCELED:
                        return "CANCELED";
                case Status::STATUS_ABORTED:
                        return "ABORTED";
                case Status::STATUS_UNKNOWN:
                default:
                        return "UNKNOWN";
        }
}
} // namespace governor
