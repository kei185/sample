#include "governor.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <functional>

namespace governor
{

using namespace std::placeholders;

Governor::Governor() : Node("governor")
{
        this->client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                this,
                "governor::navigate_to_pose");
}

void Governor::sendGoal()
{
        while (!this->client->wait_for_action_server()) {
                RCLCPP_INFO(get_logger(), "Waiting for action server...");
        }

        // アクション　Goalの作成
        auto goal_msg                 = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.stamp    = this->now();
        goal_msg.pose.header.frame_id = "map";

        goal_msg.pose.pose.position.x    = 2;
        goal_msg.pose.pose.position.y    = 0;
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        goal_msg.pose.pose.orientation.z = 0.0;

        // 進捗状況を表示するFeedbackコールバックを設定
        auto sendGoalOptions =
                rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        sendGoalOptions.feedback_callback = std::bind(&Governor::feedbackCallback, this, _1, _2);

        sendGoalOptions.result_callback = std::bind(&Governor::resultCallback, this, _1);

        // Goal をサーバーに送信
        this->client->async_send_goal(goal_msg, sendGoalOptions);
}

void Governor::feedbackCallback(

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
        RCLCPP_INFO(get_logger(), "remaining distance: %f", feedback->distance_remaining);
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

} // namespace governor
