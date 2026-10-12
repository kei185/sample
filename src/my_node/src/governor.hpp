#pragma once

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>

namespace governor
{

struct Governor : public rclcpp::Node
{
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client;

        Governor();

        void sendGoal();

        void feedbackCallback(
                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

        void resultCallback(const rclcpp_action::ClientGoalHandle<
                            nav2_msgs::action::NavigateToPose>::WrappedResult& result);
};

} // namespace governor
