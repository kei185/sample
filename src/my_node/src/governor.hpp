#pragma once

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/buffer.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cstdint>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <optional>
#include <stop_token>

namespace governor
{

const char* toString(uint8_t status);

struct Governor : public rclcpp::Node
{
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr       map;

        std::optional<nav_msgs::msg::OccupancyGrid> grid;
        geometry_msgs::msg::PoseWithCovariance      pose;

        Governor();

        void run(std::stop_token st);
        // mapとactionのコールバックもこのループで処理する。
        uint8_t sendGoal(nav2_msgs::action::NavigateToPose::Goal, std::stop_token st = {});

        std::optional<geometry_msgs::msg::TransformStamped> getTf(tf2_ros::Buffer&, std::string);

        void feedbackCallback(
                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
        void resultCallback(const rclcpp_action::ClientGoalHandle<
                            nav2_msgs::action::NavigateToPose>::WrappedResult& result);

        void mapOnMessage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);
};

} // namespace governor
