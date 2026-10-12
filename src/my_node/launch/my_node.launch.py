import os

from launch import LaunchDescription

# from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # for tf static
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])},
        ],
    )

    # joint_state_publisher = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     parameters=[{"robot_description": Command(["xacro ", default_model_path])}],
    # )
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    #     condition=IfCondition(LaunchConfiguration("gui")),
    # )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="async_slam_toolbox_node",
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            # joint_state_publisher,
            # joint_state_publisher_gui_node,
            robot_state_publisher,
            robot_localization,
            slam_toolbox,
        ]
    )
