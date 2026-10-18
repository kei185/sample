import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("my_node")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "display.launch.py")
            ),
            launch_arguments={"use_rviz": "false", "use_sim_time": use_sim_time}.items(),
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                os.path.join(pkg_share, "config", "ekf.yaml"),
                {"use_sim_time": use_sim_time},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch", "online_async_launch.py",
            )),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
    ])
