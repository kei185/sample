import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("my_node")
    use_sim_time = LaunchConfiguration("use_sim_time")
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            ' "', LaunchConfiguration("model"), '"',
            " imu_height:=", LaunchConfiguration("imu_height"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=os.path.join(pkg_share, "urdf", "sample.urdf.xacro"),
        ),
        DeclareLaunchArgument("imu_height", default_value="0.021"),
        # 地面からIMUまでの高さ[m]。未計測なので初期値は車軸の高さ。
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # 現在のEncoderは関節角を配信しないため、表示する車輪の角度は0で固定する。
    ])
