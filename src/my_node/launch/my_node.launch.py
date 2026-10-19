import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("my_node")
    use_sim_time = LaunchConfiguration("use_sim_time")
    application = Node(
        package="my_node",
        executable="my_node",
        arguments=[LaunchConfiguration("serial_device")],
        parameters=[{"use_sim_time": use_sim_time}],
        prefix='bash -c \'exec "$@" <<< ""\' --',
        # 標準入力へ改行を1個渡してからexecし、既存プロトコルのENTER待ちを通す。
        output="screen",
    )
    # nameを指定すると、同じプロセス内の全ノードに名前のremapがかかるため指定しない。

    return LaunchDescription([
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=application,
            on_exit=lambda _event, context: (
                [] if context.is_shutdown else [Shutdown(reason="my_node exited")]
            ),
        )),
        # 実機との入出力が終了したら、同じlaunchのノードも終了する。
        application,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "description.launch.py")
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch", "navigation_launch.py",
            )),
            launch_arguments={
                "params_file": LaunchConfiguration("nav2_params_file"),
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),
    ])
