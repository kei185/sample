from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="my_node",
            executable="my_node",
            arguments=[LaunchConfiguration("serial_device")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            prefix='bash -c \'exec "$@" <<< ""\' --',
            # 標準入力へ改行を1個渡して、既存プロトコルのENTER待ちを通す。
            output="screen",
        ),
        # nameを指定すると、同じプロセス内の全ノードに名前のremapがかかるため指定しない。
    ])
