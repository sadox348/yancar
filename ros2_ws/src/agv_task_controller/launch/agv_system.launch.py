from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_launch = PathJoinSubstitution([
        FindPackageShare("stm32_base_bridge"), "launch", "stm32_base_bridge.launch.py"
    ])
    controller_launch = PathJoinSubstitution([
        FindPackageShare("agv_task_controller"), "launch", "task_controller.launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={"port": LaunchConfiguration("port")}.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch)),
    ])
