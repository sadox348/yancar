from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("agv_task_controller"), "config", "controller.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="agv_task_controller",
            executable="task_controller_node",
            name="agv_task_controller",
            output="screen",
            parameters=[LaunchConfiguration("config")],
        ),
    ])
