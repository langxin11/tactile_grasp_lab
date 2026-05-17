"""触觉抓取控制器节点启动文件。

加载 YAML 配置并启动 tactile_grasp_controller_node。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("tactile_grasp_controller"),
            "config",
            "tactile_grasp_controller.yaml",
        ]
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML config file for the tactile grasp controller node.",
    )

    controller_node = Node(
        package="tactile_grasp_controller",
        executable="tactile_grasp_controller_node",
        name="tactile_grasp_controller",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, controller_node])
