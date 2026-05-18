"""触觉抓取控制器节点启动文件。

加载基础 YAML 配置和可选覆盖配置，并启动 tactile_grasp_controller_node。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _create_controller_node(context, *args, **kwargs) -> list[Node]:
    parameters = [LaunchConfiguration("config").perform(context)]
    config_override = LaunchConfiguration("config_override").perform(context)
    if config_override:
        parameters.append(config_override)

    controller_node = Node(
        package="tactile_grasp_controller",
        executable="tactile_grasp_controller_node",
        name="tactile_grasp_controller",
        output="screen",
        parameters=parameters,
    )
    return [controller_node]


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
        description="Base YAML config file for the tactile grasp controller node.",
    )
    config_override_arg = DeclareLaunchArgument(
        "config_override",
        default_value="",
        description="Optional YAML override config file for the tactile grasp controller node.",
    )

    return LaunchDescription(
        [config_arg, config_override_arg, OpaqueFunction(function=_create_controller_node)]
    )
