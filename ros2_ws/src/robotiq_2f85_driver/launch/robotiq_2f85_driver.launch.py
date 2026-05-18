"""启动 Robotiq 2F-85 lifecycle driver 节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _create_driver_node(context, *args, **kwargs) -> list[Node]:
    parameters = [LaunchConfiguration("config").perform(context)]
    config_override = LaunchConfiguration("config_override").perform(context)
    if config_override:
        parameters.append(config_override)

    parameters.append(
        {
            "serial_port": LaunchConfiguration("serial_port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "dry_run": LaunchConfiguration("dry_run"),
            "startup_activate": LaunchConfiguration("startup_activate"),
            "autostart": LaunchConfiguration("autostart"),
        }
    )

    driver_node = Node(
        package="robotiq_2f85_driver",
        executable="robotiq_driver_node",
        name="robotiq_2f85_driver",
        output="screen",
        parameters=parameters,
    )
    return [driver_node]


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("robotiq_2f85_driver"), "config", "robotiq_2f85_driver.yaml"]
    )

    declare_args = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("config_override", default_value=""),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("startup_activate", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
    ]
    return LaunchDescription(declare_args + [OpaqueFunction(function=_create_driver_node)])
