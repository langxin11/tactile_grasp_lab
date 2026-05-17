"""Launch the Robotiq 2F-85 lifecycle driver node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("robotiq_2f85_driver"), "config", "robotiq_2f85_driver.yaml"]
    )

    declare_args = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("startup_activate", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
    ]

    driver_node = Node(
        package="robotiq_2f85_driver",
        executable="robotiq_driver_node",
        name="robotiq_2f85_driver",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "dry_run": LaunchConfiguration("dry_run"),
                "startup_activate": LaunchConfiguration("startup_activate"),
                "autostart": LaunchConfiguration("autostart"),
            },
        ],
    )

    return LaunchDescription(declare_args + [driver_node])
