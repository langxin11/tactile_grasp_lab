"""硬件启动协调器启动文件。

在基础 bringup 之上额外启动硬件启动协调器节点，
负责触觉降噪校准和夹爪初始化流程。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    controller_config_default = PathJoinSubstitution(
        [
            FindPackageShare("tactile_grasp_controller"),
            "config",
            "tactile_grasp_controller.hardware.yaml",
        ]
    )
    coordinator_config_default = PathJoinSubstitution(
        [
            FindPackageShare("tactile_grasp_controller"),
            "config",
            "hardware_bringup_coordinator.yaml",
        ]
    )
    base_bringup_launch = PathJoinSubstitution(
        [
            FindPackageShare("tactile_grasp_controller"),
            "launch",
            "tactile_grasp_bringup.launch.py",
        ]
    )

    declare_args = [
        DeclareLaunchArgument("start_sensor", default_value="true"),
        DeclareLaunchArgument("start_gripper_execution", default_value="true"),
        DeclareLaunchArgument("start_controller", default_value="true"),
        DeclareLaunchArgument("start_coordinator", default_value="true"),
        DeclareLaunchArgument("auto_grasp", default_value="false"),
        DeclareLaunchArgument("hub_id", default_value="0"),
        DeclareLaunchArgument("n_sensors", default_value="2"),
        DeclareLaunchArgument("com_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud_rate", default_value="9600"),
        DeclareLaunchArgument("parity", default_value="0"),
        DeclareLaunchArgument("byte_size", default_value="8"),
        DeclareLaunchArgument("is_flush", default_value="true"),
        DeclareLaunchArgument("sampling_rate", default_value="500"),
        DeclareLaunchArgument("gripper_com_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("gripper_baudrate", default_value="115200"),
        DeclareLaunchArgument("use_fake_gripper", default_value="false"),
        DeclareLaunchArgument("controller_config", default_value=controller_config_default),
        DeclareLaunchArgument("coordinator_config", default_value=coordinator_config_default),
    ]

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_bringup_launch),
        launch_arguments={
            "start_sensor": LaunchConfiguration("start_sensor"),
            "start_gripper_execution": LaunchConfiguration("start_gripper_execution"),
            "start_controller": LaunchConfiguration("start_controller"),
            "hub_id": LaunchConfiguration("hub_id"),
            "n_sensors": LaunchConfiguration("n_sensors"),
            "com_port": LaunchConfiguration("com_port"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "parity": LaunchConfiguration("parity"),
            "byte_size": LaunchConfiguration("byte_size"),
            "is_flush": LaunchConfiguration("is_flush"),
            "sampling_rate": LaunchConfiguration("sampling_rate"),
            "gripper_com_port": LaunchConfiguration("gripper_com_port"),
            "gripper_baudrate": LaunchConfiguration("gripper_baudrate"),
            "use_fake_gripper": LaunchConfiguration("use_fake_gripper"),
            "controller_config": LaunchConfiguration("controller_config"),
        }.items(),
    )

    coordinator_node = Node(
        package="tactile_grasp_controller",
        executable="hardware_bringup_coordinator",
        name="hardware_bringup_coordinator",
        output="screen",
        parameters=[
            LaunchConfiguration("coordinator_config"),
            {"call_controller_start": LaunchConfiguration("auto_grasp")},
        ],
        condition=IfCondition(LaunchConfiguration("start_coordinator")),
    )

    return LaunchDescription(declare_args + [stack, coordinator_node])
