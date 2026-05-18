"""统一 bringup 启动文件。

依次启动 PTS 传感器节点、Robotiq Python driver 执行层、触觉闭环控制器，
支持按层裁剪以方便单独调试某一层。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # 统一 bringup 入口：
    # 1. PTS 传感器节点
    # 2. Robotiq Python driver execution layer
    # 3. 触觉闭环 controller
    papillarray_launch = PathJoinSubstitution(
        [FindPackageShare("papillarray_ros2_v2"), "launch", "papillarray.launch.py"]
    )
    gripper_execution_launch = PathJoinSubstitution(
        [
            FindPackageShare("robotiq_2f85_driver"),
            "launch",
            "robotiq_2f85_driver.launch.py",
        ]
    )
    controller_launch = PathJoinSubstitution(
        [
            FindPackageShare("tactile_grasp_controller"),
            "launch",
            "tactile_grasp_controller.launch.py",
        ]
    )
    controller_config_default = PathJoinSubstitution(
        [
            FindPackageShare("tactile_grasp_controller"),
            "config",
            "tactile_grasp_controller.yaml",
        ]
    )

    declare_args = [
        DeclareLaunchArgument("start_sensor", default_value="true"),
        DeclareLaunchArgument("start_gripper_execution", default_value="true"),
        DeclareLaunchArgument("start_controller", default_value="true"),
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
        DeclareLaunchArgument("controller_config_override", default_value=""),
    ]

    # 允许按层裁剪启动，方便单独调试某一个包。
    sensor_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(papillarray_launch),
        condition=IfCondition(LaunchConfiguration("start_sensor")),
        launch_arguments={
            "hub_id": LaunchConfiguration("hub_id"),
            "n_sensors": LaunchConfiguration("n_sensors"),
            "com_port": LaunchConfiguration("com_port"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "parity": LaunchConfiguration("parity"),
            "byte_size": LaunchConfiguration("byte_size"),
            "is_flush": LaunchConfiguration("is_flush"),
            "sampling_rate": LaunchConfiguration("sampling_rate"),
        }.items(),
    )

    gripper_execution_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_execution_launch),
        condition=IfCondition(LaunchConfiguration("start_gripper_execution")),
        launch_arguments={
            "serial_port": LaunchConfiguration("gripper_com_port"),
            "baudrate": LaunchConfiguration("gripper_baudrate"),
            "dry_run": LaunchConfiguration("use_fake_gripper"),
            "startup_activate": "false",
            "autostart": "true",
        }.items(),
    )

    controller_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch),
        condition=IfCondition(LaunchConfiguration("start_controller")),
        launch_arguments={
            "config": LaunchConfiguration("controller_config"),
            "config_override": LaunchConfiguration("controller_config_override"),
        }.items(),
    )

    return LaunchDescription(
        declare_args + [sensor_stack, gripper_execution_stack, controller_stack]
    )
