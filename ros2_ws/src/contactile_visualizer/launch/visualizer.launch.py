#!/usr/bin/env python3
"""Contactile PTS 触觉传感器数据可视化 GUI 启动文件。

通过 ROS2 launch 系统启动 Contactile 触觉数据可视化界面，
支持通过启动参数配置话题名、刷新率、历史窗口等。
"""

# ===== ROS2 launch 导入 =====
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """构建可视化节点的启动描述。

    Returns:
        LaunchDescription: 包含声明参数和可视化节点的完整启动描述。
    """
    # ===== 声明启动参数 =====
    topic_arg = DeclareLaunchArgument(
        "topic",
        default_value="/hub_0/sensor_0",
        description="要可视化的 SensorState 话题名",
    )
    refresh_hz_arg = DeclareLaunchArgument(
        "refresh_hz",
        default_value="30",
        description="GUI 刷新频率，单位 Hz",
    )
    history_sec_arg = DeclareLaunchArgument(
        "history_sec",
        default_value="10",
        description="力和力矩曲线历史窗口长度，单位 s",
    )
    view_arg = DeclareLaunchArgument(
        "view",
        default_value="global",
        description="显示视图：displacement（位移）、force（力）或 global（全局）",
    )
    display_arg = DeclareLaunchArgument(
        "display",
        default_value="curve",
        description="显示模式：text（文本）或 curve（曲线）",
    )
    pillar_id_arg = DeclareLaunchArgument(
        "pillar_id",
        default_value="0",
        description="位移和力视图使用的柱编号",
    )

    # ===== 组装启动描述 =====
    return LaunchDescription(
        [
            topic_arg,
            refresh_hz_arg,
            history_sec_arg,
            view_arg,
            display_arg,
            pillar_id_arg,
            Node(
                package="contactile_visualizer",
                executable="tactile_gui",
                name="contactile_tactile_gui",
                output="screen",
                arguments=[
                    "--topic",
                    LaunchConfiguration("topic"),
                    "--refresh-hz",
                    LaunchConfiguration("refresh_hz"),
                    "--history-sec",
                    LaunchConfiguration("history_sec"),
                    "--view",
                    LaunchConfiguration("view"),
                    "--display",
                    LaunchConfiguration("display"),
                    "--pillar-id",
                    LaunchConfiguration("pillar_id"),
                ],
            ),
        ]
    )
