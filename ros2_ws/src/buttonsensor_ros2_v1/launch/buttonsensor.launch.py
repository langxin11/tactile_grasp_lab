"""文件: buttonsensor.launch.py
简要: ButtonSensor ROS2 节点启动文件。声明所有可配置参数（Hub ID、传感器数量、
      串口配置等），并提供合理的默认值，支持通过命令行参数或 yaml 覆盖。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述：
    声明 launch 参数并创建 ButtonSensor ROS2 节点实例，
    将所有参数传递给节点，输出到 screen 方便调试。
    """
    # ---- 声明启动参数 ----
    hub_id_arg = DeclareLaunchArgument(
        "hub_id", default_value="0", description="集线器 Hub 的 ID（默认 0）"
    )
    n_sensors_arg = DeclareLaunchArgument(
        "n_sensors",
        default_value="10",
        description="使用的传感器数量（默认 10，应与实际连接数一致）",
    )
    com_port_arg = DeclareLaunchArgument(
        "com_port", default_value="/dev/ttyACM0", description="串口设备路径"
    )
    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate", default_value="9600", description="串口波特率"
    )
    parity_arg = DeclareLaunchArgument(
        "parity",
        default_value="0",
        description="奇偶校验位：0=NONE, 1=ODD, 2=EVEN",
    )
    byte_size_arg = DeclareLaunchArgument(
        "byte_size", default_value="8", description="数据位长度（默认 8 位）"
    )
    sampling_rate_arg = DeclareLaunchArgument(
        "sampling_rate", default_value="500", description="采样率（Hz），支持 100/250/500/1000"
    )
    log_dir_arg = DeclareLaunchArgument(
        "log_dir", default_value="", description="CSV log output directory; empty to disable"
    )

    # ---- 构建启动描述 ----
    return LaunchDescription(
        [
            hub_id_arg,
            n_sensors_arg,
            com_port_arg,
            baud_rate_arg,
            parity_arg,
            byte_size_arg,
            sampling_rate_arg,
            log_dir_arg,
            Node(
                package="buttonsensor_ros2_v1",
                executable="buttonsensor_ros2_node",
                name="buttonsensor_ros2_node",
                output="screen",  # 输出到终端，方便调试
                parameters=[  # 将 launch 参数映射到 ROS 节点参数
                    {"hub_id": LaunchConfiguration("hub_id")},
                    {"n_sensors": LaunchConfiguration("n_sensors")},
                    {"com_port": LaunchConfiguration("com_port")},
                    {"baud_rate": LaunchConfiguration("baud_rate")},
                    {"parity": LaunchConfiguration("parity")},
                    {"byte_size": LaunchConfiguration("byte_size")},
                    {"sampling_rate": LaunchConfiguration("sampling_rate")},
                    {"log_dir": LaunchConfiguration("log_dir")},
                ],
            ),
        ]
    )
