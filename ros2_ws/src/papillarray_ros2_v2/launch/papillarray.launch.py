# 文件: papillarray.launch.py
# 简要: PapillArray 触觉传感器 ROS2 启动文件。
#       声明所有可配置参数（串口、采样率、传感器数量等）并启动触觉发布节点，
#       该节点发布 /hub_<hub_id>/sensor_<sensor_id> -> SensorState 话题。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述：声明参数并启动 papillarray_ros2 节点。

    返回:
        LaunchDescription: 包含参数声明和节点启动配置的启动描述对象。
    """
    # hub_id: Hub 标识符
    # 该 ID 用于拼接触觉话题命名空间 /hub_<hub_id>/sensor_<sensor_id>，
    # 当系统中存在多个 Hub 时，通过不同的 hub_id 区分各个触觉采集单元。
    hub_id_arg = DeclareLaunchArgument("hub_id", default_value="0", description="ID of the hub")
    # n_sensors: 传感器数量
    # 当前 Contactile 硬件通常使用 1 或 2 个传感器。
    n_sensors_arg = DeclareLaunchArgument(
        "n_sensors",
        default_value="2",
        description="Number of sensors being used. Value can be 1 or 2",
    )
    # com_port: 串口设备路径
    # 这是 PTS 传感器侧串口（如 /dev/ttyACM0），不是 Robotiq 夹爪串口。
    com_port_arg = DeclareLaunchArgument(
        "com_port", default_value="/dev/ttyACM0", description="Name of COM port to connect with"
    )
    # baud_rate: 串口波特率
    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate", default_value="9600", description="Rate of serial connection"
    )
    # parity: 串口校验方式
    # 0=无校验(PARITY_NONE), 1=奇校验(PARITY_ODD), 2=偶校验(PARITY_EVEN)
    parity_arg = DeclareLaunchArgument(
        "parity",
        default_value="0",
        description="Parity: 0=PARITY_NONE, 1=PARITY_ODD, 2=PARITY_EVEN",
    )
    # byte_size: 字节位数，通常为 8
    byte_size_arg = DeclareLaunchArgument(
        "byte_size", default_value="8", description="Number of bits in byte. Default 8"
    )
    # is_flush: 缓冲区刷新标志
    # 当硬件输入缓冲区数据过多时，是否自动清空缓冲区。
    is_flush_arg = DeclareLaunchArgument(
        "is_flush",
        default_value="true",
        description="Flushing flag: flush hardware input buffer when it contains too many bytes",
    )
    # sampling_rate: 传感器采样频率 (Hz)
    # 可选值: 100, 250, 500, 1000。采样率由传感器硬件链路决定，
    # 上层控制器仍按自身的控制频率运行，不要求与采样率一致。
    sampling_rate_arg = DeclareLaunchArgument(
        "sampling_rate", default_value="500", description="Rate (Hz): 100, 250, 500, or 1000"
    )

    return LaunchDescription(
        [
            hub_id_arg,
            n_sensors_arg,
            com_port_arg,
            baud_rate_arg,
            parity_arg,
            byte_size_arg,
            is_flush_arg,
            sampling_rate_arg,
            Node(
                package="papillarray_ros2_v2",
                executable="papillarray_ros2_node",
                name="papillarray_ros2_node",
                output="screen",
                # 启动触觉发布节点，将传感器数据以 SensorState 消息格式发布到
                # /hub_<hub_id>/sensor_<sensor_id> 话题。
                parameters=[
                    {"hub_id": LaunchConfiguration("hub_id")},
                    {"n_sensors": LaunchConfiguration("n_sensors")},
                    {"com_port": LaunchConfiguration("com_port")},
                    {"baud_rate": LaunchConfiguration("baud_rate")},
                    {"parity": LaunchConfiguration("parity")},
                    {"byte_size": LaunchConfiguration("byte_size")},
                    {"is_flush": LaunchConfiguration("is_flush")},
                    {"sampling_rate": LaunchConfiguration("sampling_rate")},
                ],
            ),
        ]
    )
