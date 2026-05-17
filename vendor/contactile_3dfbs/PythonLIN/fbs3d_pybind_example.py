"""Contactile 3DFBS 三维力按钮传感器 Python SDK 示例程序。

演示最多 10 个 3DFBS 传感器的连接、Bias 校准与连续数据读取。
3DFBS 每个传感器仅报告全局 XYZ 合力，
适合矩阵式压力分布感知场景。

依赖:
    FBS3D_CXX_Pybind (原厂预编译 wheel, cp310 x86_64)
用法:
    python fbs3d_pybind_example.py
原厂文档:
    3DFBSPython_B1.0_MAN_FEB25.pdf
"""

import time

import FBS3D_CXX_Pybind


def main():
    """主函数：连接最多 10 个 3DFBS 传感器并持续读取数据。"""

    # =========================================================================
    # 运行参数
    # =========================================================================

    # 是否将传感器数据写入 .csv 日志文件
    is_logging = True

    # 串口连接参数
    port = "/dev/ttyACM0"  # Linux 串口设备
    rate = 115200  # 波特率
    parity = 0  # 校验位 (0=无)
    byte_size = ""  # 数据位 (8 bits)
    # 日志写入频率 (Hz)
    log_file_rate = FBS3D_CXX_Pybind.PTSDKConstants.LOG_RATE_100

    # =========================================================================
    # 初始化传感器与监听器
    # =========================================================================

    # 3DFBS 通过 DEV001 通迅集线器连接，最多支持 10 个传感器
    #   端口 A: sen0 - sen4 (5 个)
    #   端口 B: sen5 - sen9 (5 个)
    # 每个传感器仅报告全局 XYZ 合力，无独立支柱数据

    sensors = []
    for i in range(10):
        sensor = FBS3D_CXX_Pybind.PTSDKSensor()
        sensors.append(sensor)

    sen0, sen1, sen2, sen3, sen4, sen5, sen6, sen7, sen8, sen9 = sensors

    # 创建监听器并注册全部传感器
    listener = FBS3D_CXX_Pybind.PTSDKListener(is_logging)
    for sensor in sensors:
        listener.addSensor(sensor)

    # =========================================================================
    # 连接与数据读取
    # =========================================================================

    # 连接串口
    res = listener.connect(port, rate, parity, byte_size)

    # 发送 Bias 请求 (零点校准)
    res = listener.sendBiasRequest()

    # 启动后台数据监听
    listener.startListening()

    # 每秒读取一次 SEN0 的全局 XYZ 合力，共 10 次
    for _ in range(10):
        force = sen0.getGlobalForce()
        ndim = FBS3D_CXX_Pybind.PTSDKConstants.NDIM
        for dim_idx in range(ndim):
            print(f"S0: 全局 F{dim_idx} = {force[dim_idx]:.3f}")
        time.sleep(1)

    # 停止后台监听并断开串口
    listener.stopListeningAndDisconnect()


if __name__ == "__main__":
    main()
