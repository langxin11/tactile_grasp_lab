"""Contactile PTS (PapillArray) 触觉传感器 Python SDK 示例程序。

演示多线程与单线程两种数据读取模式，以及 Bias、采样率设置、
滑移检测、摩擦力估算等核心 API 调用。

依赖:
    ptsdk_cxx_pybind (原厂预编译 wheel, cp310 x86_64)
用法:
    python ptsdk_pybind_example.py
原厂文档:
    PTSPython_1.0_MAN_MAR25.pdf
"""

import time

import PTSDK_CXX_Pybind


def main():
    """主函数：依次运行多线程示例与单线程示例。"""

    # =========================================================================
    # 运行参数
    # =========================================================================

    # True: 多线程模式 (推荐，后台自动读取) / False: 单线程模式
    is_multithreaded = True

    # 是否将传感器数据写入 .csv 日志文件
    is_logging = True

    # 串口连接参数
    port = "/dev/ttyACM0"  # Linux 串口设备
    rate = 115200  # 波特率
    parity = 0  # 校验位 (0=无)
    byte_size = ""  # 数据位 (8 bits)
    is_flush = True  # 缓冲区积压时清空

    # =========================================================================
    # 初始化传感器与监听器
    # =========================================================================

    # 创建传感器对象，每个对应通迅集线器上一个物理传感器
    #   sen0: SEN0 端口
    #   sen1: SEN1 端口
    sen0 = PTSDK_CXX_Pybind.PTSDKSensor()
    sen1 = PTSDK_CXX_Pybind.PTSDKSensor()

    # 创建监听器并注册传感器
    listener = PTSDK_CXX_Pybind.PTSDKListener(is_logging)
    listener.addSensor(sen0)
    listener.addSensor(sen1)

    # =========================================================================
    # 多线程模式 (推荐)
    # =========================================================================
    # 后台线程自动收发数据帧，主线程通过 get 方法随时获取最新值。
    # 适合高频采集与实时控制场景。

    if is_multithreaded:
        # 连接串口并启动后台监听线程
        res = listener.connectAndStartListening(port, rate, parity, byte_size, is_flush)
        if res == 0:
            print("main(): 成功连接串口并启动后台监听")
        else:
            print("main(): 连接串口失败")
            exit(1)

        # 发送 Bias 请求 (零点校准)
        res = listener.sendBiasRequest()
        if res:
            print("main(): Bias 请求发送成功")
        else:
            print("main(): Bias 请求发送失败")
            exit(1)

        # 设置采样率为 500 Hz
        res = listener.setSamplingRate(PTSDK_CXX_Pybind.PTSDKConstants.SAMP_RATE_500)
        if res:
            print("main(): 采样率设置成功 (500 Hz)")
        else:
            print("main(: 采样率设置失败")
            exit(1)

        # ---- 读取整传感器数据 ----
        # 每秒读取一次 SEN0 全局合力 (XYZ)，共 10 次
        for _ in range(10):
            force = sen0.getGlobalForce()
            ndim = PTSDK_CXX_Pybind.PTSDKConstants.NDIM
            for dim_idx in range(ndim):
                print(f"S0: 全局 F{dim_idx} = {force[dim_idx]:.3f}")
            time.sleep(1)

        # 每秒读取一次 SEN1 全部支柱的三向位移
        for _ in range(10):
            s1disp = sen1.getAllDisplacements()
            for pillar_idx in range(sen1.getNPillar()):
                d0 = s1disp[0][pillar_idx]
                d1 = s1disp[1][pillar_idx]
                d2 = s1disp[2][pillar_idx]
                print(f"S1_P{pillar_idx}: D0={d0:.3f}, D1={d1:.3f}, D2={d2:.3f}")
            time.sleep(1)

        # ---- 读取单支柱数据 ----
        # 每秒读取一次 SEN0 第 3 号支柱的 XYZ 三分力
        pillar_idx = 3
        for _ in range(10):
            force = sen0.getPillarDisplacements(pillar_idx)
            ndim = PTSDK_CXX_Pybind.PTSDKConstants.NDIM
            for dim_idx in range(ndim):
                print(f"S0_P3: F{dim_idx}: {force[dim_idx]:.3f}")
            time.sleep(1)

        # ---- 滑移检测与摩擦力估算 ----
        # 启动滑移检测
        res = listener.startSlipDetection()
        if res:
            print("main(): 滑移检测启动成功")
        else:
            print("main(): 滑移检测启动失败")
            exit(1)

        # 获取全部支柱的滑移状态
        (
            is_slip_detection_active,
            is_ref_pillar_loaded,
            contact_states,
            slip_states,
        ) = sen0.getAllSlipStatus()
        for pillar_idx in range(sen0.getNPillar()):
            print(f"S0_P{pillar_idx}:")
            state = slip_states[pillar_idx]
            match state:
                case PTSDK_CXX_Pybind.PTSDKConstants.INELIGIBLE:
                    print("\t滑移检测启动时未接触")
                case PTSDK_CXX_Pybind.PTSDKConstants.CONTACT_AT_START:
                    print("\t自滑移检测启动起保持接触")
                case PTSDK_CXX_Pybind.PTSDKConstants.LOST_CONTACT:
                    print("\t已失去接触")
                case PTSDK_CXX_Pybind.PTSDKConstants.TLOADING:
                    print("\t正在承受切向载荷")
                case PTSDK_CXX_Pybind.PTSDKConstants.SLIPPED:
                    print("\t已发生滑移")

        # 获取当前摩擦力估算值
        friction = sen0.getFrictionEstimate()
        print(f"S0: 摩擦力估算 = {friction}")

        # 停止滑移检测
        res = listener.startSlipDetection()
        if res:
            print("main(): 滑移检测停止成功")
        else:
            print("main(): 滑移检测停止失败")
            exit(1)

        # 停止后台监听并断开串口
        listener.stopListeningAndDisconnect()

    # =========================================================================
    # 单线程模式
    # =========================================================================
    # 主线程显式调用 readNextSample() 阻塞等待每帧数据。
    # 适用于低频、事件驱动的读取场景。

    else:
        pillar_idx = 0

        # 连接串口 (不启动后台线程)
        res = listener.connect(port, rate, parity, byte_size)
        if res == 0:
            print("main(): 成功连接串口")
        else:
            print("main(): 连接串口失败")
            exit(1)

        # 循环读取 1000 帧，每 100 帧打印一次数据
        for i in range(1000):
            res = listener.readNextSample(True)
            if res:
                print("main(): 采样数据读取成功")
            else:
                print("main(): 采样数据读取失败")
                break

            # 读取 SEN0 第 0 号支柱的 XYZ 三分力
            s0p0_forces = sen0.getPillarForces(pillar_idx)
            if i % 100 == 0:
                ndim = PTSDK_CXX_Pybind.PTSDKConstants.NDIM
                for dim_idx in range(ndim):
                    print(f"S0_P0: F{dim_idx}: {s0p0_forces[dim_idx]:.3f}")

        # 断开串口
        listener.disconnect()


if __name__ == "__main__":
    main()
