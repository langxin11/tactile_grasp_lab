// 文件: example.cpp
// 简要: Contactile PTS (PapillArray) 触觉传感器 C++ SDK 示例程序。
//       演示单线程与多线程两种数据读取模式，以及 Bias、采样率设置、
//       滑移检测等核心 API 调用。
// 适用平台: Linux (x86_64)
// 用法:
//   1. 将 vendor/contactile_pts/C++LIN/Include 添加到头文件搜索路径
//   2. 链接 vendor/contactile_pts/C++LIN/Library/libPTSDK.a
//   3. 连接 -lpthread
//   编译示例 (g++):
//     g++ -std=c++14 -I../../Include -L../../Library example.cpp -lPTSDK -lpthread -o example
// 原厂文档: PTSDK_2.0_MAN_DEC21.pdf, PTSCOM_2.0_SPEC_SEP22.pdf

#include <stdio.h>
#include <ctime>
#include <chrono>
#include <thread>

#include "PTSDKConstants.h"
#include "PTSDKListener.h"
#include "PTSDKSensor.h"

int main() {
  // =========================================================================
  // 运行参数配置
  // =========================================================================

  // 选择示例模式
  int example;
  // example = 1;  // 单线程模式：主线程阻塞式轮询读取数据
  example = 2;    // 多线程模式：后台线程处理数据收发，主线程按需获取

  // 是否将传感器数据写入 .csv 日志文件
  bool isLogging;
  isLogging = true;
  // isLogging = false;

  // 采样率 (Hz): SAMP_RATE_100=100Hz, SAMP_RATE_500=500Hz
  int sampleRate;
  sampleRate = SAMP_RATE_500;
  // sampleRate = SAMP_RATE_100;

  // 当串口缓冲区积压过多字节时是否强制清空
  bool isFlush;
  isFlush = true;
  // isFlush = false;

  // =========================================================================
  // 初始化传感器与监听器
  // =========================================================================

  // 创建传感器对象，每个对象对应插入通迅集线器 (Communication Hub)
  // 某个接口的物理传感器
  //   - sen0: SEN0 端口传感器
  //   - sen1: SEN1 端口传感器
  PTSDKSensor sen0 = PTSDKSensor();
  PTSDKSensor sen1 = PTSDKSensor();

  // 创建监听器，负责串口连接与数据帧管理
  PTSDKListener listener = PTSDKListener(isLogging);

  // 将传感器注册到监听器，此后监听器会自动解析对应传感器的数据帧
  listener.addSensor(&sen0);
  listener.addSensor(&sen1);

  // =========================================================================
  // 串口连接参数
  // =========================================================================
  char port[] = "/dev/ttyACM0";   // Linux 串口设备路径
  int rate = 115200;              // 波特率 (PTS 默认 115200)
  int parity = 0;                 // 校验: 0=无校验, 1=奇校验, 2=偶校验
  char byteSize = 8;              // 数据位: 8 bits

  // =========================================================================
  // 示例 1: 单线程模式
  // =========================================================================
  // 适用于低频轮询场景，主线程自行调用 readNextSample() 拉取数据。
  // 每次调用阻塞直到接收到完整一帧传感器数据。

  if (example == 1) {
    // 连接串口 (不启动后台线程)
    if (listener.connect(port, rate, parity, byteSize) == 0) {
      printf("main(): 成功连接到 %s\n", port);
    } else {
      printf("main(): 连接 %s 失败\n", port);
      return -1;
    }

    // 发送 Bias 请求 (触觉传感器的零点校准)
    if (listener.sendBiasRequest()) {
      printf("main(): Bias 请求发送成功\n");
    } else {
      printf("main(): Bias 请求发送失败\n");
      return -1;
    }
    // Bias 校准需要约 1.3 秒完成
    std::this_thread::sleep_for(std::chrono::milliseconds(1300));

    // 设置采样率
    if (listener.setSamplingRate(sampleRate)) {
      printf("main(): 采样率已设置为 %d Hz\n", sampleRate);
    } else {
      printf("main(): 设置采样率失败\n");
      return -1;
    }

    // 循环读取传感器数据
    int sampleCount = 0;
    while (true) {
      sampleCount++;
      if (listener.readNextSample(isFlush)) {
        // 每 100 帧打印一次，避免刷屏
        if (sampleCount % 100 == 0) {
          // 读取 SEN0 上第 3 号支柱 (pillar) 的 XYZ 三分力
          int pInd = 3;
          double force[NDIM];
          sen0.getPillarForces(pInd, force);
          for (int dInd = 0; dInd < NDIM; dInd++) {
            printf("S0_P%d: F%d = %.3f\n", pInd, dInd, force[dInd]);
          }
          printf("\n");

          // 读取 SEN0 上第 5 号支柱的 XYZ 三向位移
          pInd = 5;
          double displacement[NDIM];
          sen0.getPillarDisplacements(pInd, displacement);
          for (int dInd = 0; dInd < NDIM; dInd++) {
            printf("S0_P%d: D%d = %.3f\n", pInd, dInd, displacement[dInd]);
          }
          printf("\n");
        }
      } else {
        printf("main(): 读取采样数据失败\n");
      }
    }

    listener.disconnect();
    return 0;
  }

  // =========================================================================
  // 示例 2: 多线程模式 (推荐)
  // =========================================================================
  // 后台线程自动接收解析数据帧，主线程通过 get 方法随时获取最新数据。
  // 适合高频采集与实时控制场景。

  if (example == 2) {
    // 连接串口并启动后台数据监听线程
    if (listener.connectAndStartListening(port, rate, parity, byteSize,
                                          isFlush) == 0) {
      printf("main(): 成功连接 %s 并已启动后台监听\n", port);
    } else {
      printf("main(): 连接 %s 失败，未启动监听\n", port);
      return -1;
    }

    // Bias 校准
    if (listener.sendBiasRequest()) {
      printf("main(): Bias 请求发送成功\n");
    } else {
      printf("main(): Bias 请求发送失败\n");
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1300));

    // 设置采样率
    if (listener.setSamplingRate(sampleRate)) {
      printf("main(): 采样率已设置为 %d Hz\n", sampleRate);
    } else {
      printf("main(): 设置采样率失败\n");
      return -1;
    }

    // 每秒打印一次，持续 10 秒
    for (int i = 0; i < 10; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // 获取 SEN1 的全局 XYZ 合力
      double globalForce[NDIM];
      sen1.getGlobalForce(globalForce);
      for (int dInd = 0; dInd < NDIM; dInd++) {
        printf("main(): S1: 全局 F%d = %.3f\n", dInd, globalForce[dInd]);
      }
      printf("\n");

      // 获取 SEN1 上全部支柱的 XYZ 三向位移
      double allDisplacements[NDIM][MAX_NPILLAR];
      sen1.getAllDisplacements(allDisplacements);
      for (int pInd = 0; pInd < sen1.getNPillar(); pInd++) {
        printf("S1_P%d:", pInd);
        for (int dInd = 0; dInd < NDIM; dInd++) {
          printf("\tD%d: %.3f", dInd, allDisplacements[dInd][pInd]);
        }
        printf("\n");
      }
      printf("\n");
    }

    // 启动滑移检测
    std::this_thread::sleep_for(std::chrono::milliseconds(1300));
    printf("main(): 启动滑移检测\n");
    listener.startSlipDetection();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 停止滑移检测
    printf("main(): 停止滑移检测\n");
    listener.stopSlipDetection();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 停止后台监听并断开串口
    listener.stopListeningAndDisconnect();
    return 0;
  }

  return 0;
}
