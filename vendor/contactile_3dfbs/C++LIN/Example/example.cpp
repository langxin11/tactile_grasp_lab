// 文件: USER_APP_EXAMPLE.cpp
// 简要: Contactile 3DFBS 三维力按钮传感器 (3-dimensional force button sensor)
//       C++ SDK 示例程序。演示最多 10 个传感器的多线程/单线程两种
//       数据读取模式。
// 适用平台: Linux (x86_64)
// 用法:
//   1. 将 vendor/contactile_3dfbs/C++LIN/ 添加到头文件搜索路径
//   2. 链接 vendor/contactile_3dfbs/C++LIN/libPTSDK.a
//   3. 连接 -lpthread
//   编译示例 (g++):
//     g++ -std=c++17 -I.. -L.. USER_APP_EXAMPLE.cpp -lPTSDK -lpthread -o example
// 原厂文档: 3DFBSC++LIN_B1.0_MAN_DEC24.pdf

#include <stdio.h>
#include <ctime>
#include <chrono>
#include <thread>

#include "PTSDKConstants.h"
#include "PTSDKListener.h"
#include "PTSDKSensor.h"
#include "stdafx.h"

int main() {
  // =========================================================================
  // 运行参数配置
  // =========================================================================

  int example;
  // example = 1;  // 多线程模式 (后台线程自动读取)
  example = 2;    // 单线程模式 (主线程阻塞式读取)

  // 是否将传感器数据写入 .csv 日志文件
  bool isLogging;
  isLogging = true;
  // isLogging = false;

  // =========================================================================
  // 初始化传感器与监听器
  // =========================================================================

  // 3DFBS 通迅集线器 (DEV001) 最多支持 10 个传感器同时连接
  //   端口 A: sen0 - sen4 (5 个)
  //   端口 B: sen5 - sen9 (5 个)
  // 每个传感器对应一个 PTSDKSensor 对象
  //
  // 注意: 请根据实际连接的传感器数量增减，无硬件连接时
  //       多创建的传感器对象不会产生数据

  PTSDKSensor sen0 = PTSDKSensor();
  PTSDKSensor sen1 = PTSDKSensor();
  PTSDKSensor sen2 = PTSDKSensor();
  PTSDKSensor sen3 = PTSDKSensor();
  PTSDKSensor sen4 = PTSDKSensor();
  PTSDKSensor sen5 = PTSDKSensor();
  PTSDKSensor sen6 = PTSDKSensor();
  PTSDKSensor sen7 = PTSDKSensor();
  PTSDKSensor sen8 = PTSDKSensor();
  PTSDKSensor sen9 = PTSDKSensor();

  // 创建监听器并注册全部传感器
  PTSDKListener listener = PTSDKListener(isLogging);
  listener.addSensor(&sen0);
  listener.addSensor(&sen1);
  listener.addSensor(&sen2);
  listener.addSensor(&sen3);
  listener.addSensor(&sen4);
  listener.addSensor(&sen5);
  listener.addSensor(&sen6);
  listener.addSensor(&sen7);
  listener.addSensor(&sen8);
  listener.addSensor(&sen9);

  // =========================================================================
  // 串口连接参数
  // =========================================================================
  char port[] = "/dev/ttyACM0";    // Linux 串口设备路径
  int rate = 115200;               // 波特率 (3DFBS 默认 115200)
  int parity = 0;                  // 校验: 0=无校验, 1=奇校验, 2=偶校验
  char byteSize = 8;               // 数据位: 8 bits
  // 日志写入频率 (Hz): LOG_RATE_100 / LOG_RATE_500 / LOG_RATE_1000
  int logFileRate = LOG_RATE_1000;

  // =========================================================================
  // 示例 1: 多线程模式 (推荐)
  // =========================================================================
  // 后台线程自动接收解析数据帧，主线程调用 get 方法获取最新数据。
  // 适用于高频连续采集场景。

  if (example == 1) {
    // 连接串口并启动后台数据监听线程
    int err = listener.connectAndStartListening(port, rate, parity, byteSize,
                                                logFileRate);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Bias 校准 (3DFBS 传感器的零点校准)
    if (listener.sendBiasRequest()) {
      printf("Bias 请求发送成功\n");
    } else {
      printf("Bias 请求发送失败\n");
      return -1;
    }

    // 每秒打印一次传感器数据，持续 10 秒
    for (int i = 0; i < 10; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // 获取 SEN0 的全局 XYZ 合力 (所有支柱合力的矢量和)
      double globalForce[NDIM];
      sen0.getGlobalForce(globalForce);
      for (int dInd = 0; dInd < NDIM; dInd++) {
        printf("S0: 全局 F%d = %.3f\n", dInd, globalForce[dInd]);
      }
      printf("\n");
    }

    // 停止后台监听并断开串口
    listener.stopListeningAndDisconnect();
    return 0;
  }

  // =========================================================================
  // 示例 2: 单线程模式
  // =========================================================================
  // 主线程显式调用 readNextSample() 阻塞等待每帧数据。
  // 适用于低频、事件驱动的读取场景。

  if (example == 2) {
    // 连接串口 (不启动后台线程)
    int err = listener.connect(port, rate, parity, byteSize);
    if (err) {
      printf("无法连接到 %s，错误码: %d\n", port, err);
      return -1;
    }
    printf("成功连接到 %s\n", port);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Bias 校准
    if (listener.sendBiasRequest()) {
      printf("Bias 请求发送成功\n");
    } else {
      printf("Bias 请求发送失败\n");
      return -1;
    }

    // 循环读取传感器数据
    while (true) {
      bool res = listener.readNextSample();
      if (res) {
        // 读取成功，获取 SEN0 全局合力
        printf("main(): 采样数据读取成功\n");
        double globalForce[NDIM];
        sen0.getGlobalForce(globalForce);
        for (int dInd = 0; dInd < NDIM; dInd++) {
          printf("S0: 全局 F%d = %.3f\n", dInd, globalForce[dInd]);
        }
        printf("\n");
      } else {
        printf("main(): 采样数据读取失败\n");
      }
    }

    listener.disconnect();
    return 0;
  }

  return 0;
}
