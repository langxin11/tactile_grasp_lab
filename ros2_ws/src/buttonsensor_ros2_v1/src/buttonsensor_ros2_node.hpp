// 文件: buttonsensor_ros2_node.hpp
// 简要: ButtonSensor ROS2 节点类声明，封装触觉传感器监听器（PTSDKListener），
//       负责多传感器数据读取、发布 Bias 服务及传感器状态话题。

#pragma once

#include <stdio.h>
#include <sys/stat.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// ==== ROS2 基础设施 ====
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

// ==== 自定义消息与服务接口 ====
// 消息
#include "sensor_interfaces/msg/button_sensor_state.hpp"

// 服务
#include "sensor_interfaces/srv/bias_request.hpp"

// ==== 触觉传感器 SDK（PTSDK）====
#ifndef PTSDKCONSTANTS_H
#include <PTSDKConstants.h>
#endif
// 兼容性处理：PTSDKListener.h 将 BYTE 定义为 byte，同时通过 "using namespace std"
// 导入 std::byte，在 ROS2 环境下二者会产生歧义。此处通过条件编译规避该问题。
#ifdef __unix__
#define byte unsigned char
#endif
#ifndef PTSDKLISTENER_H
#include <PTSDKListener.h>
#endif
#ifdef __unix__
#undef byte
#endif
#ifndef PTSDKSENSOR_H
#include <PTSDKSensor.h>
#endif

class ButtonSensorNode : public rclcpp::Node {
 public:
  // 构造函数：加载参数、创建传感器实例、注册发布者与服务，并建立串口连接
  explicit ButtonSensorNode(const rclcpp::NodeOptions& options);

  // 析构函数：停止监听、关闭 CSV 日志并断开串口连接
  ~ButtonSensorNode() {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
    listener_.stopListeningAndDisconnect();
  }

  // 从 COM 口读取最新采样数据，并发布各传感器的状态消息
  void updateData();

  // 获取当前采样率（Hz）
  int getSamplingRate() { return sampling_rate_; }

 private:
  int hub_id_;         // 集线器 ID
  int n_sensors_;      // 实际使用的传感器数量
  std::string port_;   // 串口设备路径（如 /dev/ttyACM0）
  int baud_rate_;      // 串口波特率
  int parity_;         // 奇偶校验位（0=NONE, 1=ODD, 2=EVEN）
  int byte_size_;      // 数据位长度（默认 8 位）
  int sampling_rate_;  // 采样频率（Hz）

  // ==== 底层硬件抽象 ====
  PTSDKListener listener_;  // 传感器监听器，负责串口数据收发
  std::vector<std::unique_ptr<PTSDKSensor> > sensors_;  // 传感器实例列表

  // ==== ROS 发布者 ====
  // 每个传感器对应一个独立的话题发布者
  std::vector<rclcpp::Publisher<sensor_interfaces::msg::ButtonSensorState>::SharedPtr> sensor_pubs_;

  // ==== CSV 日志 ====
  bool csv_log_enabled_ = false;  // 是否启用 CSV 日志
  std::ofstream csv_file_;        // CSV 输出文件流
  std::string log_dir_;           // CSV 输出目录路径

  // ==== ROS 服务 ====
  // Bias 请求服务：供外部节点调用以触发传感器零位校准
  rclcpp::Service<sensor_interfaces::srv::BiasRequest>::SharedPtr send_bias_request_srv_;

  // ==== 服务回调函数 ====
  // Bias 请求服务回调：向监听器发送 Bias（偏置校准）指令，等待硬件响应后返回结果
  bool sendBiasRequestSrvCallback(
      [[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::BiasRequest::Request> request,
      std::shared_ptr<sensor_interfaces::srv::BiasRequest::Response> response);
};
