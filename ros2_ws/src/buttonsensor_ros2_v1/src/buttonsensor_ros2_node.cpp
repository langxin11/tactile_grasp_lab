// 文件: buttonsensor_ros2_node.cpp
// 简要: ButtonSensor ROS2 节点实现。通过串口连接触觉传感器 Hub，
//       以固定采样率轮询传感器数据并通过 ROS 话题发布，
//       同时提供 Bias 请求服务支持传感器校准。

#include "buttonsensor_ros2_node.hpp"

// ============================================================================
// 构造函数：初始化 ROS2 节点
//   1. 声明并加载 yaml/launch 参数（hub_id、n_sensors、串口配置等）
//   2. 创建 PTSDKSensor 实例并注册到监听器
//   3. 为每个传感器创建独立的话题发布者
//   4. 注册 Bias 请求服务
//   5. 建立串口连接
// ============================================================================
ButtonSensorNode::ButtonSensorNode([[maybe_unused]] const rclcpp::NodeOptions& options)
    : Node("buttonsensor_ros2_v1_node"), listener_(true) {
  // ---- 加载参数 ----
  RCLCPP_INFO(this->get_logger(), "Loading parameters...\n");
  hub_id_ = this->declare_parameter("hub_id", 0);
  RCLCPP_INFO(this->get_logger(), "Hub id: %d", hub_id_);

  n_sensors_ = this->declare_parameter("n_sensors", 0);
  if (n_sensors_ > MAX_NSENSOR || n_sensors_ < 1) {
    RCLCPP_ERROR(this->get_logger(),
                 "\033[91mInvalid number of sensors!  %d selected (must be no more than %d)\033[0m",
                 n_sensors_, MAX_NSENSOR);
  } else {
    RCLCPP_INFO(this->get_logger(), "\033[92mUsing %d sensor/s\033[0m", n_sensors_);
  }

  port_ = this->declare_parameter("com_port", std::string(""));
  RCLCPP_INFO(this->get_logger(), "Reading from serial COM port: %s", port_.c_str());

  baud_rate_ = this->declare_parameter("baud_rate", 0);
  RCLCPP_INFO(this->get_logger(), "Baud rate: %d Hz", baud_rate_);

  parity_ = this->declare_parameter("parity", 0);
  RCLCPP_INFO(this->get_logger(), "Parity set to: %d (0=PARITY_NONE, 1=PARITY_ODD, 2=PARITY_EVEN)",
              parity_);

  byte_size_ = this->declare_parameter("byte_size", 0);
  RCLCPP_INFO(this->get_logger(), "Byte size: %d bits", byte_size_);

  sampling_rate_ = this->declare_parameter("sampling_rate", 0);
  RCLCPP_INFO(this->get_logger(), "Sampling rate: %d Hz", sampling_rate_);

  RCLCPP_INFO(this->get_logger(), "Loaded parameters.\n");

  // ---- 创建传感器实例 ----
  // 将 sensors_ 容器大小调整至实际传感器数量
  sensors_.resize(n_sensors_);

  RCLCPP_INFO(this->get_logger(), "Creating sensors...\n");

  for (size_t sensor_id = 0; sensor_id < static_cast<size_t>(n_sensors_); sensor_id++) {
    RCLCPP_INFO(this->get_logger(), "Creating sensor %zu...", sensor_id);
    auto sensor = std::make_unique<PTSDKSensor>();
    RCLCPP_INFO(this->get_logger(), "Adding sensor %zu to listener...", sensor_id);
    listener_.addSensor(sensor.get());
    RCLCPP_INFO(this->get_logger(), "Added sensor %zu to listener!\n", sensor_id);
    sensors_[sensor_id] = std::move(sensor);

    // 为每个传感器创建独立的话题发布者
    std::string topic = "/hub_" + std::to_string(hub_id_) + "/sensor_" + std::to_string(sensor_id);
    sensor_pubs_.push_back(
        this->create_publisher<sensor_interfaces::msg::ButtonSensorState>(topic, sampling_rate_));
  }

  // ---- 注册服务 ----
  RCLCPP_INFO(this->get_logger(), "Starting services...");
  std::string service_name = "/hub_" + std::to_string(hub_id_) + "/send_bias_request";
  send_bias_request_srv_ = this->create_service<sensor_interfaces::srv::BiasRequest>(
      service_name,
      [this]([[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::BiasRequest::Request>
                 request,
             std::shared_ptr<sensor_interfaces::srv::BiasRequest::Response> response) {
        return sendBiasRequestSrvCallback(request, response);
      });
  RCLCPP_INFO(this->get_logger(), "Started %s service", service_name.c_str());

  // ---- 建立串口连接 ----
  RCLCPP_INFO(this->get_logger(), "Connecting to %s port...", port_.c_str());
  if (listener_.connect(port_.c_str(), baud_rate_, parity_, char(byte_size_))) {
    RCLCPP_FATAL(this->get_logger(), "\033[91mFailed to connect to port: %s\033[0m", port_.c_str());
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(), "\033[92mConnected to port: %s\033[0m", port_.c_str());
  }

  // 注意：以下采样率设置代码已注释。SDK 内部已通过参数完成配置，
  // 如需运行时动态调整采样率，可取消注释并调用 setSamplingRate。
  // // Set sampling rate
  // RCLCPP_INFO(this->get_logger(), "Setting sampling rate to %u...", sampling_rate_);
  // if (!listener_.setSamplingRate(sampling_rate_)) {
  // 	RCLCPP_WARN(this->get_logger(), "\033[91mFailed to set sampling rate to: %u\033[0m",
  // sampling_rate_); } else { 	RCLCPP_INFO(this->get_logger(), "\033[92mSampling rate set to
  // %u\033[0m", sampling_rate_);
  // }
}

// ============================================================================
// updateData — 读取并发布传感器数据
//   从监听器读取最新一帧串口数据，遍历所有传感器将全局力
//   转换为 ButtonSensorState 消息并通过对应话题发布。
// ============================================================================
void ButtonSensorNode::updateData() {
  if (n_sensors_ == 0) {
    return;
  }

  // 从 COM 口读取最新一帧采样数据
  listener_.readNextSample();

  for (size_t sensor_id = 0; sensor_id < sensors_.size(); sensor_id++) {
    sensor_interfaces::msg::ButtonSensorState ss_msg;

    auto time = now();

    long timestamp_us = sensors_[sensor_id]->getTimestamp_us();
    ss_msg.tus = timestamp_us;

    // 读取全局力（三维分量 X, Y, Z），单位为 counts（原始 ADC 值）
    double globalForce[NDIM];
    sensors_[sensor_id]->getGlobalForce(globalForce);
    ss_msg.gfx = static_cast<float>(globalForce[X_IND]);  // 全局力 X 分量
    ss_msg.gfy = static_cast<float>(globalForce[Y_IND]);  // 全局力 Y 分量
    ss_msg.gfz = static_cast<float>(globalForce[Z_IND]);  // 全局力 Z 分量

    // 发布传感器状态消息到对应话题
    sensor_pubs_[sensor_id]->publish(ss_msg);
  }
}

// ============================================================================
// sendBiasRequestSrvCallback — Bias 请求服务回调
//   通过监听器向传感器 Hub 发送 Bias（偏置/校准）指令，
//   等待 100ms 确保硬件指令执行完毕，返回执行结果。
//   用途：消除传感器零漂，确保力读数基准归零。
// ============================================================================
bool ButtonSensorNode::sendBiasRequestSrvCallback(
    [[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::BiasRequest::Request> req,
    std::shared_ptr<sensor_interfaces::srv::BiasRequest::Response> resp) {
  RCLCPP_INFO(this->get_logger(), "sendBiasRequest callback");
  resp->result = listener_.sendBiasRequest();
  // 等待 100ms 确保硬件指令执行完毕
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return resp->result;
}

// ============================================================================
// main — 程序入口
//   初始化 ROS2，创建 ButtonSensorNode 实例，进入以采样率为节拍的
//   主循环：spin_some(处理回调) → sleep(按采样率休眠) → updateData(读取并发布数据)
// ============================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ButtonSensorNode>(rclcpp::NodeOptions());

  rclcpp::Rate loop_rate(node->getSamplingRate());

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);  // 处理 ROS 回调（服务请求等）
    loop_rate.sleep();        // 按采样率节拍休眠，保持稳定的数据发布频率
    node->updateData();       // 读取传感器数据并发布
  }

  rclcpp::shutdown();

  return 0;
}
