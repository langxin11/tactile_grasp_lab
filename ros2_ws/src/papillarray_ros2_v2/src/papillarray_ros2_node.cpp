// 文件: papillarray_ros2_node.cpp
// 简要: PapillArray 触觉传感器 ROS2 节点实现。
//       初始化传感器串口连接、配置参数，定期读取传感器数据（全局力/力矩、
//       柱体位移/力、滑动状态等），并通过 ROS 话题发布；同时提供滑动检测
//       启停和偏置请求服务。

#include "papillarray_ros2_node.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

// ==== 构造函数：参数加载、传感器初始化、服务注册、串口连接、定时器启动 ====

PapillArrayNode::PapillArrayNode([[maybe_unused]] const rclcpp::NodeOptions& options)
    : Node("papillarray_ros2_v2_node"), listener_(true) {
  // listener_ 参数: isLogging=true 时启用 CSV 日志记录，日志输出到 /home/.ros/Logs

  // ==== 1. 加载 ROS 参数 ====

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

  is_flush_ = this->declare_parameter("is_flush", false);
  RCLCPP_INFO(this->get_logger(), "Is Flush: %d", is_flush_);

  sampling_rate_ = this->declare_parameter("sampling_rate", 0);
  RCLCPP_INFO(this->get_logger(), "Sampling rate: %d Hz", sampling_rate_);

  RCLCPP_INFO(this->get_logger(), "Loaded parameters.\n");

  // ==== 2. 创建传感器实例并注册到监听器 ====

  sensors_.resize(n_sensors_);

  RCLCPP_INFO(this->get_logger(), "Creating sensors...\n");

  for (size_t sensor_id = 0; sensor_id < static_cast<size_t>(n_sensors_); sensor_id++) {
    RCLCPP_INFO(this->get_logger(), "Creating sensor %zu...", sensor_id);
    auto sensor = std::make_unique<PTSDKSensor>();
    RCLCPP_INFO(this->get_logger(), "Adding sensor %zu to listener...", sensor_id);
    // 将裸指针注册到监听器，生命周期由 sensors_ 管理
    listener_.addSensor(sensor.get());
    RCLCPP_INFO(this->get_logger(), "Added sensor %zu to listener!\n", sensor_id);
    sensors_[sensor_id] = std::move(sensor);

    // 为每个传感器创建独立的 ROS 发布者，话题格式 /hub_<hub_id>/sensor_<sensor_id>
    std::string topic = "/hub_" + std::to_string(hub_id_) + "/sensor_" + std::to_string(sensor_id);
    sensor_pubs_.push_back(this->create_publisher<sensor_interfaces::msg::SensorState>(
        topic, rclcpp::SensorDataQoS()));
  }

  // ==== 3. 注册 ROS 服务 ====

  RCLCPP_INFO(this->get_logger(), "Starting services...");
  std::string service_name = "/hub_" + std::to_string(hub_id_) + "/start_slip_detection";
  start_sd_srv_ = this->create_service<sensor_interfaces::srv::StartSlipDetection>(
      service_name,
      [this]([[maybe_unused]] const std::shared_ptr<
                 sensor_interfaces::srv::StartSlipDetection::Request>
                 request,
             std::shared_ptr<sensor_interfaces::srv::StartSlipDetection::Response> response) {
        return startSlipDetectionSrvCallback(request, response);
      });
  RCLCPP_INFO(this->get_logger(), "Started %s service", service_name.c_str());

  service_name = "/hub_" + std::to_string(hub_id_) + "/stop_slip_detection";
  stop_sd_srv_ = this->create_service<sensor_interfaces::srv::StopSlipDetection>(
      service_name,
      [this](
          [[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::StopSlipDetection::Request>
              request,
          std::shared_ptr<sensor_interfaces::srv::StopSlipDetection::Response> response) {
        return stopSlipDetectionSrvCallback(request, response);
      });
  RCLCPP_INFO(this->get_logger(), "Started %s service", service_name.c_str());

  service_name = "/hub_" + std::to_string(hub_id_) + "/send_bias_request";
  send_bias_request_srv_ = this->create_service<sensor_interfaces::srv::BiasRequest>(
      service_name,
      [this]([[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::BiasRequest::Request>
                 request,
             std::shared_ptr<sensor_interfaces::srv::BiasRequest::Response> response) {
        return sendBiasRequestSrvCallback(request, response);
      });
  RCLCPP_INFO(this->get_logger(), "Started %s service", service_name.c_str());

  // ==== 4. 建立串口连接 ====

  RCLCPP_INFO(this->get_logger(), "Connecting to %s port...", port_.c_str());
  if (listener_.connectAndStartListening(port_.c_str(), baud_rate_, parity_,
                                         static_cast<char>(byte_size_), is_flush_)) {
    RCLCPP_FATAL(this->get_logger(), "\033[91mFailed to connect to port: %s\033[0m", port_.c_str());
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(), "\033[92mConnected to port: %s\033[0m", port_.c_str());
  }

  // ==== 5. 设置采样率并启动定时器 ====

  RCLCPP_INFO(this->get_logger(), "Setting sampling rate to %u...", sampling_rate_);
  if (!listener_.setSamplingRate(sampling_rate_)) {
    RCLCPP_WARN(this->get_logger(), "\033[91mFailed to set sampling rate to: %u\033[0m",
                sampling_rate_);
  } else {
    RCLCPP_INFO(this->get_logger(), "\033[92mSampling rate set to %u\033[0m", sampling_rate_);
  }

  if (sampling_rate_ > 0) {
    // 将采样率 (Hz) 转换为定时器周期 (纳秒)
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / sampling_rate_));
    update_timer_ = this->create_wall_timer(period, [this]() { updateData(); });
  } else {
    RCLCPP_ERROR(this->get_logger(), "\033[91mInvalid sampling rate: %d\033[0m", sampling_rate_);
  }
}

// ==== 定时器回调：从所有传感器读取最新数据并发布 ====

void PapillArrayNode::updateData() {
  if (n_sensors_ == 0) {
    return;
  }

  for (size_t sensor_id = 0; sensor_id < sensors_.size(); sensor_id++) {
    auto ss_msg = std::make_shared<sensor_interfaces::msg::SensorState>();

    ss_msg->header.stamp = this->now();
    // frame_id 标识传感器在系统中的位置
    ss_msg->header.frame_id =
        "hub_" + std::to_string(hub_id_) + "/sensor_" + std::to_string(sensor_id);

    int64_t timestamp_us = sensors_[sensor_id]->getTimestamp_us();
    ss_msg->tus = timestamp_us;

    double globalForce[NDIM];
    double globalTorque[NDIM];

    // 读取全局力 (X, Y, Z)
    sensors_[sensor_id]->getGlobalForce(globalForce);
    ss_msg->gfx = static_cast<float>(globalForce[X_IND]);
    ss_msg->gfy = static_cast<float>(globalForce[Y_IND]);
    ss_msg->gfz = static_cast<float>(globalForce[Z_IND]);
    // 读取全局力矩 (X, Y, Z)
    sensors_[sensor_id]->getGlobalTorque(globalTorque);
    ss_msg->gtx = static_cast<float>(globalTorque[X_IND]);
    ss_msg->gty = static_cast<float>(globalTorque[Y_IND]);
    ss_msg->gtz = static_cast<float>(globalTorque[Z_IND]);

    // 摩擦系数估计值
    ss_msg->friction_est = static_cast<float>(sensors_[sensor_id]->getFrictionEstimate());

    // 目标抓取力，无摩擦估计时返回 -1
    ss_msg->target_grip_force = static_cast<float>(sensors_[sensor_id]->getTargetGripForce());

    int n_pillar = sensors_[sensor_id]->getNPillar();

    bool is_sd_active;
    bool is_ref_loaded;
    std::array<bool, MAX_NPILLAR> contact_states{};
    std::array<int, MAX_NPILLAR> slip_states{};

    sensors_[sensor_id]->getAllSlipStatus(&is_sd_active, &is_ref_loaded, contact_states.data(),
                                          slip_states.data());

    ss_msg->is_sd_active = is_sd_active;
    ss_msg->is_ref_loaded = is_ref_loaded;
    ss_msg->is_contact = false;

    // 遍历传感器阵列中所有柱体的状态
    for (int pillar_id = 0; pillar_id < n_pillar; pillar_id++) {
      auto ps_msg = sensor_interfaces::msg::PillarState();

      ps_msg.id = pillar_id;
      ps_msg.slip_state = slip_states[pillar_id];
      ps_msg.in_contact = contact_states[pillar_id];

      // 任意一个柱体处于接触状态，则整个传感器标记为接触
      ss_msg->is_contact = ss_msg->is_contact | ps_msg.in_contact;

      // 读取柱体位移 (X, Y, Z)
      double pillar_d[NDIM];
      sensors_[sensor_id]->getPillarDisplacements(pillar_id, pillar_d);
      ps_msg.dx = static_cast<float>(pillar_d[X_IND]);
      ps_msg.dy = static_cast<float>(pillar_d[Y_IND]);
      ps_msg.dz = static_cast<float>(pillar_d[Z_IND]);

      // 读取柱体受力 (X, Y, Z)
      double pillar_f[NDIM];
      sensors_[sensor_id]->getPillarForces(pillar_id, pillar_f);
      ps_msg.fx = static_cast<float>(pillar_f[X_IND]);
      ps_msg.fy = static_cast<float>(pillar_f[Y_IND]);
      ps_msg.fz = static_cast<float>(pillar_f[Z_IND]);

      ss_msg->pillars.push_back(ps_msg);

      // 调试用：打印第一个柱体的 Z 方向受力
      if (pillar_id == 0) {
        // RCLCPP_INFO(this->get_logger(), "From C++API: %.2f; From ROS: %.2f\n",pillar_f[2],
        // ps_msg.fZ);
      }
    }

    // 发布该传感器的完整状态消息
    sensor_pubs_[sensor_id]->publish(*ss_msg);
  }
}

// ==== 服务回调函数 ====

// 启动滑动检测服务回调：调用 SDK 启动滑动检测算法
bool PapillArrayNode::startSlipDetectionSrvCallback(
    [[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::StartSlipDetection::Request> req,
    std::shared_ptr<sensor_interfaces::srv::StartSlipDetection::Response> resp) {
  RCLCPP_INFO(this->get_logger(), "startSlipDetection callback");
  resp->result = listener_.startSlipDetection();
  return resp->result;
}

// 停止滑动检测服务回调：调用 SDK 停止滑动检测算法
bool PapillArrayNode::stopSlipDetectionSrvCallback(
    [[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::StopSlipDetection::Request> req,
    std::shared_ptr<sensor_interfaces::srv::StopSlipDetection::Response> resp) {
  RCLCPP_INFO(this->get_logger(), "stopSlipDetection callback");
  resp->result = listener_.stopSlipDetection();
  return resp->result;
}

// 偏置请求服务回调：发送偏置校准命令，用于传感器零位归零
bool PapillArrayNode::sendBiasRequestSrvCallback(
    [[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::BiasRequest::Request> req,
    std::shared_ptr<sensor_interfaces::srv::BiasRequest::Response> resp) {
  RCLCPP_INFO(this->get_logger(), "sendBiasRequest callback");
  resp->result = listener_.sendBiasRequest();
  return resp->result;
}

// ==== 主函数：初始化 ROS2 并进入事件循环 ====

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PapillArrayNode>(rclcpp::NodeOptions());
  // 阻塞式旋转，等待回调触发
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
