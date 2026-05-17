// 文件: papillarray_ros2_node.hpp
// 简要: PapillArray 触觉传感器 ROS2 节点头文件，声明节点类及其接口。
//       通过串口连接 Contactile PapillArray 传感器硬件，发布传感器状态消息，
//       并提供滑动检测与偏置请求服务。

#ifndef PAPILLARRAY_ROS2_V2_NODE_H_
#define PAPILLARRAY_ROS2_V2_NODE_H_

// ==== 标准库头文件 ====

#include <stdio.h>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

// ==== ROS2 核心头文件 ====

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

// ==== 自定义消息 ====

#include "sensor_interfaces/msg/pillar_state.hpp"
#include "sensor_interfaces/msg/sensor_state.hpp"

// ==== 自定义服务 ====

#include "sensor_interfaces/srv/bias_request.hpp"
#include "sensor_interfaces/srv/start_slip_detection.hpp"
#include "sensor_interfaces/srv/stop_slip_detection.hpp"

// GCC 13 + C++17 兼容性处理：std::byte 与 SDK 中的 BYTE 宏冲突，
// PTSDKParser.h 和 PTSDKListener.h 已改用 #define BYTE unsigned char。
// 不再使用: typedef unsigned char byte;

// ==== PapillArray SDK 头文件 ====

#ifndef PTSDKCONSTANTS_H
#include <PTSDKConstants.h>
#endif
#ifndef PTSDKLISTENER_H
#include <PTSDKListener.h>
#endif
#ifndef PTSDKSENSOR_H
#include <PTSDKSensor.h>
#endif

// ==== PapillArrayNode 节点类 ====
// 封装传感器初始化、数据采集、ROS 发布与服务回调。
// 每个节点对应一个 Hub，通过 hub_id 区分多 Hub 场景。

class PapillArrayNode : public rclcpp::Node {
public:
    // 构造函数：加载参数并初始化传感器、发布者、服务及串口连接
    PapillArrayNode(const rclcpp::NodeOptions & options);

    // 析构函数：停止监听并断开串口连接
    ~PapillArrayNode() {
        listener_.stopListeningAndDisconnect();
    }

    // 定时器回调：从传感器读取最新数据并发布到 ROS 话题
    void updateData();

private:
    // Hub 标识符，用于话题命名 /hub_<hub_id>/sensor_<sensor_id>
    int hub_id_;

    // 连接的传感器数量
    int n_sensors_;

    // ==== 串口参数 ====
    std::string port_;       // 串口设备路径（PTS 传感器侧，非 Robotiq 夹爪）
    int baud_rate_;          // 波特率
    int parity_;             // 校验位: 0=NONE, 1=ODD, 2=EVEN
    int byte_size_;          // 字节位数
    bool is_flush_;          // 缓冲区满时是否清空硬件输入缓冲
    int sampling_rate_;      // 采样频率 (Hz): 100, 250, 500 或 1000

    // SDK 监听器，管理串口连接与数据流
    PTSDKListener listener_;
    // 传感器对象列表，每个对象对应一个物理传感器
    std::vector<std::unique_ptr<PTSDKSensor> > sensors_;

    // ==== ROS 发布者 ====
    // 每个传感器一个发布者，发布 SensorState 消息到对应话题
    std::vector<rclcpp::Publisher<sensor_interfaces::msg::SensorState>::SharedPtr> sensor_pubs_;
    // 定时器，按采样率周期触发 updateData()
    rclcpp::TimerBase::SharedPtr update_timer_;

    // ==== ROS 服务 ====
    rclcpp::Service<sensor_interfaces::srv::StartSlipDetection>::SharedPtr start_sd_srv_;
    rclcpp::Service<sensor_interfaces::srv::StopSlipDetection>::SharedPtr stop_sd_srv_;
    rclcpp::Service<sensor_interfaces::srv::BiasRequest>::SharedPtr send_bias_request_srv_;

    // ==== 服务回调函数 ====
    // 启动滑动检测，返回操作是否成功
    bool startSlipDetectionSrvCallback([[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::StartSlipDetection::Request> request,
                    std::shared_ptr<sensor_interfaces::srv::StartSlipDetection::Response> response);
    // 停止滑动检测，返回操作是否成功
    bool stopSlipDetectionSrvCallback([[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::StopSlipDetection::Request> request,
                    std::shared_ptr<sensor_interfaces::srv::StopSlipDetection::Response> response);
    // 发送偏置请求，用于传感器零位校准
    bool sendBiasRequestSrvCallback([[maybe_unused]] const std::shared_ptr<sensor_interfaces::srv::BiasRequest::Request> request,
                    std::shared_ptr<sensor_interfaces::srv::BiasRequest::Response> response);

};

#endif // PAPILLARRAY_ROS2_V2_NODE_H_
