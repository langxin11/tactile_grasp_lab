/**
 * Contactile 3DFBS C++ 多线程采集与测速示例
 *
 * 编译运行:
 *   bash scripts/run_cpp.sh -- --help
 *   bash scripts/run_cpp.sh -- --confirm-no-load --duration 2 --output /tmp/cpp_rate.csv
 *
 * 不依赖 ROS2，直接链接原厂 libPTSDK.a 静态库。
 * 使用 connectAndStartListening() 多线程模式：后台线程自动接收数据帧，
 * 主线程通过 getGlobalForce() 随时获取最新值，避免阻塞式 readNextSample() 的调度开销。
 *
 * 注意: 只 include PTSDKListener.h，不直接 include PTSDKSensor.h。
 *       后者的 include guard 存在缺陷，会导致 class redefinition 编译错误。
 */

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#include "PTSDKConstants.h"
#include "PTSDKListener.h"

static volatile std::sig_atomic_t g_running = 1;

struct Options {
  std::string port = "/dev/ttyACM0";
  int baud_rate = 9600;
  int sensor_index = 0;
  double duration_sec = 0.0;
  std::string output_path;
  int print_every = 0;
  bool confirm_no_load = false;
  bool bias = true;
  bool is_flush = true;
};

void signalHandler(int) {
  g_running = 0;
}

void printUsage(const char* program) {
  printf("用法: %s [选项]\n", program);
  printf("\n");
  printf("选项:\n");
  printf("  --port PATH              串口设备，默认 /dev/ttyACM0\n");
  printf("  --baud-rate N            串口波特率，默认 9600\n");
  printf("  --sensor N               传感器索引 0..%d，默认 0\n", MAX_NSENSOR - 1);
  printf("  --duration SEC           记录时长；不指定则持续运行直到 Ctrl+C\n");
  printf("  --output PATH            写入 CSV；不指定则只统计\n");
  printf("  --print-every N          每 N 个样本打印一行；默认 0 表示不逐帧打印\n");
  printf("  --confirm-no-load        确认传感器无负载，允许 bias 校准\n");
  printf("  --no-bias                跳过 bias 校准\n");
  printf("  --no-flush               禁用串口缓冲区清空\n");
  printf("  --help                   显示帮助\n");
  printf("\n");
  printf("示例:\n");
  printf(
      "  bash scripts/run_cpp.sh -- --confirm-no-load --duration 2 --output /tmp/cpp_rate.csv\n");
  printf("  bash scripts/run_cpp.sh -- --baud-rate 115200 --confirm-no-load --duration 2\n");
}

bool parseInt(const char* text, int* value) {
  char* end = nullptr;
  long parsed = std::strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  *value = static_cast<int>(parsed);
  return true;
}

bool parseDouble(const char* text, double* value) {
  char* end = nullptr;
  double parsed = std::strtod(text, &end);
  if (end == text || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

bool requireValue(int argc, char* argv[], int index) {
  if (index + 1 < argc) {
    return true;
  }
  fprintf(stderr, "参数缺少值: %s\n", argv[index]);
  return false;
}

bool parseArgs(int argc, char* argv[], Options* options) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--port") {
      if (!requireValue(argc, argv, i)) {
        return false;
      }
      options->port = argv[++i];
    } else if (arg == "--baud-rate") {
      if (!requireValue(argc, argv, i) || !parseInt(argv[++i], &options->baud_rate)) {
        fprintf(stderr, "--baud-rate 必须是整数\n");
        return false;
      }
    } else if (arg == "--sensor") {
      if (!requireValue(argc, argv, i) || !parseInt(argv[++i], &options->sensor_index)) {
        fprintf(stderr, "--sensor 必须是整数\n");
        return false;
      }
    } else if (arg == "--duration") {
      if (!requireValue(argc, argv, i) || !parseDouble(argv[++i], &options->duration_sec)) {
        fprintf(stderr, "--duration 必须是秒数\n");
        return false;
      }
    } else if (arg == "--output") {
      if (!requireValue(argc, argv, i)) {
        return false;
      }
      options->output_path = argv[++i];
    } else if (arg == "--print-every") {
      if (!requireValue(argc, argv, i) || !parseInt(argv[++i], &options->print_every)) {
        fprintf(stderr, "--print-every 必须是整数\n");
        return false;
      }
    } else if (arg == "--confirm-no-load") {
      options->confirm_no_load = true;
    } else if (arg == "--no-bias") {
      options->bias = false;
    } else if (arg == "--no-flush") {
      options->is_flush = false;
    } else if (arg.rfind("-", 0) != 0 && i == 1) {
      options->port = arg;
    } else {
      fprintf(stderr, "未知参数: %s\n", arg.c_str());
      return false;
    }
  }

  if (options->sensor_index < 0 || options->sensor_index >= MAX_NSENSOR) {
    fprintf(stderr, "--sensor 必须在 0..%d 之间\n", MAX_NSENSOR - 1);
    return false;
  }
  if (options->baud_rate <= 0) {
    fprintf(stderr, "--baud-rate 必须大于 0\n");
    return false;
  }
  if (options->duration_sec < 0.0) {
    fprintf(stderr, "--duration 不能小于 0\n");
    return false;
  }
  if (options->print_every < 0) {
    fprintf(stderr, "--print-every 不能小于 0\n");
    return false;
  }
  if (options->bias && !options->confirm_no_load) {
    fprintf(
        stderr,
        "拒绝执行: bias 前必须确认传感器无负载，请添加 --confirm-no-load，或用 --no-bias 跳过。\n");
    return false;
  }
  return true;
}

int main(int argc, char* argv[]) {
  Options options;
  if (!parseArgs(argc, argv, &options)) {
    fprintf(stderr, "运行 %s --help 查看用法\n", argv[0]);
    return 2;
  }

  constexpr int parity = 0;
  constexpr char byte_size = 8;
  constexpr bool enable_log = false;

  printf("=== Contactile 3DFBS C++ Reader (多线程模式) ===\n");
  printf("串口: %s @ %d baud, sensor S%d\n", options.port.c_str(), options.baud_rate,
         options.sensor_index);
  if (options.duration_sec > 0.0) {
    printf("记录时长: %.3f s\n", options.duration_sec);
  } else {
    printf("记录时长: 持续运行，按 Ctrl+C 停止\n");
  }
  if (!options.output_path.empty()) {
    printf("CSV: %s\n", options.output_path.c_str());
  }

  constexpr int N_SENSORS = MAX_NSENSOR;
  PTSDKSensor sensors[N_SENSORS];

  PTSDKListener listener(enable_log);
  for (int i = 0; i < N_SENSORS; ++i) {
    listener.addSensor(&sensors[i]);
  }

  // 多线程模式：后台线程自动接收数据帧，主线程通过 getGlobalForce() 获取最新值
  int ret = listener.connectAndStartListening(options.port.c_str(), options.baud_rate, parity,
                                              byte_size, options.is_flush);
  if (ret != 0) {
    fprintf(stderr, "串口连接失败: %s (错误码: %d)\n", options.port.c_str(), ret);
    fprintf(stderr, "提示: 检查设备是否插入，用户是否在 dialout 组\n");
    return 1;
  }
  printf("串口连接成功，后台监听已启动\n");

  if (options.bias) {
    printf("发送 Bias 请求，请保持传感器无负载...\n");
    if (!listener.sendBiasRequest()) {
      fprintf(stderr, "Bias 请求失败\n");
      listener.stopListeningAndDisconnect();
      return 1;
    }
    printf("Bias 完成\n");
  } else {
    printf("跳过 Bias\n");
  }

  std::ofstream csv;
  if (!options.output_path.empty()) {
    csv.open(options.output_path);
    if (!csv.is_open()) {
      fprintf(stderr, "无法打开 CSV: %s\n", options.output_path.c_str());
      listener.stopListeningAndDisconnect();
      return 1;
    }
    csv << "timestamp_us,t_monotonic_ns,fx,fy,fz,force_norm\n";
  }

  std::signal(SIGINT, signalHandler);
  printf("开始采样...\n");
  if (options.print_every > 0) {
    printf("%12s  %8s  %8s  %8s  %9s\n", "timestamp", "FX(N)", "FY(N)", "FZ(N)", "|F|(N)");
  }

  auto wall_start = std::chrono::steady_clock::now();
  unsigned long first_ts_us = 0;
  unsigned long last_ts_us = 0;
  unsigned long samples = 0;
  unsigned long last_ts = 0;  // 用于去重，每次时间戳变化才计数

  constexpr auto poll_interval = std::chrono::milliseconds(1);

  while (g_running) {
    auto now = std::chrono::steady_clock::now();
    double wall_elapsed_sec = std::chrono::duration<double>(now - wall_start).count();
    if (options.duration_sec > 0.0 && wall_elapsed_sec >= options.duration_sec) {
      break;
    }

    // 获取当前最新传感器时间戳
    unsigned long ts_us =
        static_cast<unsigned long>(sensors[options.sensor_index].getTimestamp_us());

    // 时间戳未变化说明后台尚未收到新帧，短暂休眠后重试
    if (ts_us == last_ts) {
      std::this_thread::sleep_for(poll_interval);
      continue;
    }
    last_ts = ts_us;

    double force[NDIM] = {0};
    sensors[options.sensor_index].getGlobalForce(force);
    auto host_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    double force_norm = std::sqrt(force[X_IND] * force[X_IND] + force[Y_IND] * force[Y_IND] +
                                  force[Z_IND] * force[Z_IND]);

    if (samples == 0) {
      first_ts_us = ts_us;
    }
    last_ts_us = ts_us;
    ++samples;

    if (csv.is_open()) {
      csv << ts_us << ',' << host_ns << ',' << force[X_IND] << ',' << force[Y_IND] << ','
          << force[Z_IND] << ',' << force_norm << '\n';
    }

    if (options.print_every > 0 && samples % static_cast<unsigned long>(options.print_every) == 0) {
      printf("%12lu  %8.3f  %8.3f  %8.3f  %9.3f\n", ts_us, force[X_IND], force[Y_IND], force[Z_IND],
             force_norm);
    }

    std::this_thread::sleep_for(poll_interval);
  }

  if (csv.is_open()) {
    csv.flush();
    csv.close();
  }

  printf("停止监听并断开串口...\n");
  listener.stopListeningAndDisconnect();

  auto wall_end = std::chrono::steady_clock::now();
  double wall_elapsed_sec = std::chrono::duration<double>(wall_end - wall_start).count();
  double sensor_elapsed_sec = 0.0;
  if (samples > 1 && last_ts_us > first_ts_us) {
    sensor_elapsed_sec = static_cast<double>(last_ts_us - first_ts_us) / 1000000.0;
  }
  double wall_rate_hz =
      wall_elapsed_sec > 0.0 ? static_cast<double>(samples) / wall_elapsed_sec : 0.0;
  double sensor_rate_hz =
      sensor_elapsed_sec > 0.0 ? static_cast<double>(samples - 1) / sensor_elapsed_sec : 0.0;

  printf("\nC++ summary:\n");
  printf("  samples: %lu\n", samples);
  printf("  wall elapsed: %.6f s\n", wall_elapsed_sec);
  printf("  sensor elapsed: %.6f s\n", sensor_elapsed_sec);
  printf("  wall rate: %.1f Hz\n", wall_rate_hz);
  printf("  sensor timestamp rate: %.1f Hz\n", sensor_rate_hz);
  printf("退出\n");
  return 0;
}
