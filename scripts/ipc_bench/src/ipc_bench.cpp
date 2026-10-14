#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <string>
#include <thread>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

// [AI] IPC 基准工具: 合成 Image 大载荷, 测 跨进程/CycloneDDS/调优/FastDDS/进程内零拷贝 的
// CPU、RSS、端到端延迟。用法:
//   ipc_bench --mode pub   --topic T --w 1280 --h 800 --bpp 2 --fps 30 --secs 10
//   ipc_bench --mode sub   --topic T --secs 10
//   ipc_bench --mode both  --topic T --w 1280 --h 800 --bpp 2 --fps 30 --secs 10 [--intra 1]
// 输出: 发送/接收帧数、实测速率、延迟 p50/p95/p99/max(ms)、进程 CPU 秒与 RSS(MB)。

struct Cfg {
  std::string mode = "sub", topic = "/ipcbench/image";
  int w = 1280, h = 800, bpp = 2, intra = 0, qos_depth = 10, best_effort = 0;
  double fps = 30.0, secs = 10.0;
};

static Cfg parse(int argc, char** argv) {
  Cfg c;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--mode") c.mode = next();
    else if (a == "--topic") c.topic = next();
    else if (a == "--w") c.w = std::stoi(next());
    else if (a == "--h") c.h = std::stoi(next());
    else if (a == "--bpp") c.bpp = std::stoi(next());
    else if (a == "--fps") c.fps = std::stod(next());
    else if (a == "--secs") c.secs = std::stod(next());
    else if (a == "--intra") c.intra = std::stoi(next());
    else if (a == "--qos") c.qos_depth = std::stoi(next());
    else if (a == "--be") c.best_effort = std::stoi(next());
  }
  return c;
}

static double cpu_seconds() {
  struct rusage ru {};
  getrusage(RUSAGE_SELF, &ru);
  return ru.ru_utime.tv_sec + ru.ru_utime.tv_usec * 1e-6 +
         ru.ru_stime.tv_sec + ru.ru_stime.tv_usec * 1e-6;
}

static long rss_mb() {
  FILE* f = std::fopen("/proc/self/statm", "r");
  if (!f) return -1;
  long size = 0, res = 0;
  if (std::fscanf(f, "%ld %ld", &size, &res) != 2) res = -1;
  std::fclose(f);
  return res < 0 ? -1 : res * (sysconf(_SC_PAGESIZE) / 1024) / 1024;
}

static void report(const char* label, const std::vector<double>& lat_ms,
                   size_t frames, double elapsed, double cpu_s) {
  printf("[%s] frames=%zu rate=%.1fHz cpu=%.2fs (%.0f%% of one core) rss=%ldMB\n",
         label, frames, frames / elapsed, cpu_s, 100.0 * cpu_s / elapsed, rss_mb());
  if (lat_ms.empty()) { fflush(stdout); return; }
  std::vector<double> v = lat_ms;
  std::sort(v.begin(), v.end());
  auto q = [&](double p) { return v[std::min(v.size() - 1, (size_t)(p * v.size()))]; };
  printf("[%s] latency ms: p50=%.2f p95=%.2f p99=%.2f max=%.2f mean=%.2f\n", label,
         q(0.50), q(0.95), q(0.99), v.back(),
         std::accumulate(v.begin(), v.end(), 0.0) / v.size());
  fflush(stdout);
}

int main(int argc, char** argv) {
  Cfg c = parse(argc, argv);
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  if (c.intra) opts.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("ipc_bench_node", opts);

  const size_t payload = static_cast<size_t>(c.w) * c.h * c.bpp;
  rclcpp::QoS qos(c.intra ? 1 : c.qos_depth);
  if (c.best_effort) qos.best_effort();   // 镜像 sensor-data 语义(丢旧帧不重传)
  auto pub = node->create_publisher<sensor_msgs::msg::Image>(c.topic, qos);

  std::vector<double> lat;
  size_t rx = 0;
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
      c.topic, qos, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        ++rx;
        double ms = (node->now() - rclcpp::Time(msg->header.stamp)).seconds() * 1e3;
        lat.push_back(ms);
      });

  const auto t0 = std::chrono::steady_clock::now();
  const double cpu0 = cpu_seconds();
  size_t tx = 0;
  rclcpp::TimerBase::SharedPtr timer;
  if (c.mode == "pub" || c.mode == "both") {
    const auto period = std::chrono::duration<double>(1.0 / c.fps);
    timer = node->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period), [&]() {
          auto m = std::make_unique<sensor_msgs::msg::Image>();
          m->header.stamp = node->now();
          m->header.frame_id = "bench";
          m->height = c.h; m->width = c.w;
          m->encoding = (c.bpp == 2) ? "16UC1" : "rgb8";
          m->is_bigendian = 0;
          m->step = c.w * c.bpp;
          m->data.assign(payload, 0x5A);
          pub->publish(std::move(m));
          ++tx;
        });
  }

  const auto deadline = t0 + std::chrono::duration<double>(c.secs);
  // 事件驱动执行器(避免自旋污染 CPU 统计, 也更接近真实节点)
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  exec.cancel();
  spinner.join();
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const double cpu = cpu_seconds() - cpu0;

  char label[64];
  std::snprintf(label, sizeof(label), "mode=%s payload=%.2fMB qos=%s%d", c.mode.c_str(),
                payload / 1048576.0, c.best_effort ? "BE" : "REL", c.qos_depth);
  if (c.mode == "pub" || c.mode == "both") {
    std::snprintf(label, sizeof(label), "%s tx", c.mode.c_str());
    report(label, {}, tx, elapsed, cpu);
  }
  if (c.mode == "sub" || c.mode == "both") {
    std::snprintf(label, sizeof(label), "%s rx", c.mode.c_str());
    report(label, lat, rx, elapsed, cpu);
  }
  rclcpp::shutdown();
  return 0;
}