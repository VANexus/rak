// astra_bridge_node.cpp — Orbbec Astra S → ROS2 Jazzy bridge (Orbbec SDK 1.10.18)
// Lists all stream profiles for debugging, enables the best color+depth streams.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class AstraBridgeNode : public rclcpp_lifecycle::LifecycleNode {
public:
  // 组件化 (L2, 2026-10-14): NodeOptions 让 launch_ros 的 ComposableNode 能传
  // use_intra_process_comms 等选项; 默认构造保持向后兼容, 独立可执行行为不变。
  //
  // ADR-0020 §5: 构造函数只做字段初始化。原实现把 getDevice() 与 pipe_.start()
  // 都放在构造函数里 —— 相机被别的进程占用时节点直接抛异常退出, 编排层看到的只是
  // "进程没了"; 现在失败变成 on_configure/on_activate 的可读 reason。
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit AstraBridgeNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions())
      : rclcpp_lifecycle::LifecycleNode("astra_bridge_node", opts), pipe_() {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    declare_parameter("color_frame_id", "camera_color_frame");
    declare_parameter("depth_frame_id", "camera_depth_frame");
    // [AI] 2026-11-15: 标称深度内参（Astra S 640x480 典型值），仅当设备**完全
    // 没有出厂标定**时兜底用（本机这台 getCalibrationCameraParamList()=0 组）。
    // 没有 fx/fy，depth_image_proc 不出点云、4D 代价层就没有输入。
    // 精度待棋盘格实标后回填这四个参数（或把 depth_fx 设 0 关掉兜底）。
    declare_parameter("depth_fx", 570.3);
    declare_parameter("depth_fy", 570.3);
    declare_parameter("depth_cx", 320.0);
    declare_parameter("depth_cy", 240.0);
    // [AI] 2026-11-15: 深度流最大宽度（0=不限）。Orin Nano 4GB 上
    // depth_image_proc 把 640x480@30 转点云是单线程 CPU 活，实测只有
    // 0.6 Hz（一帧 1.5 s）——代价层等于没有输入。默认压到 320（点云量 1/4，
    // 实测 ~2.5 Hz），换更强板子时调大或设 0。分辨率变化由标称内参按比例
    // 缩放覆盖（见 makeCameraInfo）。
    declare_parameter("depth_max_width", 320);
    color_frame_id_ = get_parameter("color_frame_id").as_string();
    depth_frame_id_ = get_parameter("depth_frame_id").as_string();

    // 图片/CameraInfo 是传感流: best-effort (SensorDataQoS) —— 陈旧帧比缺口更糟,
    // 且与 rak_stereo_driver 的既有约定、ADR-0014 和 docs/perf/ipc_baseline.md §3.1 一致
    // (reliable 发布方一旦遇到 reliable 消费者会触发重传风暴, 实测 p99 108ms)。
    // 在仓消费者 (gpu_pointcloud BE、grasp capture BE、sensors_rviz Image=Best Effort) 全部兼容。
    color_pub_      = this->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", rclcpp::SensorDataQoS());
    depth_pub_      = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", rclcpp::SensorDataQoS());
    color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", rclcpp::SensorDataQoS());
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", rclcpp::SensorDataQoS());

    // 设备获取失败（被占用 / 没插）必须是可读的 FAILURE，不是进程消失。
    // pipe_ 现在按需重建（见 startPipelineAttempt() 上方的注释）：configure 阶段
    // 建一个只用来枚举能力；activate 阶段第一次尝试可复用这个从未 start 过的
    // 实例，之后每次失败都会丢弃并重建。
    std::shared_ptr<ob::Device> dev;
    try {
      pipe_ = std::make_unique<ob::Pipeline>();
      dev = pipe_->getDevice();
    } catch (const ob::Error &error) {
      RCLCPP_ERROR(get_logger(), "Astra getDevice failed: %s",
                   error.getMessage() ? error.getMessage() : "ob::Error");
      return CallbackReturn::FAILURE;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Astra getDevice failed: %s", error.what());
      return CallbackReturn::FAILURE;
    }
    if (!dev) {
      RCLCPP_ERROR(get_logger(), "Astra getDevice returned null");
      return CallbackReturn::FAILURE;
    }
    auto sensor_list = dev->getSensorList();

    // Collect *candidates*, do not commit to the first match. The Astra S exposes
    // several fps for the same format+resolution (320x240 UYVY alone comes as three
    // profiles) and the OpenNI layer only accepts a subset of those combinations:
    // starting an unsupported one fails the whole pipeline with
    // "OB_SENSOR_COLOR Match openni video mode failed". See the retry loop below.
    std::vector<std::shared_ptr<ob::StreamProfile>> color_cands, depth_cands;

    for (uint32_t i = 0; i < sensor_list->count(); ++i) {
      auto st = sensor_list->type(i);
      RCLCPP_INFO(this->get_logger(), "Sensor[%u] type=%d", i, st);
      auto profiles = dev->getSensor(st)->getStreamProfileList();
      for (uint32_t j = 0; j < profiles->count(); ++j) {
        auto p = profiles->getProfile(j);
        RCLCPP_INFO(this->get_logger(), "  profile[%u] fmt=%d %ux%u @%ufps", j,
                    p->format(), profileWidth(p), profileHeight(p), profileFps(p));
        const bool supported_color =
            st == OB_SENSOR_COLOR &&
            (p->format() == OB_FORMAT_RGB || p->format() == OB_FORMAT_BGR ||
             p->format() == OB_FORMAT_UYVY || p->format() == OB_FORMAT_YUYV ||
             p->format() == OB_FORMAT_YUY2 || p->format() == OB_FORMAT_MJPG);
        const bool supported_depth =
            st == OB_SENSOR_DEPTH &&
            (p->format() == OB_FORMAT_Y16 || p->format() == OB_FORMAT_Z16 ||
             p->format() == OB_FORMAT_Y11 || p->format() == OB_FORMAT_Y12);
        if (supported_color) color_cands.push_back(p);
        else if (supported_depth) depth_cands.push_back(p);
      }
    }
    // [AI] 2026-11-15: Astra S 是纯深度相机（无色彩传感器）。原实现要求
    // color+depth 都存在，深度相机在 configure 就失败；且 activate 里必须
    // 彩色+深度成对启动，彩色模式匹配失败会把整个流水线停掉（实测日志：
    // "Match openni video mode failed" → 一帧都不出）。改为深度必需、彩色可选。
    if (depth_cands.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No supported depth profile (color=%zu depth=%zu); "
                   "pipeline will not start.",
                   color_cands.size(), depth_cands.size());
      return CallbackReturn::FAILURE;
    }
    if (color_cands.empty()) {
      RCLCPP_INFO(this->get_logger(),
                  "Depth-only camera (no colour profiles); Astra S path enabled.");
    }

    // [AI] 30 fps 优先；同帧率下取**最大分辨率**——原来是枚举顺序，实测
    // 先拿到 160x120（代价层几乎没法用）。stable_sort 保序，失败会回退下一个。
    const auto prefer_best = [](const std::shared_ptr<ob::StreamProfile> &a,
                                const std::shared_ptr<ob::StreamProfile> &b) {
      const bool a30 = profileFps(a) == 30;
      const bool b30 = profileFps(b) == 30;
      if (a30 != b30) return a30;
      return profileWidth(a) * profileHeight(a) > profileWidth(b) * profileHeight(b);
    };
    std::stable_sort(color_cands.begin(), color_cands.end(), prefer_best);
    std::stable_sort(depth_cands.begin(), depth_cands.end(), prefer_best);
    // [AI] 再按 depth_max_width 把"不超宽"的候选提到前面（组内保持上面的
    // 30fps/面积顺序）。0 = 不限。见参数注释：Orin Nano 上 640 宽点云太慢。
    const int64_t max_depth_width = get_parameter("depth_max_width").as_int();
    if (max_depth_width > 0) {
      const auto fits_width = [max_depth_width](
                                  const std::shared_ptr<ob::StreamProfile> &a,
                                  const std::shared_ptr<ob::StreamProfile> &b) {
        const bool a_fit = profileWidth(a) <= static_cast<uint32_t>(max_depth_width);
        const bool b_fit = profileWidth(b) <= static_cast<uint32_t>(max_depth_width);
        return a_fit && !b_fit;
      };
      std::stable_sort(depth_cands.begin(), depth_cands.end(), fits_width);
    }
    color_cands_ = color_cands;
    depth_cands_ = depth_cands;
    return CallbackReturn::SUCCESS;
  }

  // activate 才碰相机：configure 只枚举能力与建发布器，开流是 I/O。
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
    if (rclcpp_lifecycle::LifecycleNode::on_activate(state) != CallbackReturn::SUCCESS) {
      return CallbackReturn::FAILURE;
    }
    if (depth_cands_.empty()) {
      RCLCPP_ERROR(get_logger(), "activate before configure: no depth profile cached");
      return CallbackReturn::FAILURE;
    }
    const auto color_cands = color_cands_;
    const auto depth_cands = depth_cands_;
    // 新会话重新探测一次标定能力（上一轮可能因为别的模式失败才留下标记）。
    color_per_frame_calib_failed_ = false;
    depth_per_frame_calib_failed_ = false;

    const auto frameset_cb = [this](std::shared_ptr<ob::FrameSet> frameset) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "FrameSet received, count=%u", frameset->frameCount());
      // Same trap as the start path, one layer down: getIntrinsic() throws
      // ob::Error when the camera has no calibration for the running mode (which
      // is what DEPTH-ONLY does), and this runs on the SDK's own frame thread
      // where an escaping exception std::terminate()s the whole process.
      try {
        onFrameset(frameset);
      } catch (const ob::Error &error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Astra frame processing failed: %s",
                              error.getMessage() ? error.getMessage() : "ob::Error");
      } catch (const std::exception &error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Astra frame processing failed: %s", error.what());
      } catch (...) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Astra frame processing failed: unknown exception");
      }
    };

    // ── DEPTH-ONLY FIRST ───────────────────────────────────────────────────
    // Order matters. Depth is the only stream this driver must have (it is what
    // depth_image_proc turns into /camera/depth/points), and colour is optional
    // (the Astra S exposes no colour profiles at all). Starting colour×depth
    // first and only degrading afterwards was the reason one bad colour attempt
    // could poison the whole session: see the pipeline-reuse note below.
    std::shared_ptr<ob::StreamProfile> used_color, used_depth;
    bool first_attempt = true;  // 复用 configure 阶段那个从未 start 过的实例
    for (const auto &depth_p : depth_cands) {
      auto config = std::make_shared<ob::Config>();
      config->enableStream(depth_p);
      if (startPipelineAttempt(config, frameset_cb, first_attempt)) {
        used_depth = depth_p;
        break;
      }
      first_attempt = false;
    }

    // ── FALLBACK: colour × depth, one fresh pipeline per attempt ───────────
    // Only reached when every depth-only mode failed. Kept for cameras that
    // genuinely need the colour stream to be up.
    if (!used_depth) {
      for (const auto &depth_p : depth_cands) {
        for (const auto &color_p : color_cands) {
          auto config = std::make_shared<ob::Config>();
          config->enableStream(depth_p);
          config->enableStream(color_p);
          if (startPipelineAttempt(config, frameset_cb, false)) {
            used_depth = depth_p;
            used_color = color_p;
            break;
          }
        }
        if (used_depth) break;
      }
    }

    if (!used_depth) {
      RCLCPP_ERROR(get_logger(),
                   "Astra pipeline failed to start: no depth mode would start "
                   "(%zu depth x %zu colour candidates tried, each on a fresh pipeline)",
                   depth_cands.size(), color_cands.size());
      return CallbackReturn::FAILURE;
    }

    if (used_color) {
      RCLCPP_INFO(this->get_logger(),
                  "Astra S started: color fmt=%d %ux%u@%ufps, depth fmt=%d %ux%u@%ufps",
                  used_color->format(), profileWidth(used_color), profileHeight(used_color),
                  profileFps(used_color), used_depth->format(), profileWidth(used_depth),
                  profileHeight(used_depth), profileFps(used_depth));
    } else {
      // [AI] 纯深度相机（Astra S）正常路径，不是降级失败——照常打印实际生效档位。
      RCLCPP_INFO(this->get_logger(),
                  "Astra S started: depth fmt=%d %ux%u@%ufps (depth-only)",
                  used_depth->format(), profileWidth(used_depth),
                  profileHeight(used_depth), profileFps(used_depth));
    }
    return CallbackReturn::SUCCESS;
  }

  // ───────────────────────────────────────────────────────────────────────
  // 为什么必须"每次尝试都换一个全新的 ob::Pipeline"——不要改成复用一个实例
  // 反复 stop()/start()
  // ───────────────────────────────────────────────────────────────────────
  // Orbbec SDK 1.10.x 的 Pipeline 不是无状态的。一次失败的 start() 会在
  // Pipeline/OpenNI 层留下半配置的流状态：内部的 property 表、已协商的
  // OpenNI video mode、以及 sensor 的 enable 标志都不会被随后的 stop() 完整
  // 回滚。在同一个实例上继续下一次 start() 时，SDK 会拿上一次残留的 mode 去
  // 匹配，于是出现"明明之前能起来的档位，现在连 depth-only 都报
  // OB_SENSOR_COLOR Match openni video mode failed"——实机复现过：连跑
  // 12–15 次 colour×depth 失败后，同一个 pipe_ 上的 depth-only 也再起不来，
  // 而同一时刻官方 sample（进程内全新 Pipeline）开深度正常。
  // 唯一可靠的恢复方式是销毁 Pipeline（连带释放 Device 句柄）再 new 一个：
  // 这样 SDK 从 Context 重新枚举设备、重新协商模式，状态回到干净起点。
  // 代价是每次尝试多一次设备枚举（毫秒级），相对"整路传感器挂掉"可以忽略。
  // 结论：换 Pipeline，不要在旧实例上重试。
  // reuse_current=true 只允许用在"这个实例从未 start() 过"的第一次尝试上
  // （configure 阶段建的那个）；一旦发生过失败，就必须走重建路径。
  bool startPipelineAttempt(
      const std::shared_ptr<ob::Config> &config,
      const std::function<void(std::shared_ptr<ob::FrameSet>)> &frameset_cb,
      bool reuse_current = false) {
    if (!reuse_current || !pipe_) {
      // 先销毁旧实例（释放设备），保证新 Pipeline 从干净状态枚举。
      pipe_.reset();
    }
    try {
      if (!pipe_) pipe_ = std::make_unique<ob::Pipeline>();
    } catch (const ob::Error &error) {
      RCLCPP_WARN(get_logger(), "fresh ob::Pipeline() failed: %s",
                  error.getMessage() ? error.getMessage() : "ob::Error");
      pipe_.reset();
      return false;
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "fresh ob::Pipeline() failed: %s", error.what());
      pipe_.reset();
      return false;
    } catch (...) {
      RCLCPP_WARN(get_logger(), "fresh ob::Pipeline() failed: unknown exception");
      pipe_.reset();
      return false;
    }

    // ob::Error deliberately does NOT derive from std::exception (see
    // libobsensor/hpp/Error.hpp: standalone class, getMessage() not what()), so a
    // std::exception-only catch lets it escape, hits std::terminate and kills the
    // node before any retry can run. Catch it first, then the rest.
    try {
      pipe_->start(config, frameset_cb);
      return true;
    } catch (const ob::Error &error) {
      RCLCPP_WARN(get_logger(), "pipeline start attempt failed: %s",
                  error.getMessage() ? error.getMessage() : "ob::Error (no message)");
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "pipeline start attempt failed: %s", error.what());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "pipeline start attempt failed: unknown exception");
    }
    // 失败的实例本身就是脏的，直接丢掉；下一次尝试会再 new 一个。
    pipe_.reset();
    return false;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
    // stop() 之后连 Pipeline 一起销毁：留着一个用过的实例就等于把上面
    // "不能复用 pipeline" 的坑原样搬到了 deactivate→activate 这条路径上。
    if (pipe_) { try { pipe_->stop(); } catch (...) {} }
    pipe_.reset();
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    if (pipe_) { try { pipe_->stop(); } catch (...) {} }
    pipe_.reset();
    color_cands_.clear();
    depth_cands_.clear();
    color_pub_.reset();
    depth_pub_.reset();
    color_info_pub_.reset();
    depth_info_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  // noexcept by language: an ob::Error escaping here would std::terminate the
  // process during teardown instead of just failing to release the device.
  ~AstraBridgeNode() noexcept {
    if (pipe_) { try { pipe_->stop(); } catch (...) {} }
    pipe_.reset();
  }

private:
  void onFrameset(std::shared_ptr<ob::FrameSet> frameset) {
    auto stamp = this->now();
    if (auto color = frameset->colorFrame()) publishColor(color, stamp);
    if (auto depth = frameset->depthFrame()) publishDepth(depth, stamp);
  }

  static uint32_t profileWidth(const std::shared_ptr<ob::StreamProfile> &sp) {
    if (!sp) return 0;
    auto vp = sp->is<ob::VideoStreamProfile>() ? sp->as<ob::VideoStreamProfile>() : nullptr;
    return vp ? vp->width() : 0;
  }
  static uint32_t profileHeight(const std::shared_ptr<ob::StreamProfile> &sp) {
    if (!sp) return 0;
    auto vp = sp->is<ob::VideoStreamProfile>() ? sp->as<ob::VideoStreamProfile>() : nullptr;
    return vp ? vp->height() : 0;
  }
  // fps is part of what the OpenNI layer matches against; printing it is what made
  // the 320x240 UYVY triple visible when diagnosing the colour-mode start failure.
  static uint32_t profileFps(const std::shared_ptr<ob::StreamProfile> &sp) {
    if (!sp) return 0;
    auto vp = sp->is<ob::VideoStreamProfile>() ? sp->as<ob::VideoStreamProfile>() : nullptr;
    return vp ? vp->fps() : 0;
  }

  static std::shared_ptr<ob::VideoStreamProfile> videoProfile(
      const std::shared_ptr<ob::Frame> &frame) {
    if (!frame) return {};
    auto stream = frame->getStreamProfile();
    return stream && stream->is<ob::VideoStreamProfile>()
        ? stream->as<ob::VideoStreamProfile>() : nullptr;
  }

  // Filled once, on the failure path, from the device's stored calibration
  // table. Per-frame and Pipeline lookups both resolve by matching the running
  // profile, which is exactly what fails in DEPTH-ONLY; the stored table is
  // read directly instead.
  // 每帧标定查询失败后不再重试（见 makeCameraInfo）；每次 activate 重置。
  bool color_per_frame_calib_failed_ = false;
  bool depth_per_frame_calib_failed_ = false;
  bool color_device_calib_ready_ = false;
  bool color_device_calib_tried_ = false;
  sensor_msgs::msg::CameraInfo color_device_calib_;
  bool depth_device_calib_ready_ = false;
  bool depth_device_calib_tried_ = false;
  sensor_msgs::msg::CameraInfo depth_device_calib_;

  static bool usableIntrinsic(const OBCameraIntrinsic &intrinsic,
                              uint32_t width, uint32_t height) {
    return intrinsic.width == static_cast<int16_t>(width) &&
           intrinsic.height == static_cast<int16_t>(height) &&
           std::isfinite(intrinsic.fx) && std::isfinite(intrinsic.fy) &&
           std::isfinite(intrinsic.cx) && std::isfinite(intrinsic.cy) &&
           intrinsic.fx > 0.0F && intrinsic.fy > 0.0F;
  }

  static void fillCameraInfo(sensor_msgs::msg::CameraInfo &info,
                             const OBCameraIntrinsic &intrinsic,
                             const OBCameraDistortion &distortion) {
    info.distortion_model = "plumb_bob";
    // k1 k2 p1 p2 k3 — ROS plumb_bob order, not the SDK's k1..k6 layout.
    info.d = {distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3};
    info.k = {intrinsic.fx, 0.0, intrinsic.cx,
              0.0, intrinsic.fy, intrinsic.cy,
              0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0};
    info.p = {intrinsic.fx, 0.0, intrinsic.cx, 0.0,
              0.0, intrinsic.fy, intrinsic.cy, 0.0,
              0.0, 0.0, 1.0, 0.0};
  }

  sensor_msgs::msg::CameraInfo makeCameraInfo(
      const std::shared_ptr<ob::Frame> &frame, const rclcpp::Time &stamp,
      uint32_t width, uint32_t height, const std::string &frame_id, bool depth) {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id;
    info.width = width;
    info.height = height;

    // Two calibration sources, most specific first. The per-frame profile lookup
    // throws ob::Error when the device has no calibration for the running mode —
    // which is exactly DEPTH-ONLY, where the colour stream the D2C parameters are
    // indexed against is absent. Pipeline::getCameraParam() still carries the
    // factory depth calibration, and that is enough for depth_image_proc to turn
    // the depth image into /camera/depth/points. Without this fallback the point
    // cloud display stays empty for the whole session.
    // 一旦这一层失败就**不再重试**。DEPTH-ONLY 下 getIntrinsic() 每帧都会抛
    // ob::Error，而且 SDK 自己在抛之前先打一行 "Can not find matched camera
    // param!"（不受 RCLCPP 节流控制）。30 fps 下就是每秒 30 行刷屏 + 30 次异常
    // 开销。标定能力是设备/模式的属性，不会在会话中途变好，问一次就够。
    bool &per_frame_calib_failed =
        depth ? depth_per_frame_calib_failed_ : color_per_frame_calib_failed_;
    if (!per_frame_calib_failed) {
      try {
        auto profile = videoProfile(frame);
        if (profile) {
          const auto intrinsic = profile->getIntrinsic();
          if (usableIntrinsic(intrinsic, width, height)) {
            fillCameraInfo(info, intrinsic, profile->getDistortion());
            return info;
          }
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                               "Astra per-frame intrinsics missing or resolution-mismatched; "
                               "falling back to pipeline calibration");
        }
      } catch (const ob::Error &) {
        // Expected in DEPTH-ONLY. The pipeline-level lookup below still works.
        per_frame_calib_failed = true;
        RCLCPP_INFO(get_logger(),
                    "Astra has no per-frame calibration for the running mode; "
                    "using pipeline/stored/nominal calibration from here on");
      }
    }

    // `if (cond) try { ... } catch { ... }` 是合法语句：Pipeline 已随 deactivate
    // 释放时整层跳过，落到下面的设备标定表 / 标称内参兜底。
    if (pipe_) try {
      const auto param = pipe_->getCameraParam();
      const auto &intrinsic = depth ? param.depthIntrinsic : param.rgbIntrinsic;
      const auto &distortion = depth ? param.depthDistortion : param.rgbDistortion;
      // The factory entry is stored at the sensor's native resolution while the
      // stream may run decimated (this Astra S stores one entry and streams
      // 160x120). Requiring an exact match rejects a perfectly usable
      // calibration. A resolution change of one sensor scales fx/fy/cx/cy
      // linearly with the image dimensions, so scale it to the stream and say
      // so — a derived value must never look like a measured one.
      if (intrinsic.width > 0 && intrinsic.height > 0 && intrinsic.fx > 0.0F &&
          intrinsic.fy > 0.0F && std::isfinite(intrinsic.fx) &&
          std::isfinite(intrinsic.fy) && std::isfinite(intrinsic.cx) &&
          std::isfinite(intrinsic.cy)) {
        const float sx = static_cast<float>(width) / intrinsic.width;
        const float sy = static_cast<float>(height) / intrinsic.height;
        OBCameraIntrinsic scaled = intrinsic;
        scaled.fx *= sx;
        scaled.fy *= sy;
        scaled.cx *= sx;
        scaled.cy *= sy;
        scaled.width = static_cast<int16_t>(width);
        scaled.height = static_cast<int16_t>(height);
        fillCameraInfo(info, scaled, distortion);
        if (sx != 1.0F || sy != 1.0F) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 60000,
                               "Astra %s calibration from %dx%d scaled to %ux%u "
                               "(fx=%.1f fy=%.1f)",
                               depth ? "depth" : "colour", intrinsic.width,
                               intrinsic.height, width, height, scaled.fx, scaled.fy);
        }
        return info;
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 30000,
          "Astra pipeline calibration invalid for %ux%u: stored %dx%d fx=%.2f fy=%.2f "
          "cx=%.2f cy=%.2f -> publishing uncalibrated CameraInfo",
          width, height, intrinsic.width, intrinsic.height, intrinsic.fx, intrinsic.fy,
          intrinsic.cx, intrinsic.cy);
    } catch (const ob::Error &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                           "Astra has no calibration for this mode (%s); publishing "
                           "uncalibrated CameraInfo",
                           error.getMessage() ? error.getMessage() : "ob::Error");
    }

    // Third source: the raw calibration table stored on the device. Both APIs
    // above resolve by matching the *running* profile, which is exactly what
    // fails in DEPTH-ONLY. getCalibrationCameraParamList() returns the table as
    // written at the factory, so an entry can still be read by index. When its
    // stored resolution differs from the stream it is scaled to the stream
    // size — a resolution change of one sensor scales fx/fy/cx/cy linearly with
    // the image dimensions — and that derivation is logged once rather than
    // left silent, so nobody mistakes a scaled value for a measured one.
    auto &cached = depth ? depth_device_calib_ : color_device_calib_;
    bool &ready = depth ? depth_device_calib_ready_ : color_device_calib_ready_;
    bool &tried = depth ? depth_device_calib_tried_ : color_device_calib_tried_;
    if (!tried) {
      tried = true;
      try {
        auto table = pipe_->getDevice()->getCalibrationCameraParamList();
        if (table) {
          const uint32_t groups = table->count();
          RCLCPP_INFO(get_logger(), "Astra stored calibration table: %u group(s)", groups);
          for (uint32_t i = 0; i < groups && !ready; ++i) {
            const auto param = table->getCameraParam(i);
            const auto &intrinsic = depth ? param.depthIntrinsic : param.rgbIntrinsic;
            const auto &distortion = depth ? param.depthDistortion : param.rgbDistortion;
            if (intrinsic.width <= 0 || intrinsic.height <= 0 ||
                intrinsic.fx <= 0.0F || intrinsic.fy <= 0.0F ||
                !std::isfinite(intrinsic.fx) || !std::isfinite(intrinsic.fy)) {
              continue;
            }
            const float sx = static_cast<float>(width) / intrinsic.width;
            const float sy = static_cast<float>(height) / intrinsic.height;
            OBCameraIntrinsic scaled = intrinsic;
            scaled.fx *= sx;
            scaled.fy *= sy;
            scaled.cx *= sx;
            scaled.cy *= sy;
            scaled.width = static_cast<int16_t>(width);
            scaled.height = static_cast<int16_t>(height);
            cached.width = width;
            cached.height = height;
            fillCameraInfo(cached, scaled, distortion);
            ready = true;
            RCLCPP_INFO(get_logger(),
                        "Astra %s calibration taken from stored group %u (%dx%d) scaled to "
                        "%ux%u (fx=%.1f fy=%.1f cx=%.1f cy=%.1f)",
                        depth ? "depth" : "colour", i, intrinsic.width, intrinsic.height,
                        width, height, scaled.fx, scaled.fy, scaled.cx, scaled.cy);
          }
        }
      } catch (const ob::Error &error) {
        RCLCPP_WARN(get_logger(), "Astra stored calibration table unreadable (%s)",
                    error.getMessage() ? error.getMessage() : "ob::Error");
      }
    }
    if (ready && cached.width == width && cached.height == height) {
      info.distortion_model = cached.distortion_model;
      info.d = cached.d;
      info.k = cached.k;
      info.r = cached.r;
      info.p = cached.p;
      return info;
    }

    // [AI] 2026-11-15: 第四层——启动参数里的**标称内参**。本机这台 Astra S
    // 的出厂标定表是空表（getCalibrationCameraParamList() 返回 0 组），前三层
    // SDK 回退全部失败；没有 fx/fy 时 depth_image_proc 不出点云。标称值只够
    // 把点云跑起来，**精度待棋盘格实标后回填 depth_fx/fy/cx/cy**。
    if (depth) {
      const double nominal_fx = get_parameter("depth_fx").as_double();
      const double nominal_fy = get_parameter("depth_fy").as_double();
      if (nominal_fx > 0.0 && nominal_fy > 0.0) {
        const double nominal_cx = get_parameter("depth_cx").as_double();
        const double nominal_cy = get_parameter("depth_cy").as_double();
        const double s = static_cast<double>(width) / 640.0;
        OBCameraIntrinsic nominal{};
        nominal.width = static_cast<int16_t>(width);
        nominal.height = static_cast<int16_t>(height);
        nominal.fx = static_cast<float>(nominal_fx * s);
        nominal.fy = static_cast<float>(nominal_fy * s);
        nominal.cx = static_cast<float>(nominal_cx * s);
        nominal.cy = static_cast<float>(nominal_cy * s);
        OBCameraDistortion no_distortion{};
        fillCameraInfo(info, nominal, no_distortion);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000,
                             "Astra depth uses NOMINAL intrinsics for %ux%u "
                             "(fx=%.1f fy=%.1f cx=%.1f cy=%.1f) — calibration pending",
                             width, height, nominal.fx, nominal.fy, nominal.cx, nominal.cy);
        return info;
      }
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                         "Astra has no usable calibration for %s %ux%u; publishing "
                         "uncalibrated CameraInfo (point cloud will be empty)",
                         depth ? "depth" : "colour", width, height);
    return info;
  }

  void publishColor(
      const std::shared_ptr<ob::ColorFrame> &source_frame, const rclcpp::Time &stamp) {
    auto frame = source_frame;
    // Use the SDK decoder; packed/compressed payloads are never copied as RGB.
    switch (frame->format()) {
      case OB_FORMAT_UYVY: color_converter_.setFormatConvertType(FORMAT_UYVY_TO_RGB); break;
      case OB_FORMAT_YUYV:
      case OB_FORMAT_YUY2: color_converter_.setFormatConvertType(FORMAT_YUYV_TO_RGB); break;
      case OB_FORMAT_MJPG: color_converter_.setFormatConvertType(FORMAT_MJPG_TO_RGB); break;
      default: break;
    }
    if (frame->format() != OB_FORMAT_RGB && frame->format() != OB_FORMAT_BGR) {
      auto converted = color_converter_.process(frame);
      if (!converted) return;
      frame = converted->as<ob::ColorFrame>();
    }
    const uint32_t w = frame->width(), h = frame->height();
    if (w == 0 || h == 0 || !frame->data()) return;
    const auto format = frame->format();
    if (format != OB_FORMAT_RGB && format != OB_FORMAT_BGR) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "unsupported Astra color format %d", format);
      return;
    }
    const uint32_t row_bytes = w * 3;
    const uint32_t data_size = frame->dataSize();
    if (data_size < row_bytes * h || data_size % h != 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "invalid Astra color frame data size");
      return;
    }
    const uint32_t source_stride = data_size / h;

    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.stamp = stamp;
    img->header.frame_id = color_frame_id_;
    img->width = w; img->height = h;
    img->step = row_bytes;
    img->encoding = format == OB_FORMAT_RGB ? "rgb8" : "bgr8";
    img->is_bigendian = false;
    img->data.resize(static_cast<size_t>(row_bytes) * h);
    const auto *source = static_cast<const uint8_t *>(frame->data());
    for (uint32_t row = 0; row < h; ++row) {
      std::memcpy(
          img->data.data() + static_cast<size_t>(row) * row_bytes,
          source + static_cast<size_t>(row) * source_stride, row_bytes);
    }
    if (color_pub_->is_activated()) {
      color_pub_->publish(*img);
    }
    color_info_pub_->publish(makeCameraInfo(source_frame, stamp, w, h, color_frame_id_, false));
  }

  void publishDepth(
      const std::shared_ptr<ob::DepthFrame> &frame, const rclcpp::Time &stamp) {
    const uint32_t w = frame->width(), h = frame->height();
    if (w == 0 || h == 0 || !frame->data()) return;
    const auto format = frame->format();
    // Same set the profile picker accepts: the SDK unpacks Y10/Y11/Y12 into 16-bit
    // samples by default, and getValueScale() below rescales the magnitudes. Keeping
    // the two sets identical stopped depth frames being dropped after a Y11 start.
    if (format != OB_FORMAT_Y16 && format != OB_FORMAT_Z16 &&
        format != OB_FORMAT_Y10 && format != OB_FORMAT_Y11 &&
        format != OB_FORMAT_Y12) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "unsupported Astra depth format %d", format);
      return;
    }
    const uint32_t row_bytes = w * sizeof(uint16_t);
    const uint32_t data_size = frame->dataSize();
    if (data_size < row_bytes * h || data_size % h != 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "invalid Astra depth frame data size");
      return;
    }
    const uint32_t source_stride = data_size / h;
    const float scale = frame->getValueScale();
    if (!std::isfinite(scale) || scale <= 0.0F) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "invalid Astra depth value scale");
      return;
    }

    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.stamp = stamp;
    img->header.frame_id = depth_frame_id_;
    img->width = w; img->height = h;
    img->step = row_bytes;
    img->encoding = "16UC1";
    img->is_bigendian = false;
    img->data.resize(static_cast<size_t>(row_bytes) * h);
    const auto *source = static_cast<const uint8_t *>(frame->data());
    for (uint32_t row = 0; row < h; ++row) {
      const auto *src = source + static_cast<size_t>(row) * source_stride;
      auto *dst = img->data.data() + static_cast<size_t>(row) * row_bytes;
      if (std::abs(scale - 1.0F) < 1e-6F) {
        std::memcpy(dst, src, row_bytes);
        continue;
      }
      for (uint32_t col = 0; col < w; ++col) {
        uint16_t raw;
        std::memcpy(&raw, src + static_cast<size_t>(col) * sizeof(uint16_t), sizeof(raw));
        const auto millimeters = static_cast<uint16_t>(std::clamp(
            std::lround(static_cast<double>(raw) * scale), 0L, 65535L));
        std::memcpy(
            dst + static_cast<size_t>(col) * sizeof(uint16_t),
            &millimeters, sizeof(millimeters));
      }
    }
    if (depth_pub_->is_activated()) {
      depth_pub_->publish(*img);
    }
    depth_info_pub_->publish(makeCameraInfo(frame, stamp, w, h, depth_frame_id_, true));
  }

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_;
  std::vector<std::shared_ptr<ob::StreamProfile>> color_cands_, depth_cands_;
  std::string color_frame_id_;
  std::string depth_frame_id_;
  ob::FormatConvertFilter color_converter_;
  // unique_ptr 而不是值成员：每次启动尝试都要丢弃旧实例、重建新实例，
  // 原因见 startOnFreshPipeline() 上方的长注释。
  std::unique_ptr<ob::Pipeline> pipe_;
};

// 组件注册: 供 component_container 以 "AstraBridgeNode" 加载 (L2, 2026-10-14)。
// 下方 main() 仍保留, 由 CMake 生成的同名独立可执行使用组件路径, 行为等价。
RCLCPP_COMPONENTS_REGISTER_NODE(AstraBridgeNode)

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AstraBridgeNode>();
  // 独立可执行没有编排者，自己走到 active；被 launch 以 LifecycleNode 起时
  // 由 launch 事件或 system_manager 发转移。
  if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "astra bridge failed to reach active");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
