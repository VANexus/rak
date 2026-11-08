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
    std::shared_ptr<ob::Device> dev;
    try {
      dev = pipe_.getDevice();
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
    if (color_cands.empty() || depth_cands.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No supported color/depth profile (color=%zu depth=%zu); "
                   "pipeline will not start.",
                   color_cands.size(), depth_cands.size());
      return CallbackReturn::FAILURE;
    }

    // 30 fps first, enumeration order otherwise (stable_sort keeps the rest as-is).
    const auto prefer_30 = [](const std::shared_ptr<ob::StreamProfile> &a,
                              const std::shared_ptr<ob::StreamProfile> &b) {
      return profileFps(a) == 30 && profileFps(b) != 30;
    };
    std::stable_sort(color_cands.begin(), color_cands.end(), prefer_30);
    std::stable_sort(depth_cands.begin(), depth_cands.end(), prefer_30);
    color_cands_ = color_cands;
    depth_cands_ = depth_cands;
    return CallbackReturn::SUCCESS;
  }

  // activate 才碰相机：configure 只枚举能力与建发布器，开流是 I/O。
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
    if (rclcpp_lifecycle::LifecycleNode::on_activate(state) != CallbackReturn::SUCCESS) {
      return CallbackReturn::FAILURE;
    }
    if (color_cands_.empty() || depth_cands_.empty()) {
      RCLCPP_ERROR(get_logger(), "activate before configure: no stream profiles cached");
      return CallbackReturn::FAILURE;
    }
    const auto color_cands = color_cands_;
    const auto depth_cands = depth_cands_;

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

    // ob::Error deliberately does NOT derive from std::exception (see
    // libobsensor/hpp/Error.hpp: standalone class, getMessage() not what()), so a
    // std::exception-only catch lets it escape, hits std::terminate and kills the
    // node before any retry can run. Catch it first, then the rest.
    const auto fail = [this](const std::string &why) {
      RCLCPP_WARN(this->get_logger(), "pipeline start attempt failed: %s", why.c_str());
      try { pipe_.stop(); } catch (...) {}
    };

    // Try every colour candidate against every depth candidate until one starts.
    // Each attempt builds a fresh ob::Config, and a failed start is torn down
    // before the next try so the pipeline is not left half-open.
    std::shared_ptr<ob::StreamProfile> used_color, used_depth;
    for (const auto &depth_p : depth_cands) {
      for (const auto &color_p : color_cands) {
        auto config = std::make_shared<ob::Config>();
        config->enableStream(depth_p);
        config->enableStream(color_p);
        try {
          pipe_.start(config, frameset_cb);
          used_color = color_p;
          used_depth = depth_p;
          break;
        } catch (const ob::Error &error) {
          fail(error.getMessage() ? error.getMessage() : "ob::Error (no message)");
        } catch (const std::exception &error) {
          fail(error.what());
        } catch (...) {
          fail("unknown exception");
        }
      }
      if (used_depth) break;
    }

    // Depth is what feeds /camera/depth/points; degrade to depth-only rather than
    // bringing down the whole sensor view if the camera refuses every colour mode.
    if (!used_depth) {
      auto config = std::make_shared<ob::Config>();
      config->enableStream(depth_cands.front());
      try {
        pipe_.start(config, frameset_cb);
        used_depth = depth_cands.front();
        RCLCPP_WARN(this->get_logger(), "Astra started DEPTH-ONLY (no colour mode would start)");
      } catch (const ob::Error &error) {
        RCLCPP_ERROR(get_logger(), "Astra pipeline failed to start: %s",
                     error.getMessage() ? error.getMessage() : "ob::Error");
        return CallbackReturn::FAILURE;
      } catch (const std::exception &error) {
        RCLCPP_ERROR(get_logger(), "Astra pipeline failed to start: %s", error.what());
        return CallbackReturn::FAILURE;
      }
    }

    if (used_color) {
      RCLCPP_INFO(this->get_logger(),
                  "Astra S started: color fmt=%d %ux%u@%ufps, depth fmt=%d %ux%u@%ufps",
                  used_color->format(), profileWidth(used_color), profileHeight(used_color),
                  profileFps(used_color), used_depth->format(), profileWidth(used_depth),
                  profileHeight(used_depth), profileFps(used_depth));
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
    try { pipe_.stop(); } catch (...) {}
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    try { pipe_.stop(); } catch (...) {}
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
    try { pipe_.stop(); } catch (...) {}
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
    }

    try {
      const auto param = pipe_.getCameraParam();
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
        auto table = pipe_.getDevice()->getCalibrationCameraParamList();
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
  ob::Pipeline pipe_;
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
