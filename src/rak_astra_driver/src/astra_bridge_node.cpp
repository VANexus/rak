// astra_bridge_node.cpp — Orbbec Astra S → ROS2 Jazzy bridge (Orbbec SDK 1.10.18)
// Lists all stream profiles for debugging, enables the best color+depth streams.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

class AstraBridgeNode : public rclcpp::Node {
public:
  AstraBridgeNode() : Node("astra_bridge_node"), pipe_() {
    declare_parameter("color_frame_id", "camera_color_frame");
    declare_parameter("depth_frame_id", "camera_depth_frame");
    color_frame_id_ = get_parameter("color_frame_id").as_string();
    depth_frame_id_ = get_parameter("depth_frame_id").as_string();

    color_pub_      = this->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
    depth_pub_      = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
    color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", 10);

    auto dev = pipe_.getDevice();
    auto sensor_list = dev->getSensorList();
    auto config = std::make_shared<ob::Config>();
    bool color_enabled = false, depth_enabled = false;

    for (uint32_t i = 0; i < sensor_list->count(); ++i) {
      auto st = sensor_list->type(i);
      RCLCPP_INFO(this->get_logger(), "Sensor[%u] type=%d", i, st);
      auto profiles = dev->getSensor(st)->getStreamProfileList();
      std::shared_ptr<ob::StreamProfile> chosen;
      for (uint32_t j = 0; j < profiles->count(); ++j) {
        auto p = profiles->getProfile(j);
        RCLCPP_INFO(this->get_logger(), "  profile[%u] fmt=%d %ux%u", j, p->format(), profileWidth(p), profileHeight(p));
        const bool supported_color =
            st == OB_SENSOR_COLOR &&
            (p->format() == OB_FORMAT_RGB || p->format() == OB_FORMAT_BGR ||
             p->format() == OB_FORMAT_UYVY || p->format() == OB_FORMAT_YUYV ||
             p->format() == OB_FORMAT_YUY2 || p->format() == OB_FORMAT_MJPG);
        const bool supported_depth =
            st == OB_SENSOR_DEPTH &&
            (p->format() == OB_FORMAT_Y16 || p->format() == OB_FORMAT_Z16 ||
             p->format() == OB_FORMAT_Y11 || p->format() == OB_FORMAT_Y12);
        if (!chosen && (supported_color || supported_depth)) {
          chosen = p;
        }
      }
      if (chosen) {
        RCLCPP_INFO(this->get_logger(), "  -> enabling fmt=%d %ux%u", chosen->format(), profileWidth(chosen), profileHeight(chosen));
        config->enableStream(chosen);
        color_enabled = color_enabled || st == OB_SENSOR_COLOR;
        depth_enabled = depth_enabled || st == OB_SENSOR_DEPTH;
      } else if (st == OB_SENSOR_COLOR || st == OB_SENSOR_DEPTH) {
        RCLCPP_ERROR(
            this->get_logger(),
            "No supported color/depth profile; pipeline will not start.");
      }
    }
    if (!color_enabled || !depth_enabled) {
      throw std::runtime_error("Astra requires supported color and depth profiles");
    }
    pipe_.start(config, [this](std::shared_ptr<ob::FrameSet> frameset) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FrameSet received, count=%u", frameset->frameCount());
      try {
        onFrameset(frameset);
      } catch (const std::exception &error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Astra frame processing failed: %s", error.what());
      }
    });
    RCLCPP_INFO(this->get_logger(), "Astra S pipeline started");
  }

  ~AstraBridgeNode() { pipe_.stop(); }

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

  static std::shared_ptr<ob::VideoStreamProfile> videoProfile(
      const std::shared_ptr<ob::Frame> &frame) {
    if (!frame) return {};
    auto stream = frame->getStreamProfile();
    return stream && stream->is<ob::VideoStreamProfile>()
        ? stream->as<ob::VideoStreamProfile>() : nullptr;
  }

  sensor_msgs::msg::CameraInfo makeCameraInfo(
      const std::shared_ptr<ob::Frame> &frame, const rclcpp::Time &stamp,
      uint32_t width, uint32_t height, const std::string &frame_id) {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id;
    info.width = width;
    info.height = height;
    auto profile = videoProfile(frame);
    if (!profile) return info;

    const auto intrinsic = profile->getIntrinsic();
    if (intrinsic.width != static_cast<int>(width) ||
        intrinsic.height != static_cast<int>(height) ||
        !std::isfinite(intrinsic.fx) || !std::isfinite(intrinsic.fy) ||
        !std::isfinite(intrinsic.cx) || !std::isfinite(intrinsic.cy) ||
        intrinsic.fx <= 0.0F || intrinsic.fy <= 0.0F) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Astra SDK returned invalid or resolution-mismatched intrinsics; publishing "
          "uncalibrated CameraInfo");
      return info;
    }

    const auto distortion = profile->getDistortion();
    info.distortion_model = "plumb_bob";
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
    color_pub_->publish(*img);
    color_info_pub_->publish(makeCameraInfo(source_frame, stamp, w, h, color_frame_id_));
  }

  void publishDepth(
      const std::shared_ptr<ob::DepthFrame> &frame, const rclcpp::Time &stamp) {
    const uint32_t w = frame->width(), h = frame->height();
    if (w == 0 || h == 0 || !frame->data()) return;
    const auto format = frame->format();
    if (format != OB_FORMAT_Y16 && format != OB_FORMAT_Z16) {
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
    depth_pub_->publish(*img);
    depth_info_pub_->publish(makeCameraInfo(frame, stamp, w, h, depth_frame_id_));
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_;
  std::string color_frame_id_;
  std::string depth_frame_id_;
  ob::FormatConvertFilter color_converter_;
  ob::Pipeline pipe_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AstraBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
