// astra_bridge_node.cpp — Orbbec Astra S → ROS2 Jazzy bridge (Orbbec SDK 1.10.18)
// Lists all stream profiles for debugging, enables the best color+depth streams.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <libobsensor/ObSensor.hpp>

#include <memory>
#include <vector>

class AstraBridgeNode : public rclcpp::Node {
public:
  AstraBridgeNode() : Node("astra_bridge_node"), pipe_() {
    color_pub_      = this->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
    depth_pub_      = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
    color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", 10);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTf();

    auto dev = pipe_.getDevice();
    auto sensor_list = dev->getSensorList();
    auto config = std::make_shared<ob::Config>();

    for (uint32_t i = 0; i < sensor_list->count(); ++i) {
      auto st = sensor_list->type(i);
      RCLCPP_INFO(this->get_logger(), "Sensor[%u] type=%d", i, st);
      auto profiles = dev->getSensor(st)->getStreamProfileList();
      std::shared_ptr<ob::StreamProfile> chosen;
      for (uint32_t j = 0; j < profiles->count(); ++j) {
        auto p = profiles->getProfile(j);
        RCLCPP_INFO(this->get_logger(), "  profile[%u] fmt=%d %ux%u", j, p->format(), p->width(), p->height());
        if (!chosen) {
          // prefer MJPG for color, Y16 for depth; otherwise pick first available
          if ((st == OB_SENSOR_COLOR && (p->format() == OB_FORMAT_MJPG || p->format() == OB_FORMAT_YUYV || p->format() == OB_FORMAT_RGB)) ||
              (st == OB_SENSOR_DEPTH && (p->format() == OB_FORMAT_Y16 || p->format() == OB_FORMAT_Y12))) {
            chosen = p;
          }
        }
      }
      if (chosen) {
        RCLCPP_INFO(this->get_logger(), "  -> enabling fmt=%d %ux%u", chosen->format(), chosen->width(), chosen->height());
        config->enableStream(chosen);
      }
    }
    pipe_.start(config, [this](std::shared_ptr<ob::FrameSet> frameset) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FrameSet received, count=%lu", frameset->frameCount());
      onFrameset(frameset);
    });
    RCLCPP_INFO(this->get_logger(), "Astra S pipeline started");
  }

  ~AstraBridgeNode() { pipe_.stop(); }

private:
  void publishStaticTf() {
    auto stamp = this->now();
    std::vector<geometry_msgs::msg::TransformStamped> tfs(3);

    tfs[0].header.stamp = stamp;
    tfs[0].header.frame_id = "base_link";
    tfs[0].child_frame_id = "camera_link";
    tfs[0].transform.translation.x = 0.10;
    tfs[0].transform.translation.y = 0.0;
    tfs[0].transform.translation.z = 0.15;
    tfs[0].transform.rotation.w = 1;

    tfs[1].header.stamp = stamp;
    tfs[1].header.frame_id = "camera_link";
    tfs[1].child_frame_id = "camera_color_frame";
    tfs[1].transform.rotation.w = 1;

    tfs[2].header.stamp = stamp;
    tfs[2].header.frame_id = "camera_link";
    tfs[2].child_frame_id = "camera_depth_frame";
    tfs[2].transform.translation.x = 0.0;
    tfs[2].transform.translation.y = 0.025;
    tfs[2].transform.translation.z = 0.0;
    tfs[2].transform.rotation.w = 1;

    tf_static_broadcaster_->sendTransform(tfs);
  }

  void onFrameset(std::shared_ptr<ob::FrameSet> frameset) {
    auto count = frameset->frameCount();
    auto stamp = this->now();
    for (uint32_t i = 0; i < count; ++i) {
      auto frame = frameset->getFrame(i);
      auto fmt = frame->format();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "frame[%u] type=%d fmt=%d", i, frame->type(), fmt);
      if (fmt == OB_FORMAT_MJPG || fmt == OB_FORMAT_RGB || fmt == OB_FORMAT_BGR || fmt == OB_FORMAT_YUYV) {
        publishColor(frame, stamp);
      } else if (fmt == OB_FORMAT_Y16 || fmt == OB_FORMAT_Y12 || fmt == OB_FORMAT_Y11) {
        publishDepth(frame, stamp);
      }
    }
  }

  static uint32_t frameWidth(std::shared_ptr<ob::Frame> & frame) {
    auto sp = frame->getStreamProfile();
    if (!sp) return 0;
    auto vp = std::dynamic_pointer_cast<ob::VideoStreamProfile>(sp);
    return vp ? vp->width() : 0;
  }
  static uint32_t frameHeight(std::shared_ptr<ob::Frame> & frame) {
    auto sp = frame->getStreamProfile();
    if (!sp) return 0;
    auto vp = std::dynamic_pointer_cast<ob::VideoStreamProfile>(sp);
    return vp ? vp->height() : 0;
  }
  static OBFormat frameFormat(std::shared_ptr<ob::Frame> & frame) {
    return frame->format();
  }

  void publishColor(std::shared_ptr<ob::Frame> frame, const rclcpp::Time & stamp) {
    uint32_t w = frameWidth(frame), h = frameHeight(frame);
    if (w == 0 || h == 0) return;
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.stamp = stamp;
    img->header.frame_id = "camera_color_frame";
    img->width = w; img->height = h;
    img->step = w * 3;
    img->encoding = "bgr8";
    img->data.resize(img->step * h);
    std::memcpy(img->data.data(), frame->data(), img->data.size());
    color_pub_->publish(*img);
    color_info_pub_->publish(makeColorInfo(stamp, w, h));
  }

  void publishDepth(std::shared_ptr<ob::Frame> frame, const rclcpp::Time & stamp) {
    uint32_t w = frameWidth(frame), h = frameHeight(frame);
    if (w == 0 || h == 0) return;
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.stamp = stamp;
    img->header.frame_id = "camera_depth_frame";
    img->width = w; img->height = h;
    img->step = w * 2;
    img->encoding = "16UC1";
    img->data.resize(img->step * h);
    std::memcpy(img->data.data(), frame->data(), img->data.size());
    depth_pub_->publish(*img);
    depth_info_pub_->publish(makeDepthInfo(stamp, w, h));
  }

  sensor_msgs::msg::CameraInfo makeColorInfo(const rclcpp::Time & stamp, uint32_t w, uint32_t h) {
    sensor_msgs::msg::CameraInfo ci;
    ci.header.stamp = stamp;
    ci.header.frame_id = "camera_color_frame";
    ci.width = w; ci.height = h;
    ci.distortion_model = "plumb_bob";
    ci.k = {525.0, 0, 320.0,  0, 525.0, 240.0,  0, 0, 1};
    return ci;
  }
  sensor_msgs::msg::CameraInfo makeDepthInfo(const rclcpp::Time & stamp, uint32_t w, uint32_t h) {
    sensor_msgs::msg::CameraInfo ci;
    ci.header.stamp = stamp;
    ci.header.frame_id = "camera_depth_frame";
    ci.width = w; ci.height = h;
    ci.distortion_model = "plumb_bob";
    ci.k = {570.0, 0, 320.0,  0, 570.0, 240.0,  0, 0, 1};
    return ci;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  ob::Pipeline pipe_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AstraBridgeNode>());
  rclcpp::shutdown();
  return 0;
}