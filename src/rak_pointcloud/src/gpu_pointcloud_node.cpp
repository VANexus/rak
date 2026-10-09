// Publishes a point cloud built on the GPU from a depth image + camera_info.
//
// Drop-in replacement for the Orbbec SDK's CPU cloud: same topic shape, same
// optical frame, same "no return -> NaN" convention. Set `enable_point_cloud:
// false` on the camera when this node owns the cloud.
//
// Memory is allocated once for the largest image seen and reused; the node does
// no per-frame host allocation, so steady state is copy + kernel + message.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <cuda_runtime.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "rak_gpu/cuda.hpp"
#include "rak_pointcloud/depth_to_xyz.hpp"

namespace rak::pointcloud
{

class GpuPointCloudNode : public rclcpp::Node
{
public:
  GpuPointCloudNode() : rclcpp::Node("gpu_pointcloud")
  {
    depth_topic_ = declare_parameter("depth_topic", std::string("/camera/depth/image_raw"));
    info_topic_ = declare_parameter("camera_info_topic", std::string("/camera/depth/camera_info"));
    points_topic_ = declare_parameter("points_topic", std::string("/camera/depth/points_gpu"));
    // 16-bit depth in millimetres is what the Orbbec drivers emit; override for others.
    depth_scale_ = declare_parameter("depth_scale", 0.001);
    // 1 = every pixel, 2 = every other pixel on each axis, ...
    decimation_ = std::max(1, static_cast<int>(declare_parameter("decimation", 1)));
    min_range_m_ = declare_parameter("min_range_m", 0.1);
    max_range_m_ = declare_parameter("max_range_m", 10.0);
    log_every_n_ = declare_parameter("log_every_n", 0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic_, qos);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic_, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {info_ = msg;});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_depth(msg);});

    rak::gpu::init_cuda();
    RCLCPP_INFO(
      get_logger(), "gpu point cloud: %s + %s -> %s (decimation %d)",
      depth_topic_.c_str(), info_topic_.c_str(), points_topic_.c_str(), decimation_);
  }

private:
  void on_depth(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!info_) {
      RCLCPP_WARN_ONCE(get_logger(), "no camera_info on %s yet; dropping frames", info_topic_.c_str());
      return;
    }
    if (msg->encoding != "16UC1" && msg->encoding != "mono16") {
      RCLCPP_ERROR_ONCE(get_logger(), "unsupported depth encoding %s", msg->encoding.c_str());
      return;
    }
    const int src_w = static_cast<int>(msg->width);
    const int src_h = static_cast<int>(msg->height);
    if (src_w <= 0 || src_h <= 0) {
      return;
    }
    // `step` is bytes per row and may be padded; only the tight case is handled, which is
    // what every driver here emits. Checked rather than assumed so a padded image is loud.
    if (msg->step != static_cast<std::uint32_t>(src_w) * 2u) {
      RCLCPP_ERROR_ONCE(
        get_logger(), "row stride %u is not 2*width (%d); refusing to misread the image",
        msg->step, src_w * 2);
      return;
    }

    const int out_w = (src_w + decimation_ - 1) / decimation_;
    const int out_h = (src_h + decimation_ - 1) / decimation_;
    ensure_buffers(src_w, src_h, out_w, out_h);

    const auto & k = info_->k;
    const auto t0 = std::chrono::steady_clock::now();

    depth_dev_.upload(reinterpret_cast<const std::uint16_t *>(msg->data.data()),
                      static_cast<std::size_t>(src_w) * src_h, stream_.get());

    // One message object, reused. `PointCloud2Modifier::resize()` value-initialises the
    // data vector, so rebuilding it per frame would memset the whole cloud (16 MB at full
    // resolution) on top of the copy that actually carries the values.
    sensor_msgs::msg::PointCloud2 & cloud = cloud_;
    if (cloud.point_step == 0 ||
      cloud.data.size() != static_cast<std::size_t>(out_w) * out_h * cloud.point_step)
    {
      cloud.fields.clear();
      cloud.point_step = 0;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(static_cast<std::size_t>(out_w) * out_h);
    }
    cloud.header = msg->header;
    cloud.is_bigendian = false;
    cloud.is_dense = false;  // invalid returns stay in place as NaN
    // `resize()` sets width to the point count and height to 1; put the real grid back.
    cloud.width = static_cast<std::uint32_t>(out_w);
    cloud.height = static_cast<std::uint32_t>(out_h);

    launch_depth_to_xyz(
      depth_dev_.data(), xyz_dev_.data(),
      src_w, src_h, decimation_, out_w, out_h,
      static_cast<float>(k[0]), static_cast<float>(k[4]),
      static_cast<float>(k[2]), static_cast<float>(k[5]),
      static_cast<float>(depth_scale_), static_cast<float>(min_range_m_),
      static_cast<float>(max_range_m_), stream_.get());

    // The kernel already emits the message's point stride (x, y, z, pad), so this is a
    // straight device->host copy into the message's own buffer -- no second host pass.
    xyz_dev_.download(reinterpret_cast<float *>(cloud.data.data()),
                      static_cast<std::size_t>(out_w) * out_h * 4, stream_.get());
    stream_.synchronize();

    points_pub_->publish(cloud);

    if (log_every_n_ > 0 && ++frame_count_ % log_every_n_ == 0) {
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(
        get_logger(), "%dx%d -> %dx%d (%zu pts) in %.1f ms",
        src_w, src_h, out_w, out_h, static_cast<std::size_t>(out_w) * out_h, ms);
    }
  }

  void ensure_buffers(int src_w, int src_h, int out_w, int out_h)
  {
    const std::size_t src_count = static_cast<std::size_t>(src_w) * src_h;
    const std::size_t xyz_count = static_cast<std::size_t>(out_w) * out_h * 4;
    if (src_count > src_capacity_) {
      depth_dev_.allocate(src_count);
      src_capacity_ = src_count;
    }
    if (xyz_count > xyz_capacity_) {
      xyz_dev_.allocate(xyz_count);
      xyz_capacity_ = xyz_count;
    }
  }

  std::string depth_topic_, info_topic_, points_topic_;
  double depth_scale_{0.001};
  int decimation_{1};
  double min_range_m_{0.1}, max_range_m_{10.0};
  int log_every_n_{0};
  std::size_t frame_count_{0};

  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

  rak::gpu::CudaStream stream_;
  sensor_msgs::msg::PointCloud2 cloud_;
  rak::gpu::DeviceBuffer<std::uint16_t> depth_dev_;
  rak::gpu::DeviceBuffer<float> xyz_dev_;
  std::size_t src_capacity_{0}, xyz_capacity_{0};
};

}  // namespace rak::pointcloud

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rak::pointcloud::GpuPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
