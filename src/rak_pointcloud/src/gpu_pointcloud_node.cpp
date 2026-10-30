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
#include <rclcpp_components/register_node_macro.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "rak_gpu/cuda.hpp"
#include "rak_pointcloud/depth_to_xyz.hpp"

namespace rak::pointcloud
{

class GpuPointCloudNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  // [AI] 组件化: 接受 NodeOptions(launch_ros 的 ComposableNode 用它传 use_intra_process_comms
  // 等选项); 默认构造保持向后兼容, 单独可执行的行为不变。
  //
  // ADR-0020 §5：构造函数**不碰 CUDA**。原实现先建订阅、再 init_cuda()，于是第一帧
  // depth 可能在 CUDA 上下文就绪之前进回调 —— 这是已经存在的初始化顺序缺陷，不是
  // 风格问题。现在顺序由生命周期固定：configure 里 init_cuda() 成功后**才**建订阅。
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit GpuPointCloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("gpu_pointcloud", options)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
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

    // 资源先于 I/O：CUDA 上下文失败时返回 FAILURE（编排层看到可读原因），
    // 而不是让第一帧在回调里崩。
    try {
      rak::gpu::init_cuda();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "init_cuda failed: %s", error.what());
      return CallbackReturn::FAILURE;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic_, qos);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic_, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {info_ = msg;});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_depth(msg);});

    RCLCPP_INFO(
      get_logger(), "gpu point cloud: %s + %s -> %s (decimation %d)",
      depth_topic_.c_str(), info_topic_.c_str(), points_topic_.c_str(), decimation_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    // 必须转发生命周期基类实现，否则 LifecyclePublisher 永远不启用，
    // publish() 静默丢弃（本仓在 Python 侧踩过同款坑）。
    if (rclcpp_lifecycle::LifecycleNode::on_activate(state) != CallbackReturn::SUCCESS) {
      return CallbackReturn::FAILURE;
    }
    frame_count_ = 0;
    info_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    depth_sub_.reset();
    info_sub_.reset();
    points_pub_.reset();
    depth_dev_.free();
    xyz_dev_.free();
    src_capacity_ = 0;
    xyz_capacity_ = 0;
    return CallbackReturn::SUCCESS;
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

    if (points_pub_ && points_pub_->is_activated()) {
      points_pub_->publish(cloud);
    }

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
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

  rak::gpu::CudaStream stream_;
  sensor_msgs::msg::PointCloud2 cloud_;
  rak::gpu::DeviceBuffer<std::uint16_t> depth_dev_;
  rak::gpu::DeviceBuffer<float> xyz_dev_;
  std::size_t src_capacity_{0}, xyz_capacity_{0};
};

}  // namespace rak::pointcloud

// [AI] 组件注册: 供 component_container 以 "rak::pointcloud::GpuPointCloudNode" 加载。
// 文件内的 main() 仍保留(见下), 由 CMake 生成的同名可执行使用组件路径, 行为等价。
RCLCPP_COMPONENTS_REGISTER_NODE(rak::pointcloud::GpuPointCloudNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rak::pointcloud::GpuPointCloudNode>();
  // 独立可执行 = 没有编排者（supervised:=false 的等价物）：自己走到 active。
  // 被 launch 以 LifecycleNode 起时，configure/activate 由 launch 或 system_manager 发。
  if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "point cloud node failed to reach active");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
