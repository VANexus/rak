#include <eye/capture_node.hpp>

#include <core/cuda.hpp>

#include <nvjpeg.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace rak::eye {

namespace {

int xioctl(int fd, unsigned long request, void *arg) {
  int r;
  do { r = ioctl(fd, request, arg); } while (r == -1 && errno == EINTR);
  return r;
}

}  // namespace

// ---------- NvJpegDecoder (pimpl) ----------

namespace {
std::string nvjpeg_status_text(nvjpegStatus_t s) {
  switch (s) {
    case NVJPEG_STATUS_SUCCESS: return "SUCCESS";
    case NVJPEG_STATUS_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case NVJPEG_STATUS_INVALID_PARAMETER: return "INVALID_PARAMETER";
    case NVJPEG_STATUS_BAD_JPEG: return "BAD_JPEG";
    case NVJPEG_STATUS_JPEG_NOT_SUPPORTED: return "JPEG_NOT_SUPPORTED";
    case NVJPEG_STATUS_ALLOCATOR_FAILURE: return "ALLOCATOR_FAILURE";
    case NVJPEG_STATUS_EXECUTION_FAILED: return "EXECUTION_FAILED";
    case NVJPEG_STATUS_ARCH_MISMATCH: return "ARCH_MISMATCH";
    case NVJPEG_STATUS_INTERNAL_ERROR: return "INTERNAL_ERROR";
    case NVJPEG_STATUS_IMPLEMENTATION_NOT_SUPPORTED: return "IMPLEMENTATION_NOT_SUPPORTED";
    case NVJPEG_STATUS_INCOMPLETE_BITSTREAM: return "INCOMPLETE_BITSTREAM";
  }
  return "UNKNOWN(" + std::to_string((int)s) + ")";
}
void nvjpegCheck(nvjpegStatus_t s, const char *what) {
  if (s != NVJPEG_STATUS_SUCCESS) {
    throw std::runtime_error(std::string("nvJPEG ") + what + " failed: " + nvjpeg_status_text(s));
  }
}
}  // namespace

struct NvJpegDecoder::Impl {
  nvjpegHandle_t handle = nullptr;
  nvjpegJpegState_t state = nullptr;
};

NvJpegDecoder::NvJpegDecoder() : impl_(new Impl) {
  nvjpegCheck(nvjpegCreateSimple(&impl_->handle), "nvjpegCreateSimple");
  nvjpegCheck(nvjpegJpegStateCreate(impl_->handle, &impl_->state), "nvjpegJpegStateCreate");
}
NvJpegDecoder::~NvJpegDecoder() {
  if (impl_) {
    if (impl_->state) nvjpegJpegStateDestroy(impl_->state);
    if (impl_->handle) nvjpegDestroy(impl_->handle);
    delete impl_;
  }
}

size_t NvJpegDecoder::decode_to_gpu(const unsigned char *data, size_t len, int width, int height,
                                    unsigned char *gpu_bgr, cudaStream_t stream) {
  nvjpegImage_t dst{};
  dst.channel[0] = gpu_bgr;
  dst.pitch[0] = (size_t)width * 3;  // tight row pitch for our own buffer

  nvjpegCheck(nvjpegDecode(impl_->handle, impl_->state, data, len,
                           NVJPEG_OUTPUT_BGRI, &dst, stream),
              "nvjpegDecode");
  return dst.pitch[0];
}

// ---------- CaptureNode ----------

CaptureNode::CaptureNode(const rclcpp::NodeOptions &opts)
    : Node("eye_capture", opts), logger_(get_logger()) {
  // Declare parameters (with defaults from Config) and load config_.
  declare_parameter("device", config_.device);
  declare_parameter("sbs_width", config_.sbs_width);
  declare_parameter("sbs_height", config_.sbs_height);
  declare_parameter("fps", config_.fps);
  declare_parameter("frame_id", config_.frame_id);
  declare_parameter("left_topic", config_.left_topic);
  declare_parameter("right_topic", config_.right_topic);
  declare_parameter("left_info_topic", config_.left_info_topic);
  declare_parameter("right_info_topic", config_.right_info_topic);

  get_parameter("device", config_.device);
  get_parameter("sbs_width", config_.sbs_width);
  get_parameter("sbs_height", config_.sbs_height);
  get_parameter("fps", config_.fps);
  get_parameter("frame_id", config_.frame_id);
  get_parameter("left_topic", config_.left_topic);
  get_parameter("right_topic", config_.right_topic);
  get_parameter("left_info_topic", config_.left_info_topic);
  get_parameter("right_info_topic", config_.right_info_topic);

  eye_w_ = config_.sbs_width / 2;
  eye_h_ = config_.sbs_height;

  pub_left_ = create_publisher<sensor_msgs::msg::Image>(config_.left_topic, rclcpp::SensorDataQoS());
  pub_right_ = create_publisher<sensor_msgs::msg::Image>(config_.right_topic, rclcpp::SensorDataQoS());
  pub_left_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.left_info_topic, rclcpp::SensorDataQoS());
  pub_right_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.right_info_topic, rclcpp::SensorDataQoS());

  // Allocate GPU scratch for each eye (BGR = 3 bytes per pixel)
  size_t eye_bytes = (size_t)eye_w_ * eye_h_ * 3;
  left_gpu_ = std::make_unique<rak::core::DeviceBuffer<uint8_t>>(eye_bytes);
  right_gpu_ = std::make_unique<rak::core::DeviceBuffer<uint8_t>>(eye_bytes);
  // Temp GPU buffer for the full decoded SBS frame
  sbs_w_ = config_.sbs_width;
  sbs_h_ = config_.sbs_height;
  sbs_gpu_ = std::make_unique<rak::core::DeviceBuffer<uint8_t>>((size_t)sbs_w_ * sbs_h_ * 3);

  rak::core::init_cuda(0);

  // nvJPEG decoder (GPU-side MJPEG decode)
  nvjpeg_ = std::make_unique<NvJpegDecoder>();

  // CUDA stream for async decode + copy
  stream_ = std::make_unique<rak::core::CudaStream>();

  if (!open_device() || !init_device()) {
    throw std::runtime_error("eye: failed to initialise camera " + config_.device);
  }

  running_ = true;
  capture_thread_ = std::thread([this] { capture_loop(); });

  RCLCPP_INFO(logger_, "eye: capturing %s %dx%d SBS @ %d fps -> left/right %dx%d (nvJPEG GPU decode)",
              config_.device.c_str(), config_.sbs_width, config_.sbs_height,
              config_.fps, eye_w_, eye_h_);
}

CaptureNode::~CaptureNode() {
  running_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  close_device();
}

bool CaptureNode::open_device() {
  fd_ = open(config_.device.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "eye: cannot open %s (%s)", config_.device.c_str(), strerror(errno));
    return false;
  }
  return true;
}

bool CaptureNode::init_device() {
  struct v4l2_capability cap{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) {
    RCLCPP_ERROR(logger_, "eye: VIDIOC_QUERYCAP failed"); return false;
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    RCLCPP_ERROR(logger_, "eye: device is not a capture device"); return false;
  }
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    RCLCPP_ERROR(logger_, "eye: device does not support streaming"); return false;
  }

  struct v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config_.sbs_width;
  fmt.fmt.pix.height = config_.sbs_height;
  fmt.fmt.pix.pixelformat = v4l2_fourcc('M', 'J', 'P', 'G');
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) {
    RCLCPP_ERROR(logger_, "eye: VIDIOC_S_FMT failed"); return false;
  }
  RCLCPP_INFO(logger_, "eye: format set to %dx%d %.4s", fmt.fmt.pix.width, fmt.fmt.pix.height,
              (char *)&fmt.fmt.pix.pixelformat);

  struct v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = config_.fps;
  xioctl(fd_, VIDIOC_S_PARM, &parm);

  struct v4l2_requestbuffers req{};
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
    RCLCPP_ERROR(logger_, "eye: VIDIOC_REQBUFS failed"); return false;
  }

  buffers_.resize(req.count);
  for (size_t i = 0; i < buffers_.size(); ++i) {
    struct v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
      RCLCPP_ERROR(logger_, "eye: VIDIOC_QUERYBUF failed"); return false;
    }
    buffers_[i].length = buf.length;
    buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[i].start == MAP_FAILED) {
      RCLCPP_ERROR(logger_, "eye: mmap failed"); return false;
    }
    if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
      RCLCPP_ERROR(logger_, "eye: VIDIOC_QBUF failed"); return false;
    }
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
    RCLCPP_ERROR(logger_, "eye: VIDIOC_STREAMON failed"); return false;
  }
  return true;
}

void CaptureNode::close_device() {
  if (fd_ < 0) return;
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd_, VIDIOC_STREAMOFF, &type);
  for (auto &b : buffers_) {
    if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
  }
  buffers_.clear();
  close(fd_);
  fd_ = -1;
}

bool CaptureNode::grab_frame(std::vector<uint8_t> &jpeg_bytes) {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);
  struct timeval tv{};
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  int r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
  if (r <= 0) return false;

  struct v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_DQBUF, &buf) == -1) return false;

  jpeg_bytes.assign(static_cast<uint8_t *>(buffers_[buf.index].start),
                    static_cast<uint8_t *>(buffers_[buf.index].start) + buf.bytesused);

  xioctl(fd_, VIDIOC_QBUF, &buf);
  return true;
}

void CaptureNode::capture_loop() {
  std::vector<uint8_t> jpeg;

  // Host-side download buffer for publishing (zero-copy image transport is the upgrade path)
  std::vector<uint8_t> host_eye((size_t)eye_w_ * eye_h_ * 3);

  while (running_ && rclcpp::ok()) {
    if (!grab_frame(jpeg)) continue;

    // 1) GPU-side decode of the full SBS MJPEG frame into sbs_gpu_ (device memory).
    size_t decoded_pitch = nvjpeg_->decode_to_gpu(
        jpeg.data(), jpeg.size(), sbs_w_, sbs_h_, sbs_gpu_->data(), *stream_);

    // 2) Split into left/right eyes on the device (cudaMemcpy2D, async on our stream).
    CUDA_CHECK(cudaMemcpy2DAsync(
        left_gpu_->data(), (size_t)eye_w_ * 3,
        sbs_gpu_->data(), decoded_pitch,
        (size_t)eye_w_ * 3, eye_h_, cudaMemcpyDeviceToDevice, *stream_));
    CUDA_CHECK(cudaMemcpy2DAsync(
        right_gpu_->data(), (size_t)eye_w_ * 3,
        sbs_gpu_->data() + eye_w_ * 3, decoded_pitch,
        (size_t)eye_w_ * 3, eye_h_, cudaMemcpyDeviceToDevice, *stream_));

    // 3) Download each eye to host for the ROS message.
    CUDA_CHECK(cudaMemcpyAsync(host_eye.data(), left_gpu_->data(), host_eye.size(),
                                          cudaMemcpyDeviceToHost, *stream_));
    stream_->synchronize();

    auto stamp = now();

    auto make_image = [&](const std::string &frame, const uint8_t *eye_data) {
      auto msg = std::make_shared<sensor_msgs::msg::Image>();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame;
      msg->height = eye_h_;
      msg->width = eye_w_;
      msg->encoding = "bgr8";
      msg->step = eye_w_ * 3;
      msg->is_bigendian = 0;
      msg->data.assign(eye_data, eye_data + (size_t)eye_w_ * eye_h_ * 3);
      return msg;
    };

    pub_left_->publish(*make_image(config_.frame_id + "_left", host_eye.data()));

    // Right eye: download again (reuse host_eye buffer).
    CUDA_CHECK(cudaMemcpy(host_eye.data(), right_gpu_->data(), host_eye.size(),
                                     cudaMemcpyDeviceToHost));
    pub_right_->publish(*make_image(config_.frame_id + "_right", host_eye.data()));

    // TODO: publish CameraInfo (loaded from calibration YAML)
  }
}

}  // namespace rak::eye

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rak::eye::CaptureNode)
