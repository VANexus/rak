#include <eye/capture_node.hpp>

#include <core/cuda.hpp>

#include <nvjpeg.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>

#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>

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
  // GPU_HYBRID: Huffman decode runs on GPU too, so nvjpegDecode() is fully
  // async (CPU submits and returns immediately). This lets us overlap decode
  // with the next frame's camera grab.
  nvjpegCheck(nvjpegCreate(NVJPEG_BACKEND_GPU_HYBRID, nullptr, &impl_->handle), "nvjpegCreate(GPU_HYBRID)");
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

  // Ping-pong GPU buffer for the decoded SBS frame (pipeline overlap)
  sbs_w_ = config_.sbs_width;
  sbs_h_ = config_.sbs_height;
  for (int i = 0; i < kSlots; ++i) {
    sbs_gpu_[i] = std::make_unique<rak::core::DeviceBuffer<uint8_t>>((size_t)sbs_w_ * sbs_h_ * 3);
  }

  // Pinned (page-locked) host download buffers — required for true async DMA
  size_t eye_bytes = (size_t)eye_w_ * eye_h_ * 3;
  left_host_ = std::make_unique<rak::core::PinnedBuffer<uint8_t>>(eye_bytes);
  right_host_ = std::make_unique<rak::core::PinnedBuffer<uint8_t>>(eye_bytes);

  rak::core::init_cuda(0);

  // nvJPEG decoder (GPU-side MJPEG decode) — owned by the decode thread.
  nvjpeg_ = std::make_unique<NvJpegDecoder>();

  // Two streams on two threads: decode_stream_ is used by the decode thread,
  // copy_stream_ by the main capture thread (grab + copy + publish).
  decode_stream_ = std::make_unique<rak::core::CudaStream>();
  copy_stream_ = std::make_unique<rak::core::CudaStream>();

  if (!open_device() || !init_device()) {
    throw std::runtime_error("eye: failed to initialise camera " + config_.device);
  }

  running_ = true;
  decode_thread_ = std::thread([this] { decode_loop(); });
  capture_thread_ = std::thread([this] { capture_loop(); });

  RCLCPP_INFO(logger_, "eye: capturing %s %dx%d SBS @ %d fps -> left/right %dx%d (nvJPEG GPU decode thread, pinned DMA)",
              config_.device.c_str(), config_.sbs_width, config_.sbs_height,
              config_.fps, eye_w_, eye_h_);
}

CaptureNode::~CaptureNode() {
  running_ = false;
  decode_cv_.notify_all();  // wake decode thread so it can exit
  if (decode_thread_.joinable()) decode_thread_.join();
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

  // Disable exposure_dynamic_framerate: when enabled, the camera lowers its
  // output frame rate whenever the auto-exposure time grows (e.g. low light),
  // which would drag us from 30 fps down to ~20 fps. With it off, exposure is
  // clamped to the frame interval so the camera actually delivers the rate we
  // asked for via S_PARM. Skip silently if the control is absent.
  {
    struct v4l2_control ctrl{};
    ctrl.id = 0x009a0903;  // V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE (not in this kernel's videodev2.h)
    ctrl.value = 0;
    if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) == -1) {
      RCLCPP_WARN(logger_, "eye: could not disable exposure_dynamic_framerate (errno %d) — fps may drop in low light", errno);
    } else {
      RCLCPP_INFO(logger_, "eye: exposure_dynamic_framerate disabled -> fixed %d fps", config_.fps);
    }
  }

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

void CaptureNode::decode_loop() {
  // Dedicated decode thread: nvjpegDecode blocks the CPU (~10 ms). By running
  // it here, that 10 ms overlaps the main thread's camera grab (~33 ms).
  // Handshake: main thread sets decode_slot_ to say "decode jpeg_buf_[slot]
  // into sbs_gpu_[slot]"; we do it, then set ready_slot_ = slot.
  while (running_) {
    int slot;
    {
      std::unique_lock<std::mutex> lk(decode_mutex_);
      decode_cv_.wait(lk, [this] { return decode_slot_ >= 0 || !running_; });
      if (!running_) break;
      slot = decode_slot_;
      decode_slot_ = -1;
    }
    // Synchronous GPU decode into the ping-pong slot. This blocks ~10 ms.
    decoded_pitch_[slot] = nvjpeg_->decode_to_gpu(
        jpeg_buf_[slot].data(), jpeg_buf_[slot].size(),
        sbs_w_, sbs_h_, sbs_gpu_[slot]->data(), *decode_stream_);
    {
      std::lock_guard<std::mutex> lk(decode_mutex_);
      ready_slot_ = slot;
    }
  }
}

void CaptureNode::capture_loop() {
  // Main thread: grab -> copy -> publish. The decode of frame N happens on the
  // decode thread while we grab frame N+1 (~33 ms > 10 ms decode), so by the
  // time we need frame N decoded, it is always ready.
  //
  // Timeline (wall-clock, one iteration == one published frame):
  //   grab frame N into jpeg_buf_[slot]      (~33 ms; decode thread decodes N-1 meanwhile)
  //   hand slot to decode thread (non-blocking)
  //   wait for ready_slot_ == 1-slot         (~0 ms, decode always finished)
  //   copy sbs_gpu_[1-slot] -> host + publish (~0.2 ms pinned DMA)
  //   slot = 1-slot

  const size_t eye_bytes = (size_t)eye_w_ * eye_h_ * 3;
  int slot = 0;

  // Prime: grab + handoff the first frame so the decode thread has work.
  if (!grab_frame(jpeg_buf_[slot])) return;
  {
    std::lock_guard<std::mutex> lk(decode_mutex_);
    decode_slot_ = slot;
  }
  decode_cv_.notify_one();
  slot = 1 - slot;

  int frame_idx = 0;
  auto fps_t0 = std::chrono::high_resolution_clock::now();
  while (running_ && rclcpp::ok()) {
    // --- stage 1: grab next frame (~33 ms, decode thread works during this) ---
    if (!grab_frame(jpeg_buf_[slot])) continue;

    // Hand this frame to the decode thread (it will finish well before we need it).
    {
      std::lock_guard<std::mutex> lk(decode_mutex_);
      decode_slot_ = slot;
    }
    decode_cv_.notify_one();

    // --- stage 2: wait for the OTHER frame (1-slot) to be decoded ---
    {
      std::unique_lock<std::mutex> lk(decode_mutex_);
      decode_cv_.wait(lk, [this, slot] { return ready_slot_ == 1 - slot || !running_; });
      ready_slot_ = -1;
    }

    const int pub_slot = 1 - slot;
    const size_t decoded_pitch = decoded_pitch_[pub_slot];

    // --- stage 3: copy decoded SBS frame -> pinned host, split left/right ---
    CUDA_CHECK(cudaMemcpy2DAsync(
        left_host_->data(), (size_t)eye_w_ * 3,
        sbs_gpu_[pub_slot]->data(), decoded_pitch,
        (size_t)eye_w_ * 3, eye_h_, cudaMemcpyDeviceToHost, *copy_stream_));
    CUDA_CHECK(cudaMemcpy2DAsync(
        right_host_->data(), (size_t)eye_w_ * 3,
        sbs_gpu_[pub_slot]->data() + eye_w_ * 3, decoded_pitch,
        (size_t)eye_w_ * 3, eye_h_, cudaMemcpyDeviceToHost, *copy_stream_));
    copy_stream_->synchronize();

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
      msg->data.assign(eye_data, eye_data + eye_bytes);
      return msg;
    };
    pub_left_->publish(*make_image(config_.frame_id + "_left", left_host_->data()));
    pub_right_->publish(*make_image(config_.frame_id + "_right", right_host_->data()));

    slot = 1 - slot;

    if (++frame_idx % 30 == 0) {
      auto t1 = std::chrono::high_resolution_clock::now();
      float secs = std::chrono::duration<float>(t1 - fps_t0).count();
      float fps = 30.0f / secs;
      RCLCPP_INFO(logger_, "eye: %.1f fps", fps);
      fps_t0 = t1;
    }
    // TODO: publish CameraInfo (loaded from calibration YAML)
  }
}

}  // namespace rak::eye

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rak::eye::CaptureNode)
