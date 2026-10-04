#pragma once

#include <rak_gpu/cuda.hpp>
#include <rak_gpu/vpi_res.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include <cuda_runtime_api.h>

namespace rak::stereo_driver {

/** RAII wrapper around nvJPEG for GPU-side MJPEG -> BGR decode. */
class NvJpegDecoder {
public:
  NvJpegDecoder();
  ~NvJpegDecoder();
  NvJpegDecoder(const NvJpegDecoder &) = delete;
  NvJpegDecoder &operator=(const NvJpegDecoder &) = delete;

  /** Decode an MJPEG buffer directly into a GPU BGR buffer.
   *  Returns the decoded row pitch (bytes per row, may include padding). */
  size_t decode_to_gpu(const unsigned char *data, size_t len, int width, int height,
                       unsigned char *gpu_bgr, cudaStream_t stream);

private:
  struct Impl;
  Impl *impl_ = nullptr;
};

/**
 * V4L2 stereo camera capture node.
 *
 * Captures an SBS (side-by-side) MJPEG frame from a USB camera, decodes it on
 * the GPU via nvJPEG, splits it into left/right halves on-device, and publishes
 * each half. The frame stays GPU-resident from decode through the SBS split.
 *
 * Default: 1280x720 SBS @ 30 Hz -> left 640x720 + right 640x720.
 */
class CaptureNode : public rclcpp::Node {
public:
  struct Config {
    std::string device = "/dev/video0";
    int sbs_width = 1280;     // full SBS frame width
    int sbs_height = 720;     // full SBS frame height
    int fps = 30;
    std::string frame_id = "stereo_camera";
    std::string left_topic = "eye/left/image_raw";
    std::string right_topic = "eye/right/image_raw";
    std::string left_info_topic = "eye/left/camera_info";
    std::string right_info_topic = "eye/right/camera_info";
    bool publish_compressed = false;
  };

  explicit CaptureNode(const rclcpp::NodeOptions &opts = {});
  ~CaptureNode() override;

  CaptureNode(const CaptureNode &) = delete;
  CaptureNode &operator=(const CaptureNode &) = delete;

private:
  void capture_loop();
  bool open_device();
  bool init_device();
  void close_device();
  bool grab_frame(std::vector<uint8_t> &jpeg_bytes);

  Config config_;
  int fd_ = -1;

  // V4L2 buffer (mmap)
  struct Buffer { void *start = nullptr; size_t length = 0; };
  std::vector<Buffer> buffers_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;

  // GPU scratch buffers (ping-pong for pipeline overlap)
  static constexpr int kSlots = 2;
  std::unique_ptr<rak::gpu::DeviceBuffer<uint8_t>> sbs_gpu_[kSlots];  // decoded SBS frame per slot
  int eye_w_ = 0;
  int eye_h_ = 0;
  int sbs_w_ = 0;
  int sbs_h_ = 0;

  // Pinned host download buffers (page-locked for true async DMA)
  std::unique_ptr<rak::gpu::PinnedBuffer<uint8_t>> left_host_;
  std::unique_ptr<rak::gpu::PinnedBuffer<uint8_t>> right_host_;

  // --- decode thread: nvJPEG decode blocks the CPU (~10 ms), so it runs on a
  //     dedicated thread. The main thread does grab + copy + publish (~33 ms);
  //     since decode (10 ms) < grab (33 ms), the decode thread always finishes
  //     before the next frame is needed. Ping-pong double buffering. ---
  void decode_loop();

  std::unique_ptr<NvJpegDecoder> nvjpeg_;
  std::unique_ptr<rak::gpu::CudaStream> decode_stream_;   // owned by decode thread
  std::unique_ptr<rak::gpu::CudaStream> copy_stream_;     // owned by main thread (capture_loop)

  // Ping-pong jpeg source buffers (filled by main, consumed by decode thread).
  std::vector<uint8_t> jpeg_buf_[kSlots];
  size_t decoded_pitch_[kSlots] = {0, 0};

  // Cross-thread handshake (mutex + condvar).
  std::mutex decode_mutex_;
  std::condition_variable decode_cv_;
  int decode_slot_ = -1;                 // slot the main thread wants decoded (-1 = none)
  int ready_slot_ = -1;                  // slot that has been decoded and is ready to publish

  std::thread capture_thread_;
  std::thread decode_thread_;
  std::atomic<bool> running_{false};

  rclcpp::Logger logger_;
};

}  // namespace rak::stereo_driver
