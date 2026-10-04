#pragma once

#include <cuda_runtime_api.h>
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/Types.h>
#include <vpi/WarpMap.h>
#include <vpi/algo/Remap.h>
#include <vpi/algo/StereoDisparity.h>

#include <rak_gpu/cuda.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace rak::gpu {

#define VPI_CHECK(call)                                           \
  do {                                                            \
    VPIStatus s = (call);                                         \
    if (s != VPI_SUCCESS) {                                       \
      throw std::runtime_error(std::string(__FILE__) + ":" +       \
        std::to_string(__LINE__) + " " #call " failed: " +         \
        vpiStatusGetName(s));                                     \
    }                                                             \
  } while (0)

/** Which VPI backend to use. PVA is the low-power vision accelerator on Jetson. */
enum class VpiBackend { GPU, PVA, VIC };

/** Convert our backend enum to a VPI backend flag (uint64_t bit). */
uint64_t vpi_backend_flag(VpiBackend b);

/** RAII VPI stream wrapping a CUDA stream. */
class VpiStream {
public:
  explicit VpiStream(VpiBackend backend = VpiBackend::GPU);
  ~VpiStream();

  VpiStream(const VpiStream &) = delete;
  VpiStream &operator=(const VpiStream &) = delete;

  [[nodiscard]] VPIStream get() const noexcept { return stream_; }
  operator VPIStream() const noexcept { return stream_; }

  void synchronize() const;

private:
  VPIStream stream_{};
};

/** A VPI image backed by a device pointer (zero-copy from CUDA memory). */
class VpiImage {
public:
  VpiImage();
  VpiImage(int width, int height, VPIImageFormat fmt, VpiBackend backend = VpiBackend::GPU);
  ~VpiImage();

  VpiImage(const VpiImage &) = delete;
  VpiImage &operator=(const VpiImage &) = delete;
  VpiImage(VpiImage &&) noexcept;
  VpiImage &operator=(VpiImage &&) noexcept;

  /** Wrap an existing device pointer (CUDA memory) without taking ownership. */
  void wrap_device(int width, int height, VPIImageFormat fmt, void *data,
                   VpiBackend backend = VpiBackend::GPU);

  [[nodiscard]] VPIImage get() const noexcept { return img_; }
  operator VPIImage() const noexcept { return img_; }

  [[nodiscard]] int width() const noexcept { return width_; }
  [[nodiscard]] int height() const noexcept { return height_; }
  [[nodiscard]] VPIImageFormat format() const noexcept { return fmt_; }

private:
  void reset();

  VPIImage img_{};
  int width_ = 0;
  int height_ = 0;
  VPIImageFormat fmt_ = VPI_IMAGE_FORMAT_Y8_ER;
  bool owns_ = false;
};

/** Stereo rectification using a precomputed VPI Remap warp map. */
class VpiRectifier {
public:
  VpiRectifier();
  ~VpiRectifier();

  /** Build the remap from calibration (camera matrix, distortion, R, P). */
  void init(const float camera_matrix[9], const float dist_coeffs[5],
            const float R[9], const float P[9],
            int width, int height, VpiBackend backend = VpiBackend::GPU);

  /** Submit rectification asynchronously. */
  void rectify(VpiStream &stream, const VpiImage &in, VpiImage &out) const;

private:
  VPIPayload payload_{};
  VPIWarpMap map_{};
  VpiBackend backend_ = VpiBackend::GPU;
};

/** Convert a VPI status to a human string for logging. */
std::string vpi_status_str(VPIStatus s);

}  // namespace rak::gpu
