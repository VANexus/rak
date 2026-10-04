#pragma once

#include <cuda_runtime_api.h>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace rak::gpu {

/** Turn a CUDA runtime/driver error into an exception with file:line context. */
#define CUDA_CHECK(call)                                          \
  do {                                                            \
    const cudaError_t err = (call);                               \
    if (err != cudaSuccess) {                                     \
      throw std::runtime_error(std::string(__FILE__) + ":" +       \
        std::to_string(__LINE__) + " " #call " failed: " +         \
        cudaGetErrorString(err));                                 \
    }                                                             \
  } while (0)

#define CU_CHECK(call)                                            \
  do {                                                            \
    const CUresult err = (call);                                  \
    if (err != CUDA_SUCCESS) {                                    \
      const char *msg = nullptr;                                  \
      cuGetErrorString(err, &msg);                               \
      throw std::runtime_error(std::string(__FILE__) + ":" +       \
        std::to_string(__LINE__) + " " #call " failed: " +         \
        (msg ? msg : "unknown"));                                 \
    }                                                             \
  } while (0)

using cuda_error = cudaError_t;

/** RAII CUDA stream. One per processing stage keeps the pipeline async. */
class CudaStream {
public:
  explicit CudaStream(unsigned int flags = cudaStreamNonBlocking);
  ~CudaStream();

  CudaStream(const CudaStream &) = delete;
  CudaStream &operator=(const CudaStream &) = delete;
  CudaStream(CudaStream &&) noexcept;
  CudaStream &operator=(CudaStream &&) noexcept;

  [[nodiscard]] cudaStream_t get() const noexcept { return stream_; }
  operator cudaStream_t() const noexcept { return stream_; }

  void synchronize() const;
  /** Record an event on this stream (cheap, non-blocking). */
  cudaEvent_t record_event();
  /** Make this stream wait on an event recorded on another stream. */
  void wait_event(cudaEvent_t event) const;

private:
  cudaStream_t stream_{};
};

/** Pool of streams for parallel work (e.g. left + right eye concurrently). */
class StreamPool {
public:
  explicit StreamPool(std::size_t n = 2);
  [[nodiscard]] CudaStream &get(std::size_t i) { return *streams_[i]; }
  [[nodiscard]] std::size_t size() const { return streams_.size(); }

private:
  std::vector<std::unique_ptr<CudaStream>> streams_;
};

/** Scoped timer using CUDA events. Measures GPU elapsed time in ms. */
class GpuTimer {
public:
  explicit GpuTimer(cudaStream_t stream = 0);
  ~GpuTimer();
  /** Milliseconds since start(). */
  [[nodiscard]] float elapsed_ms() const;
  void start();
  void stop();

private:
  cudaStream_t stream_;
  cudaEvent_t start_{};
  cudaEvent_t stop_{};
  bool started_{false};
};

/** Device properties query (Jetson Orin Nano: 1024 CUDA cores, 8 GB). */
struct DeviceProps {
  std::string name;
  int compute_major = 0;
  int compute_minor = 0;
  size_t total_mem = 0;
  int sm_count = 0;
  int clock_rate_khz = 0;
  int warp_size = 0;
};

DeviceProps device_properties(int device = 0);
int device_count();

/** Set the active device and print a one-line summary at startup. */
void init_cuda(int device = 0);

/** Aligned device-side buffer. Trivially-copyable scalar only. */
template <typename T>
class DeviceBuffer {
public:
  DeviceBuffer() = default;
  explicit DeviceBuffer(size_t count) { allocate(count); }
  ~DeviceBuffer() { free(); }

  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer &operator=(const DeviceBuffer &) = delete;
  DeviceBuffer(DeviceBuffer &&o) noexcept { ptr_ = o.ptr_; size_ = o.size_; o.ptr_ = nullptr; o.size_ = 0; }
  DeviceBuffer &operator=(DeviceBuffer &&o) noexcept { if (this != &o) { free(); ptr_ = o.ptr_; size_ = o.size_; o.ptr_ = nullptr; o.size_ = 0; } return *this; }

  void allocate(size_t count) { free(); void *ptr = nullptr; CUDA_CHECK(cudaMalloc(&ptr, count * sizeof(T))); ptr_ = static_cast<T *>(ptr); size_ = count; }
  void free() { if (ptr_) { cudaFree(ptr_); ptr_ = nullptr; size_ = 0; } }

  [[nodiscard]] T *data() noexcept { return ptr_; }
  [[nodiscard]] const T *data() const noexcept { return ptr_; }
  [[nodiscard]] size_t size() const noexcept { return size_; }
  [[nodiscard]] size_t bytes() const noexcept { return size_ * sizeof(T); }

  void upload(const T *host, size_t count, cudaStream_t stream = 0) {
    CUDA_CHECK(cudaMemcpyAsync(ptr_, host, count * sizeof(T), cudaMemcpyHostToDevice, stream));
  }
  void download(T *host, size_t count, cudaStream_t stream = 0) const {
    CUDA_CHECK(cudaMemcpyAsync(host, ptr_, count * sizeof(T), cudaMemcpyDeviceToHost, stream));
  }

private:
  T *ptr_ = nullptr;
  size_t size_ = 0;
};

/** Page-locked (pinned) host buffer for async cudaMemcpy with the GPU. */
template <typename T>
class PinnedBuffer {
public:
  PinnedBuffer() = default;
  explicit PinnedBuffer(size_t count) { allocate(count); }
  ~PinnedBuffer() { free(); }

  PinnedBuffer(const PinnedBuffer &) = delete;
  PinnedBuffer &operator=(const PinnedBuffer &) = delete;
  PinnedBuffer(PinnedBuffer &&o) noexcept { ptr_ = o.ptr_; size_ = o.size_; o.ptr_ = nullptr; o.size_ = 0; }
  PinnedBuffer &operator=(PinnedBuffer &&o) noexcept {
    if (this != &o) { free(); ptr_ = o.ptr_; size_ = o.size_; o.ptr_ = nullptr; o.size_ = 0; }
    return *this;
  }

  void allocate(size_t count) {
    free();
    void *ptr = nullptr;
    CUDA_CHECK(cudaMallocHost(&ptr, count * sizeof(T)));
    ptr_ = static_cast<T *>(ptr);
    size_ = count;
  }
  void free() { if (ptr_) { cudaFreeHost(ptr_); ptr_ = nullptr; size_ = 0; } }

  [[nodiscard]] T *data() noexcept { return ptr_; }
  [[nodiscard]] const T *data() const noexcept { return ptr_; }
  [[nodiscard]] size_t size() const noexcept { return size_; }
  [[nodiscard]] size_t bytes() const noexcept { return size_ * sizeof(T); }

private:
  T *ptr_ = nullptr;
  size_t size_ = 0;
};

}  // namespace rak::gpu
