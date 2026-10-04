#include <rak_gpu/cuda.hpp>

#include <cuda.h>

#include <iomanip>
#include <iostream>
#include <sstream>

namespace rak::gpu {

// ---------- CudaStream ----------

CudaStream::CudaStream(unsigned int flags) { CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, flags)); }
CudaStream::~CudaStream() {
  if (stream_) { cudaStreamDestroy(stream_); }
}
CudaStream::CudaStream(CudaStream &&o) noexcept : stream_(o.stream_) { o.stream_ = nullptr; }
CudaStream &CudaStream::operator=(CudaStream &&o) noexcept {
  if (this != &o) { if (stream_) cudaStreamDestroy(stream_); stream_ = o.stream_; o.stream_ = nullptr; }
  return *this;
}

void CudaStream::synchronize() const { CUDA_CHECK(cudaStreamSynchronize(stream_)); }

cudaEvent_t CudaStream::record_event() {
  cudaEvent_t e;
  CUDA_CHECK(cudaEventCreate(&e));
  CUDA_CHECK(cudaEventRecord(e, stream_));
  return e;
}

void CudaStream::wait_event(cudaEvent_t event) const {
  CUDA_CHECK(cudaStreamWaitEvent(stream_, event, 0));
}

// ---------- StreamPool ----------

StreamPool::StreamPool(std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    streams_.push_back(std::make_unique<CudaStream>());
  }
}

// ---------- GpuTimer ----------

GpuTimer::GpuTimer(cudaStream_t stream) : stream_(stream) {
  CUDA_CHECK(cudaEventCreate(&start_));
  CUDA_CHECK(cudaEventCreate(&stop_));
}
GpuTimer::~GpuTimer() {
  cudaEventDestroy(start_);
  cudaEventDestroy(stop_);
}
void GpuTimer::start() { cudaEventRecord(start_, stream_); started_ = true; }
void GpuTimer::stop() { if (started_) { cudaEventRecord(stop_, stream_); cudaEventSynchronize(stop_); started_ = false; } }
float GpuTimer::elapsed_ms() const {
  if (!started_) {
    float ms = 0.f;
    cudaEventElapsedTime(&ms, start_, stop_);
    return ms;
  }
  return 0.f;
}

// ---------- device query ----------

DeviceProps device_properties(int device) {
  cudaDeviceProp p{};
  CUDA_CHECK(cudaGetDeviceProperties(&p, device));
  DeviceProps d;
  d.name = p.name;
  d.compute_major = p.major;
  d.compute_minor = p.minor;
  d.total_mem = p.totalGlobalMem;
  d.sm_count = p.multiProcessorCount;
  d.warp_size = p.warpSize;
  // clockRate was removed from cudaDeviceProp in CUDA 13.x; query via attribute.
  cudaDeviceGetAttribute(&d.clock_rate_khz, cudaDevAttrClockRate, device);
  return d;
}

int device_count() {
  int n = 0;
  CUDA_CHECK(cudaGetDeviceCount(&n));
  return n;
}

void init_cuda(int device) {
  CUDA_CHECK(cudaSetDevice(device));
  auto d = device_properties(device);
  std::ostringstream oss;
  oss << "[rak::gpu] CUDA device " << device << ": " << d.name
      << " (SM " << d.compute_major << "." << d.compute_minor << ", "
      << d.sm_count << " SMs, " << (d.total_mem >> 20) << " MB, warp " << d.warp_size << ")";
  std::cout << oss.str() << std::endl;
}

}  // namespace rak::gpu
