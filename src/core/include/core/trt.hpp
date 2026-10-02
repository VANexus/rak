#pragma once

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include <core/cuda.hpp>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace rak::core {

/** TensorRT logger that forwards to ROS2 or stderr. */
class TrtLogger : public nvinfer1::ILogger {
public:
  explicit TrtLogger(Severity threshold = Severity::kWARNING) : threshold_(threshold) {}
  void log(Severity severity, const char *msg) noexcept override;
  void set_threshold(Severity s) { threshold_ = s; }
private:
  Severity threshold_;
};

/** RAII TensorRT engine + execution context. Zero-copy with CUDA buffers. */
class TrtEngine {
public:
  TrtEngine();
  ~TrtEngine();

  TrtEngine(const TrtEngine &) = delete;
  TrtEngine &operator=(const TrtEngine &) = delete;

  /** Load a serialized .plan / .engine from disk. */
  void load(const std::string &path, TrtLogger &logger);

  /** Build an engine from an ONNX model (one-time, slow). */
  void build_from_onnx(const std::string &onnx_path, TrtLogger &logger,
                      nvinfer1::BuilderFlag precision = nvinfer1::BuilderFlag::kFP16);

  [[nodiscard]] nvinfer1::IExecutionContext *context() { return context_.get(); }
  [[nodiscard]] nvinfer1::ICudaEngine *engine() { return engine_.get(); }
  [[nodiscard]] bool ready() const { return engine_ != nullptr; }

  /** Number of IO tensors. */
  [[nodiscard]] int num_io() const;

  /** Index by name. */
  [[nodiscard]] int index_of(const std::string &name) const;

  /** Allocate the engine's binding buffers on the given device. Call once after load. */
  void allocate_buffers();

  /** Pointer to device buffer for binding i (for binding-based API). */
  [[nodiscard]] void *device_buffer(int binding_index);
  [[nodiscard]] const void *device_buffer(int binding_index) const;

  /** Enqueue inference asynchronously on the given stream. */
  bool infer(cudaStream_t stream = 0);

  /** Print a summary of all bindings (debug). */
  void print_bindings() const;

private:
  struct TxDeleter {
    template <typename T>
    void operator()(T *obj) const { delete obj; }
  };

  std::unique_ptr<TrtLogger> logger_own_;
  std::unique_ptr<nvinfer1::IRuntime, TxDeleter> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TxDeleter> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TxDeleter> context_;

  std::vector<void *> buffers_;       // device buffers per binding
  std::vector<size_t > buffer_sizes_;
  int device_id_ = 0;
};

/** Convenience: return TensorRT version string. */
std::string trt_version();

}  // namespace rak::core
