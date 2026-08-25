#include <core/trt.hpp>

#include <NvOnnxParser.h>

#include <fstream>
#include <iostream>
#include <sstream>

namespace rak::core {

// ---------- TrtLogger ----------

void TrtLogger::log(Severity severity, const char *msg) noexcept {
  if (severity > threshold_) return;
  const char *prefix = "";
  switch (severity) {
    case Severity::kINTERNAL_ERROR: prefix = "[TRT INTERNAL]"; break;
    case Severity::kERROR:          prefix = "[TRT ERROR]"; break;
    case Severity::kWARNING:        prefix = "[TRT WARN]"; break;
    case Severity::kINFO:           prefix = "[TRT INFO]"; break;
    case Severity::kVERBOSE:        prefix = "[TRT VERBOSE]"; break;
  }
  std::cerr << prefix << " " << msg << std::endl;
}

// ---------- TrtEngine ----------

TrtEngine::TrtEngine() {
  logger_own_ = std::make_unique<TrtLogger>();
}

TrtEngine::~TrtEngine() {
  buffers_.clear();
  context_.reset();
  engine_.reset();
  runtime_.reset();
}

void TrtEngine::load(const std::string &path, TrtLogger &logger) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) throw std::runtime_error("TrtEngine: cannot open " + path);
  auto size = f.tellg();
  f.seekg(0, std::ios::beg);
  std::vector<char> blob((size_t)size);
  f.read(blob.data(), size);

  runtime_.reset(nvinfer1::createInferRuntime(logger));
  if (!runtime_) throw std::runtime_error("TrtEngine: createInferRuntime failed");
  engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
  if (!engine_) throw std::runtime_error("TrtEngine: deserializeCudaEngine failed for " + path);
  context_.reset(engine_->createExecutionContext());
  if (!context_) throw std::runtime_error("TrtEngine: createExecutionContext failed");

  allocate_buffers();
}

void TrtEngine::build_from_onnx(const std::string &onnx_path, TrtLogger &logger,
                                nvinfer1::BuilderFlag precision) {
  auto builder = std::unique_ptr<nvinfer1::IBuilder, TxDeleter>(nvinfer1::createInferBuilder(logger));
  if (!builder) throw std::runtime_error("TrtEngine: createInferBuilder failed");

  const auto explicit_batch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition, TxDeleter>(builder->createNetworkV2(explicit_batch));
  auto config = std::unique_ptr<nvinfer1::IBuilderConfig, TxDeleter>(builder->createBuilderConfig());
  auto parser = std::unique_ptr<nvonnxparser::IParser, TxDeleter>(nvonnxparser::createParser(*network, logger));

  if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    throw std::runtime_error("TrtEngine: failed to parse " + onnx_path);
  }

  config->setFlag(precision);
  if (builder->platformHasFastFp16()) config->setFlag(nvinfer1::BuilderFlag::kFP16);

  auto plan = std::unique_ptr<nvinfer1::IHostMemory, TxDeleter>(builder->buildSerializedNetwork(*network, *config));
  if (!plan) throw std::runtime_error("TrtEngine: build failed");

  runtime_.reset(nvinfer1::createInferRuntime(logger));
  engine_.reset(runtime_->deserializeCudaEngine(plan->data(), plan->size()));
  context_.reset(engine_->createExecutionContext());
  allocate_buffers();
}

int TrtEngine::num_io() const {
  return engine_ ? engine_->getNbIOTensors() : 0;
}

int TrtEngine::index_of(const std::string &name) const {
  if (!engine_) return -1;
  for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
    if (name == engine_->getIOTensorName(i)) return i;
  }
  return -1;
}

void TrtEngine::allocate_buffers() {
  if (!engine_) return;
  const int n = engine_->getNbIOTensors();
  buffers_.assign(n, nullptr);
  buffer_sizes_.assign(n, 0);
  for (int i = 0; i < n; ++i) {
    auto dims = engine_->getTensorShape(engine_->getIOTensorName(i));
    size_t vol = 1;
    for (int j = 0; j < dims.nbDims; ++j) vol *= dims.d[j];
    nvinfer1::DataType dt = engine_->getTensorDataType(engine_->getIOTensorName(i));
    size_t elem = (dt == nvinfer1::DataType::kHALF) ? 2 : (dt == nvinfer1::DataType::kINT32) ? 4 : 1;
    size_t bytes = vol * elem;
    buffer_sizes_[i] = bytes;
    if (bytes > 0) {
      void *ptr = nullptr;
      CUDA_CHECK(cudaMalloc(&ptr, bytes));
      buffers_[i] = ptr;
      context_->setTensorAddress(engine_->getIOTensorName(i), ptr);
    }
  }
}

void *TrtEngine::device_buffer(int binding_index) {
  return binding_index >= 0 && binding_index < (int)buffers_.size() ? buffers_[binding_index] : nullptr;
}
const void *TrtEngine::device_buffer(int binding_index) const {
  return binding_index >= 0 && binding_index < (int)buffers_.size() ? buffers_[binding_index] : nullptr;
}

bool TrtEngine::infer(cudaStream_t stream) {
  if (!context_) return false;
  return context_->enqueueV3(stream);
}

void TrtEngine::print_bindings() const {
  if (!engine_) return;
  for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
    const char *name = engine_->getIOTensorName(i);
    auto mode = engine_->getTensorIOMode(name);
    auto dims = engine_->getTensorShape(name);
    std::ostringstream oss;
    oss << "  [" << i << "] " << name << " " << (mode == nvinfer1::TensorIOMode::kINPUT ? "IN" : "OUT")
        << " [";
    for (int j = 0; j < dims.nbDims; ++j) oss << (j ? "x" : "") << dims.d[j];
    oss << "]";
    std::cout << oss.str() << std::endl;
  }
}

std::string trt_version() {
  return std::to_string(NV_TENSORRT_MAJOR) + "." + std::to_string(NV_TENSORRT_MINOR) + "." +
         std::to_string(NV_TENSORRT_PATCH);
}

}  // namespace rak::core
