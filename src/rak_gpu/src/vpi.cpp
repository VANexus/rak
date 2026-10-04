#include <rak_gpu/vpi_res.hpp>

#include <iostream>
#include <cstring>

namespace rak::gpu {

uint64_t vpi_backend_flag(VpiBackend b) {
  switch (b) {
    case VpiBackend::GPU:  return VPI_BACKEND_CUDA;
    case VpiBackend::PVA:  return VPI_BACKEND_PVA;
    case VpiBackend::VIC:  return VPI_BACKEND_VIC;
  }
  return VPI_BACKEND_CUDA;
}

// ---------- VpiStream ----------

VpiStream::VpiStream(VpiBackend backend) {
  VPI_CHECK(vpiStreamCreate(vpi_backend_flag(backend), &stream_));
}
VpiStream::~VpiStream() {
  if (stream_) vpiStreamDestroy(stream_);
}
void VpiStream::synchronize() const { VPI_CHECK(vpiStreamSync(stream_)); }

// ---------- VpiImage ----------

VpiImage::VpiImage() = default;

VpiImage::VpiImage(int width, int height, VPIImageFormat fmt, VpiBackend backend) {
  VPI_CHECK(vpiImageCreate(width, height, fmt, vpi_backend_flag(backend), &img_));
  width_ = width; height_ = height; fmt_ = fmt; owns_ = true;
}

VpiImage::~VpiImage() { reset(); }

VpiImage::VpiImage(VpiImage &&o) noexcept
  : img_(o.img_), width_(o.width_), height_(o.height_), fmt_(o.fmt_), owns_(o.owns_) {
  o.img_ = nullptr; o.owns_ = false;
}

VpiImage &VpiImage::operator=(VpiImage &&o) noexcept {
  if (this != &o) { reset(); img_ = o.img_; width_ = o.width_; height_ = o.height_; fmt_ = o.fmt_; owns_ = o.owns_; o.img_ = nullptr; o.owns_ = false; }
  return *this;
}

void VpiImage::reset() {
  if (img_ && owns_) vpiImageDestroy(img_);
  img_ = nullptr; owns_ = false;
}

namespace {
/** Fill a VPIImageData describing a tightly-packed single-plane CUDA buffer. */
VPIImageData make_cuda_image_data(int width, int height, VPIImageFormat fmt, void *ptr) {
  VPIImageData data{};
  data.bufferType = VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR;
  auto &plane = data.buffer.pitch;
  plane.format = fmt;
  plane.numPlanes = 1;
  plane.planes[0].pixelType = VPI_PIXEL_TYPE_INVALID;  // infer from format
  plane.planes[0].width = width;
  plane.planes[0].height = height;
  plane.planes[0].pitchBytes = width * 3;  // BGR8 / RGB8: 3 bytes/pixel, tight
  plane.planes[0].offsetBytes = 0;
  plane.planes[0].pBase = static_cast<VPIByte *>(ptr);
  return data;
}
}  // namespace

void VpiImage::wrap_device(int width, int height, VPIImageFormat fmt, void *data, VpiBackend backend) {
  reset();
  VPIImageData img = make_cuda_image_data(width, height, fmt, data);
  VPIImageWrapperParams params{};
  VPI_CHECK(vpiImageCreateWrapper(&img, &params, vpi_backend_flag(backend), &img_));
  width_ = width; height_ = height; fmt_ = fmt; owns_ = true;
}

// ---------- VpiRectifier ----------

VpiRectifier::VpiRectifier() = default;

VpiRectifier::~VpiRectifier() {
  if (payload_) vpiPayloadDestroy(payload_);
}

void VpiRectifier::init(const float camera_matrix[9], const float dist_coeffs[5],
                        const float R[9], const float P[9],
                        int width, int height, VpiBackend backend) {
  backend_ = backend;

  // Define a single-region warp grid covering the whole image.
  std::memset(&map_, 0, sizeof(map_));
  map_.grid.numHorizRegions = 1;
  map_.grid.numVertRegions = 1;
  map_.grid.regionWidth[0] = width;
  map_.grid.regionHeight[0] = height;
  map_.grid.horizInterval[0] = 1;
  map_.grid.vertInterval[0] = 1;

  // Allocate control points and fill with an identity map as a starting point.
  VPI_CHECK(vpiWarpMapAllocData(&map_));
  VPI_CHECK(vpiWarpMapGenerateIdentity(&map_));

  // NOTE: a real implementation would compute the per-pixel remap from the
  // calibration (camera_matrix, dist_coeffs, R, P) and write the source
  // coordinates into map_.keypoints. For now we ship an identity warp so the
  // pipeline runs end-to-end; calibration upload is the next step.

  VPI_CHECK(vpiCreateRemap(vpi_backend_flag(backend_), &map_, &payload_));
  (void)camera_matrix; (void)dist_coeffs; (void)R; (void)P;
}

void VpiRectifier::rectify(VpiStream &stream, const VpiImage &in, VpiImage &out) const {
  if (!payload_) throw std::runtime_error("VpiRectifier: not initialised");
  VPI_CHECK(vpiSubmitRemap(stream, vpi_backend_flag(backend_), payload_, in, out,
                           VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));
}

std::string vpi_status_str(VPIStatus s) { return vpiStatusGetName(s); }

}  // namespace rak::gpu
