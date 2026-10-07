#include "rak_pointcloud/depth_to_xyz.hpp"

#include <cmath>

#include <cuda_runtime.h>

namespace rak::pointcloud
{
namespace
{

__global__ void depth_to_xyz_kernel(
  const std::uint16_t * __restrict__ depth, float * __restrict__ xyz,
  int src_w, int src_h, int step, int out_w, int out_h,
  float fx, float fy, float cx, float cy,
  float depth_scale, float min_z, float max_z)
{
  const int u = blockIdx.x * blockDim.x + threadIdx.x;
  const int v = blockIdx.y * blockDim.y + threadIdx.y;
  if (u >= out_w || v >= out_h) {
    return;
  }
  const std::size_t out = static_cast<std::size_t>(v) * out_w + u;
  const int su = u * step;
  const int sv = v * step;

  // 4 floats per point: x, y, z, pad -- the PointCloud2 point_step the node asks for.
  float * point = xyz + 4 * out;
  point[3] = 0.0f;

  float z = 0.0f;
  if (su < src_w && sv < src_h) {
    z = static_cast<float>(depth[static_cast<std::size_t>(sv) * src_w + su]) * depth_scale;
  }

  // `!(z >= min_z)` rather than `z < min_z` so a NaN depth lands here too.
  if (!(z >= min_z) || z > max_z) {
    point[0] = NAN;
    point[1] = NAN;
    point[2] = NAN;
    return;
  }
  // Pinhole back-projection in the depth optical frame (x right, y down, z forward).
  point[0] = (static_cast<float>(su) - cx) * z / fx;
  point[1] = (static_cast<float>(sv) - cy) * z / fy;
  point[2] = z;
}

}  // namespace

void launch_depth_to_xyz(
  const std::uint16_t * d_depth, float * d_xyz,
  int src_w, int src_h, int step, int out_w, int out_h,
  float fx, float fy, float cx, float cy,
  float depth_scale, float min_z, float max_z,
  cudaStream_t stream)
{
  constexpr int kThreads = 16;
  const dim3 block(kThreads, kThreads);
  const dim3 grid(
    static_cast<unsigned>((out_w + kThreads - 1) / kThreads),
    static_cast<unsigned>((out_h + kThreads - 1) / kThreads));
  depth_to_xyz_kernel<<<grid, block, 0, stream>>>(
    d_depth, d_xyz, src_w, src_h, step, out_w, out_h,
    fx, fy, cx, cy, depth_scale, min_z, max_z);
}

}  // namespace rak::pointcloud
