// Depth image -> XYZ point cloud on the GPU.
//
// The Orbbec SDK builds its cloud on the CPU, one point per depth pixel. At
// 1280x800 @ 30 Hz that is ~1.0M back-projections per frame competing with
// everything else on a 6-core board. The maths is embarrassingly parallel, so
// it belongs on the GPU; the node only pays for the host<->device copies and
// the ROS message.
#ifndef RAK_POINCLOUD_DEPTH_TO_XYZ_HPP_
#define RAK_POINCLOUD_DEPTH_TO_XYZ_HPP_

#include <cstdint>

#include <cuda_runtime.h>

namespace rak::pointcloud
{

/// Back-project a 16-bit depth image into an interleaved float buffer.
///
/// Output is **four floats per point** -- `x, y, z, pad` -- row-major, with
/// `step` decimation applied on the way in. The padding is not decoration:
/// `PointCloud2Modifier::setPointCloud2FieldsByString(1, "xyz")` lays out the
/// PCL `PointXYZ` stride (16 bytes), so emitting a tight 12-byte triple and
/// memcpy-ing it in scrambles every point after the first. Writing the stride
/// here keeps the device->host copy a straight memcpy.
///
/// Pixels whose depth falls outside `[min_z, max_z]` are written as NaN --
/// `PointCloud2` consumers already treat NaN as "no return", and compacting
/// would need a prefix sum for no benefit here.
///
/// `d_depth` is `src_w * src_h` uint16 in the sensor's native unit; `depth_scale`
/// converts it to metres. No synchronisation is performed; the caller owns the
/// stream.
void launch_depth_to_xyz(
  const std::uint16_t * d_depth, float * d_xyz,
  int src_w, int src_h, int step, int out_w, int out_h,
  float fx, float fy, float cx, float cy,
  float depth_scale, float min_z, float max_z,
  cudaStream_t stream);

}  // namespace rak::pointcloud

#endif  // RAK_POINCLOUD_DEPTH_TO_XYZ_HPP_
