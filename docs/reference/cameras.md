# RGB-D Camera Backends

> 状态：现行 · 最后更新：2026-10-07

`rak_sensor_bringup` owns the selectable RGB-D camera entry point. It starts one
backend at a time and keeps camera SDKs out of navigation and perception
consumers.

## Select a camera

```bash
ros2 launch rak_sensor_bringup depth_camera.launch.py
ros2 launch rak_sensor_bringup depth_camera.launch.py camera:=astra
ros2 launch rak_sensor_bringup depth_camera.launch.py camera:=gemini335l
```

Gemini 335L is the default. Set its serial explicitly when multiple matching
devices may be connected:

```bash
ros2 launch rak_sensor_bringup depth_camera.launch.py \
  camera:=gemini335l serial_number:=YOUR_SERIAL
```

On this robot the selector pins the connected unit by serial. On another unit,
pass its serial explicitly. The profile is kept in
`src/rak_sensor_bringup/config/gemini335l.yaml`; override it with
`config_file_path:=/path/to/another_profile.yaml`.

For focused device bringup, `astra.launch.py` and `gemini335l.launch.py` remain
available separately. Do not launch either alongside `depth_camera.launch.py`.

## Driver Ownership

| Backend | ROS driver | Ownership |
|---|---|---|
| Gemini 335L | Orbbec `orbbec_camera`, SDK v2 | Upstream Orbbec ROS 2 wrapper; this workspace only supplies a launch adapter |
| Astra | `rak_astra_driver` | Existing Astra-specific bridge and SDK v1 integration |

The Gemini backend uses Orbbec's `gemini_330_series.launch.py`. The upstream
wrapper supports ROS 2 Jazzy and lists Gemini 335L as supported. The robot uses
the official `ros-jazzy-orbbec-camera` binary package (2.9.3); install it from
the ROS apt repository and source `/opt/ros/jazzy` before this workspace. The
official udev rule must also be installed so the ordinary ROS user can access
USB without running the node as root. Do not copy the SDK or upstream driver
sources into `src/vendor`. Both the package and device access rule are already
installed on the current Jetson.

References:

- [Orbbec SDK v2 ROS 2 wrapper](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)
- [Gemini 330 series documentation](https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation)

## ROS Interface

The intended shared RGB-D interface is:

| Stream | Topic |
|---|---|
| RGB image | `/camera/color/image_raw` |
| RGB calibration | `/camera/color/camera_info` |
| Depth image | `/camera/depth/image_raw` |
| Depth calibration | `/camera/depth/camera_info` |

The Gemini adapter uses `camera_name:=camera` so the upstream driver's
namespace matches the Astra topics. The verified device profiles support
1280x800 at 30 Hz for both RGB (`RGB`) and depth (`Y16`). The default
configuration uses V4L2 and RGB, as recommended by Orbbec to reduce Jetson CPU
load. It enables hardware noise removal, RGB/depth frame synchronization,
host-synchronized timestamps, factory stream extrinsics, and combined 200 Hz
accelerometer/gyroscope output. Software depth alignment and software noise
removal are disabled. Depth images use `16UC1`; the Orbbec SDK scales pixel
values to millimeters. Color and depth image transport are restricted to the
installed `raw` plugin, and the unsupported color-preset override is left
unset. Point-cloud publication is disabled by default to avoid unnecessary
bandwidth and processing; enable it for a specific run with
`enable_point_cloud:=true`. Keep `enable_point_cloud` out of the Gemini YAML
profile: the upstream launch merges YAML values over launch arguments, so a
YAML value would override the adapter's explicit launch setting.

The Gemini endpoints above were verified on the Jetson with the connected
335L. Its RGB image is `rgb8` in `camera_color_optical_frame`; depth is `16UC1`
in `camera_depth_optical_frame`. Both streams ran at approximately 29.96 Hz,
and the combined IMU topic ran at approximately 200 Hz. Across 80 paired
CameraInfo updates, the RGB/depth header timestamp skew was approximately
5.41 ms. CameraInfo messages contained valid 1280x800 intrinsics. The SDK
reported USB 3.2 and factory calibration available; customer calibration is
not set. The source Astra adapter uses the same image topic names, but it
cannot be built or run on this Jetson until Orbbec SDK v1.10.18 is installed.

This common topic layout does not imply identical frame IDs, image encodings,
timestamps, or RGB/depth registration. Consumers must use each message's
`CameraInfo` and frame IDs. Gemini factory depth-to-color extrinsics and
camera-internal TF were received, but depth registration is disabled and the
camera-to-`base_link` transform still requires measured installation
calibration; neither backend may invent it. One recoverable SDK warning about
an absent optional metadata field appeared during the first run; the image and
IMU streams continued at their target rates, and it did not recur after restart.

The selector guarantees only that one RGB-D backend is started. It does not
start a point-cloud processor, object detector, SLAM source, or Nav2 depth
obstacle layer.
