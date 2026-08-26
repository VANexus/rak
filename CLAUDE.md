# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`rak` is an AI-native stereo vision system for Jetson Orin Nano (JetPack 7.2) as a ROS 2 (Jazzy) colcon workspace. Neural depth / detection / segmentation — not traditional CV. The pipeline keeps image data GPU-resident from camera capture through inference; CUDA streams overlap capture → decode → inference.

## System context (rak robot)

This is the **brain** of a three-repo robot. Perception lives here; real-time control lives in two STM32 leaf nodes that this repo commands over CAN FD.

```
[USB SBS Camera] → eye (V4L2 + nvJPEG GPU decode)
        ↓  left/right image_raw (GPU-resident)
   depth (VPI rectify + TRT depth)
        ↓  depth + rectified
   mind (TRT detection/segmentation)
        ↓  detections
   world (3D tracking & fusion)
        ↓  TrackedObjects
   nerve → CAN FD → STM32 nodes
                        ↓
        ┌───────────────┴───────────────┐
        ↓                               ↓
  rak-arm                           rak-chassis
  STM32G474 · M4F                   STM32H743 · M7
  Arm joint control                 Mobile base control
  (GitHub: Thecnfor/rak-arm)        (GitHub: Thecnfor/rak-chassis)
```

- **This repo (rak):** stereo vision + AI perception on Jetson. `eye` captures, `depth`/`mind` infer, `world` tracks, `nerve` (Rust, socketcan) sends motion commands + receives feedback over CAN FD.
- **rak-arm:** STM32G474 arm joint controller. Receives joint commands, runs HRTIM precision motor control, reports encoder/current state.
- **rak-chassis:** STM32H743 mobile base controller. Receives drive commands, runs wheel motor control + IMU/odometry fusion.

The CAN FD bus is the nervous system. `nerve` bridges ROS 2 messages ↔ CAN frames (`CanFrame.msg`). When writing perception-to-action logic, remember the loop closes through those two STM32 nodes — their command/feedback protocol (in their repos) defines what this brain can ask of the body.

## Build

The C++ packages (`core`, `eye`) link NVIDIA libraries that only exist on the Jetson target — CUDA 13.2, TensorRT (`nvinfer`/`nvonnxparser`), and VPI (`/opt/nvidia/vpi4`). They will not compile on a generic machine (this dev box has none of them). Build on the Jetson:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

- `core` must build first — every other C++ package does `find_package(core REQUIRED)` and links `core::core_lib`.
- `core` also calls `find_package(vision_msgs REQUIRED)`; that is an apt package (`ros-jazzy-vision-msgs`), not in this repo.
- The only CI-less checks are the shared `ament_lint_auto` blocks; there are no functional unit tests for the C++ code.
- Gotcha on a non-Jetson checkout: this dev machine runs ROS `lyrical` (`source /opt/ros/lyrical/setup.bash`), not `jazzy`, and still cannot satisfy the NV deps — only `forge` (pure Python) builds here.

## Packages

| Package | Language | Status |
|---------|----------|--------|
| `core` | C++ | **Foundation, implemented.** Custom messages + CUDA/TensorRT/VPI shared utilities |
| `eye` | C++ | **Only fully implemented node.** V4L2 + nvJPEG GPU decode, SBS split, GPU image publish |
| `depth`, `mind`, `world` | C++ | Scaffolds only — package + build skeleton, no source yet |
| `forge` | Python | Scaffold only |
| `boot` | — | Scaffold only — contains no launch files yet (README's `stereo.launch.py` is planned, not present) |
| `nerve` | Rust | Documented in README, **not yet in the repo** |

## Architecture

```
[USB SBS Camera] → eye (V4L2 + nvJPEG GPU decode)
        ↓  left/right image_raw (GPU-resident)
   depth (VPI rectify + TRT depth)
        ↓  depth + rectified
   mind (TRT detection/segmentation)
        ↓  detections
   world (3D tracking & fusion)
        ↓  TrackedObjects
   nerve → CAN FD → STM32 nodes
```

**`core`** (`src/core/include/core/*.hpp`) is the shared foundation and the idiomatic patterns for new code:
- `CUDA_CHECK` / `CU_CHECK` / `VPI_CHECK` macros throw `std::runtime_error` with file:line — never leak raw status codes.
- RAII wrappers: `CudaStream` + `StreamPool` + `GpuTimer` (CUDA events), `DeviceBuffer` / `PinnedBuffer` (templated, move-only), `TrtEngine` (load serialized `.engine` or `build_from_onnx`, then `allocate_buffers` + `enqueueV3`), `VpiStream` / `VpiImage` / `VpiRectifier` (VPI backends: GPU / PVA / VIC).
- Custom messages: `CanFrame`, `TrackedObject(s)`, `ModelConfig`.

**`eye`** (`src/eye/*`) is the reference implementation and the subtle part:
- V4L2 MJPEG capture of a side-by-side (SBS) frame (SP2812, 1280×720 @ 30 Hz), `NVJPEG_BACKEND_GPU_HYBRID` decode straight into a GPU BGR buffer.
- **Two threads, two CUDA streams, ping-pong double buffering.** `nvjpegDecode` blocks the CPU ~10 ms, so it runs on a dedicated decode thread; the main thread does grab (~33 ms) + copy + publish. Because grab > decode, the decode of frame N finishes while frame N+1 is being grabbed. See the `capture_loop()` timeline comment.
- Main thread copies the decoded SBS frame to host with two `cudaMemcpy2DAsync` calls (one per eye half) into **pinned** buffers, then publishes `bgr8` images on `eye/left/image_raw` and `eye/right/image_raw` (and matching `camera_info` topics — publishers exist but nothing publishes to them yet).
- FPS pitfall: some USB cameras support `V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE` (0x009a0903), which silently drops output to ~20 fps in low light. The node disables it at init — keep that.

Run the node: `ros2 run eye eye_capture` (composable component `rak::eye::CaptureNode`, registered via `rclcpp_components_register_node`).