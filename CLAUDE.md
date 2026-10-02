# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`rak` is an AI-native stereo vision system for Jetson Orin Nano (JetPack 7.2) as a ROS 2 (Jazzy) colcon workspace. Neural depth / detection / segmentation — not traditional CV. The pipeline keeps image data GPU-resident from camera capture through inference; CUDA streams overlap capture → decode → inference.

## System context (rak robot)

This is the **brain** of a three-repo robot. Perception lives here; real-time control lives in two STM32F407 leaf nodes that this repo commands over classic CAN 2.0A @ 1 Mbps.

```
[USB SBS Camera] → eye (V4L2 + nvJPEG GPU decode)
        ↓  left/right image_raw (GPU-resident)
   depth (VPI rectify + TRT depth)
        ↓  depth + rectified
   mind (TRT detection/segmentation)
        ↓  detections
   world (3D tracking & fusion)
        ↓  TrackedObjects
   can bridge (planned, not implemented yet) → CAN 2.0A 1 Mbps
                        ↓
        ┌───────────────┴───────────────┐
        ↓                               ↓
  vehical_rak_arm                   vehical_rak_chassis
  STM32F407VET6 · M4F               STM32F407VET6 · M4F
  4 joints 25 kHz + base stepper    4 mecanum wheels 25 kHz
  VANexus/vehical_rak_arm           VANexus/vehical_rak_chassis
```

> The two leaf nodes are **the same board**: WHEELTEC C30D (STM32F407VET6 with an
> onboard 4-channel AT8236 driver, onboard CAN transceiver on CAN1 = PD0/PD1).
> Only the role differs. The bus is **classic CAN 2.0A @ 1 Mbps — no CAN FD**: the
> Orin's `mttcan` can do FD, the F407's bxCAN cannot, so an FD frame would be
> error-framed by both leaf nodes. Never pass `fd on` when bringing up `can0`.

- **This repo (rak):** perception + decision on the Orin — `eye` captures, `brain` decides, `astra_bridge` feeds the camera. **There is no CAN bridge here yet**: `nerve` was never implemented, `CanFrame.msg` is gone, and `motor_stub/cmd_vel_stub.py` only forwards Nav2's `Twist` from `/cmd_vel` to `/cmd_vel_raw` (no `Vy`, no enable bit, no CRC/counter). The wire contract lives in the leaf-node repos.
- **vehical_rak_chassis:** mecanum base. Takes `(Vx, Vy, ωz)` at 100 Hz, does its own wheel inverse kinematics and closes four wheels at **25 kHz** (fixed-point, inside a TIM ISR, outside the RTOS scheduler); reports measured body velocity at 100 Hz.
- **vehical_rak_arm:** 4-DOF arm. Takes joint targets from MoveIt — **the IK lives here, not on the MCU** — and closes 4 identical MG513P30_12V geared motors at **25 kHz**, plus a base stepper on a 1 kHz position loop. It never powers into motion by itself: homing is explicit (`HOME_REQUEST`, low-speed single-direction stall detection), and a lost bus makes it *hold position* instead of releasing, because the arm falls when unpowered.

That CAN bus is the nervous system, but the wire format is **not** defined here: the source of truth is `docs/PROTOCOL.md` (CAN v3) plus the machine-readable `docs/protocol-signals.yaml`, shipped in each leaf-node repo, with `tools/can_check.py` as the codec gate (structure check + golden vectors + CRC tamper cases). Read those before writing perception-to-action logic. Frame IDs carry priority — safety `0x010`, EMCY `0x020/0x021` and the 1 kHz interlock `0x030/0x031` all sit below command frames `0x100/0x101/0x102`, so an e-stop always wins arbitration. Bus timing: 100 Hz Jetson↔leaf, 1 kHz between the two leaf nodes.

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
   can bridge (planned) → CAN 2.0A 1 Mbps → 2× STM32F407
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