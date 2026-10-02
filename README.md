# rak

AI-native stereo vision system for Jetson Orin Nano (JetPack 7.2).

Performance-first, C++-priority. Neural-network depth, detection & segmentation — not traditional CV. GPU-resident zero-copy pipeline from capture through inference.

## Packages

| Package | Language | Purpose |
|---------|----------|---------|
| `core` | C++ | Foundation: custom messages, CUDA / TensorRT / VPI shared utilities |
| `eye` | C++ | Stereo camera capture: V4L2 + **nvJPEG GPU decode**, SBS split, GPU image publish |
| `depth` | C++ | Stereo depth: VPI rectification + TensorRT neural depth estimation |
| `mind` | C++ | AI inference engine (TensorRT FP16/INT8, DLA offload) |
| `world` | C++ | Perception fusion: 3D tracking, object maps |
| `can bridge` | — | **planned, not implemented**: classic CAN 2.0A @ 1 Mbps to the two STM32F407 leaf nodes (no CAN FD — the F407 cannot). Today `motor_stub` only forwards Nav2's `Twist`, without `Vy` |
| `boot` | — | Launch files, config YAML, RViz |
| `forge` | Python | Tools: stereo calibration, profiling, CAN bus monitor |

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

Data stays GPU-resident from `eye` capture through `depth`/`mind` inference. CUDA streams overlap capture → decode → inference.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run the camera

```bash
ros2 run eye eye_capture
# or via launch:
ros2 launch boot stereo.launch.py
```

Camera: SP2812 SBS, 1280x720 @ 30 Hz → left 640x720 + right 640x720, MJPEG.

### 30 fps tuning

The eye capture node sustains a full 30 fps end-to-end. Two things make that happen:

1. **Decode thread.** `nvjpegDecode()` blocks the CPU for ~10 ms and cannot be
   pipelined in a single thread. The node runs it on a dedicated background
   thread with ping-pong double-buffering; the main thread does grab + copy +
   publish. Since camera grab (~33 ms) dominates decode (~10 ms), the decode
   thread always finishes before the next frame is needed.

2. **Fixed frame rate.** Some USB cameras expose an auto-exposure mode that
   drags the output frame rate down to ~20 fps in low light
   (`V4L2_CID_EXPOSURE_DYNAMIC_FRAMERATE`). The node disables this control at
   startup so the camera actually delivers the 30 fps requested via `S_PARM`.

## Hardware target

Jetson Orin Nano, JetPack 7.2 (CUDA 13.2, TensorRT 10.16, VPI 4.1.4).
