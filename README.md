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
| `nerve` | Rust | CAN FD bridge to multiple STM32 microcontrollers (socketcan) |
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
   nerve → CAN FD → STM32 nodes
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

## Hardware target

Jetson Orin Nano, JetPack 7.2 (CUDA 13.2, TensorRT 10.16, VPI 4.1.4).
