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

## Remote desktop (VNC)

The touchscreen is shared, not re-created: `x11vnc` attaches to whatever X
display GDM is currently running (`:0` at the greeter, `:1` once someone logs
in) and streams that exact framebuffer. There is no `Xvfb`/`Xvnc`, no second
compositor and no extra window manager — the whole server costs **11.6 MiB RSS**
and touches 0 CMA.

| Network | Address |
|---------|---------|
| LAN | `orin.xrak:5900` (i.e. `192.168.5.215`, wired `192.168.5.108`) |
| Public (netmaker) | `100.121.213.15:5900` |

Password: `kali` (the team default — same as sudo).

Any stock RFB client works, because this is plain VNC and not a vendor
protocol: macOS **Screen Sharing** (`vnc://orin.xrak:5900`), Linux
(`remmina`, `tigervnc-viewer`, `virt-viewer`), Windows (RealVNC / TightVNC /
UltraVNC, or Remmina). `-ncache 0` is set on purpose — client-side caching
makes macOS Screen Sharing and several Windows viewers render the desktop wrong.

### VNC vs. the camera — read before you log in

Sharing the *logged-in* session is exactly what starves `eye`, and that trade is
already recorded in `scripts/rak-mode.sh`: a logged-in GNOME session drives
`CmaFree` to **0 kB** of the 256 MiB pool (measured again during this setup:
800 kB), and `eye` needs that pool for 4 `V4L2_MEMORY_MMAP` buffers, so capture
fails at `VIDIOC_REQBUFS`. **x11vnc is not part of that cost** — it holds no CMA
and no GPU memory; the desktop is what costs. So:

- Camera / SLAM / nav2 work → `sudo scripts/rak-mode.sh headless`. gdm gets
  masked, the X server goes away, and x11vnc just idles (one log line per 10 s,
  ~0 memory) refusing connections until the display comes back.
- Need the screen → `sudo scripts/rak-mode.sh desktop`, type the password once
  at the touchscreen (autologin is deliberately off), log in, and VNC resumes on
  its own against the new `:1`.

So "VNC is down" after a `headless` is the expected state, not a fault.

### Why a wrapper script

GDM does not keep one stable display: logging in or out tears down the X server
and starts a new one with a different number **and** a different
`Xauthority` (`/run/user/<uid>/gdm/Xauthority`). A service pointing at a fixed
`:0` therefore dies silently the first time someone logs in. `/usr/local/sbin/x11vnc-share`
re-derives both from the live `Xorg` process on every start; `x11vnc` exits when
its X server does, and systemd's `Restart=always` re-runs the probe. It also
copies the cookie to `/run/x11vnc.xauth` first, because `xauth` refuses to lock a
file owned by `gdm`/`xrak` even when run as root — that is what makes
`-auth guess` fail with `error in locking authority file`.

### Files and operations

```bash
/usr/local/sbin/x11vnc-share        # wrapper: probe display + cookie, exec x11vnc
/etc/systemd/system/x11vnc.service  # Restart=always, After=display-manager (no Wants,
                                    #  so a masked gdm does not wedge it)
/etc/x11vnc.pass                   # VNC password, root 600

sudo systemctl status|restart x11vnc
journalctl -u x11vnc -f
sudo x11vnc -storepasswd <pw> /etc/x11vnc.pass   # to rotate the password
```

To check it without a GUI client, the handshake plus auth can be driven from the
command line (openssl's legacy provider does the VNC DES): the server should
answer `RFB 003.008`, offer security type `2`, accept `kali`, and return a
`1024x600` `ServerInit` with desktop name `xrak:1`.

## Hardware target

Jetson Orin Nano, JetPack 7.2 (CUDA 13.2, TensorRT 10.16, VPI 4.1.4).
