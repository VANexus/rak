"""Full-stack load test: every TRT-capable component at once, measured.

Runs what the robot actually runs, in one process tree, with a monitor sampling memory
and per-process RSS:

  1. usb_cam 1280x720@30 (MJPG)            -- camera driver
  2. roadseg_node (TensorRT, 384x672)      -- segmentation
  3. apriltag_node                         -- fiducial detection
  4. rtabmap (localization mode)           -- visual localization
  5. Nav2 controller/planner/behaviors/bt  -- navigation stack (4 servers)
  6. sherpa kokoro TTS (CPU)               -- speech content layer

Deliberately NOT included: qwentts (1.2-1.5 GB, only for special cases), Laya decider
(engine not present on this board yet), lidar (v1.0 runs without it), depth camera
(not attached). Everything TRT-capable IS run through TensorRT.

Output: per-component peak RSS, aggregate peak, and any component that failed to start.
"""
from __future__ import annotations

import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import psutil

MONITOR = Path("/tmp/loadtest_monitor.sh")
CAM_INFO = "file:///home/xrak/.ros/camera_info/front_camera.yaml"
ENGINE = "/home/xrak/rak/data/models/roadseg/twinlite_plus_small_fp16_384x672.plan"
KOKORO = "/home/xrak/rak/acoustic/models/kokoro-int8-multi-lang-v1_1"

ROS_ENV = None


def ros_env(domain: int) -> dict:
    """Source the ROS overlay and pin a private DDS domain."""
    global ROS_ENV
    if ROS_ENV is None:
        out = subprocess.run(
            ["bash", "-lc",
             "source /opt/ros/jazzy/setup.bash && "
             "source /home/xrak/rak/install/architecture/setup.bash 2>/dev/null; "
             "env -0"],
            capture_output=True)
        env = {}
        for chunk in out.stdout.split(b"\0"):
            if b"=" in chunk:
                k, v = chunk.split(b"=", 1)
                env[k.decode()] = v.decode()
        ROS_ENV = env
    env = dict(ROS_ENV)
    env["ROS_DOMAIN_ID"] = str(domain)
    return env


def spawn(name: str, cmd: list[str], domain: int, log: Path) -> subprocess.Popen:
    handle = log.open("wb")
    proc = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT,
                            env=ros_env(domain), start_new_session=True)
    print(f"  start {name:22s} pid={proc.pid}", flush=True)
    return proc


def tts_worker(text: str, log: Path) -> subprocess.Popen:
    """Standalone kokoro TTS loop; the ROS tts_node needs the mic/speaker chain."""
    script = f'''
import time, resource, sherpa_onnx as so
MD = "{KOKORO}"
cfg = so.OfflineTtsConfig()
cfg.model.num_threads = 4
cfg.model.provider = "cpu"
k = cfg.model.kokoro
k.model = f"{{MD}}/model.int8.onnx"; k.voices = f"{{MD}}/voices.bin"
k.tokens = f"{{MD}}/tokens.txt"; k.lexicon = f"{{MD}}/lexicon-us-en.txt,{{MD}}/lexicon-zh.txt"
cfg.max_num_sentences = 1
tts = so.OfflineTts(cfg)
print("tts ready rss=%dMB" % (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024), flush=True)
end = time.time() + 150
n = 0
while time.time() < end:
    t0 = time.perf_counter()
    a = tts.generate("{text}", sid=0, speed=1.0)
    el = time.perf_counter() - t0
    dur = len(a.samples) / a.sample_rate
    n += 1
    print(f"synth {{n}} {{el*1000:.0f}}ms audio={{dur:.2f}}s rtf={{el/dur:.2f}}", flush=True)
    time.sleep(1)
print("tts done", n, flush=True)
'''
    handle = log.open("wb")
    return subprocess.Popen([sys.executable, "-c", script], stdout=handle,
                            stderr=subprocess.STDOUT, start_new_session=True)


def asr_worker(log: Path) -> subprocess.Popen:
    """Streaming zipformer ASR on synthetic 16 kHz audio (no mic needed).

    Uses the *online* recognizer, which is what the real asr_node runs. Note that
    `OnlineRecognizer.from_transducer` exposes trt_* options, but the pip wheel is
    CPU-only ("Please compile with -DSHERPA_ONNX_ENABLE_GPU=ON"), so TRT cannot be
    used for ASR here either -- same upstream limitation as TTS.
    """
    script = '''
import time, resource, numpy as np, sherpa_onnx as so
MD = "/home/xrak/rak/acoustic/models/sherpa-onnx-streaming-zipformer-zh-14M-2023-02-23"
rec = so.OnlineRecognizer.from_transducer(
    tokens=f"{MD}/tokens.txt",
    encoder=f"{MD}/encoder-epoch-99-avg-1.int8.onnx",
    decoder=f"{MD}/decoder-epoch-99-avg-1.int8.onnx",
    joiner=f"{MD}/joiner-epoch-99-avg-1.int8.onnx",
    num_threads=2, provider="cpu", decoding_method="greedy_search")
print("asr ready rss=%dMB" % (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024), flush=True)
end = time.time() + 150
n = 0
while time.time() < end:
    stream = rec.create_stream()
    stream.accept_waveform(16000, (np.random.randn(16000) * 0.01).astype(np.float32))
    t0 = time.perf_counter()
    while rec.is_ready(stream):
        rec.decode_stream(stream)
    el = (time.perf_counter() - t0) * 1000
    n += 1
    print(f"utt {n} {el:.0f}ms text={rec.get_result(stream)!r}", flush=True)
    time.sleep(0.3)
print("asr done", n, flush=True)
'''
    handle = log.open("wb")
    return subprocess.Popen([sys.executable, "-c", script], stdout=handle,
                            stderr=subprocess.STDOUT, start_new_session=True)


def laya_worker(log: Path) -> subprocess.Popen:
    """Laya TensorRT decider, driven directly through the engine (no ROS node needed)."""
    script = '''
import time, resource, ctypes, numpy as np, tensorrt as trt
PLAN = "/home/xrak/rak/data/models/laya-typed-decisions/laya-typed-decisions-bf16.plan"
lib = ctypes.CDLL("libcudart.so")
lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
lib.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
eng = trt.Runtime(trt.Logger(trt.Logger.ERROR)).deserialize_cuda_engine(open(PLAN,"rb").read())
ctx = eng.create_execution_context()
shapes = {"input_ids": (1,160), "attention_mask": (1,160), "marker_pos": (1,7),
          "marker_mask": (1,7), "qtype": (1,)}
host, dev = {}, {}
for n, shp in shapes.items():
    ctx.set_input_shape(n, shp)
    dt = trt.nptype(eng.get_tensor_dtype(n))
    host[n] = np.zeros(shp, dtype=dt)
    p = ctypes.c_void_p(); lib.cudaMalloc(ctypes.byref(p), host[n].nbytes); dev[n] = p
    ctx.set_tensor_address(n, p.value)
outs = {}
for i in range(eng.num_io_tensors):
    n = eng.get_tensor_name(i)
    if str(eng.get_tensor_mode(n)).startswith("TensorIOMode.OUTPUT"):
        shp = tuple(ctx.get_tensor_shape(n))
        host[n] = np.zeros(shp, dtype=trt.nptype(eng.get_tensor_dtype(n)))
        p = ctypes.c_void_p(); lib.cudaMalloc(ctypes.byref(p), host[n].nbytes); dev[n] = p
        ctx.set_tensor_address(n, p.value)
stream = ctypes.c_void_p(); lib.cudaStreamCreate(ctypes.byref(stream))
print("laya ready rss=%dMB" % (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024), flush=True)
end = time.time() + 150
n = 0
lat = []
while time.time() < end:
    for name in shapes:
        host[name][:] = np.random.randint(0, 1000, size=shapes[name]).astype(host[name].dtype)
        lib.cudaMemcpy(dev[name], host[name].ctypes.data, host[name].nbytes, 1)
    t0 = time.perf_counter()
    ok = ctx.execute_async_v3(stream.value)
    lib.cudaStreamSynchronize(stream)
    lat.append((time.perf_counter()-t0)*1000)
    n += 1
    time.sleep(0.2)
lat = lat[1:] or lat
print(f"laya done {n} iters, mean {sum(lat)/len(lat):.1f} ms ok={ok}", flush=True)
'''
    handle = log.open("wb")
    return subprocess.Popen([sys.executable, "-c", script], stdout=handle,
                            stderr=subprocess.STDOUT, start_new_session=True)


def tree_rss(proc: subprocess.Popen) -> int:
    """Summed RSS of the process and its children, in MB."""
    try:
        root = psutil.Process(proc.pid)
        total = root.memory_info().rss
        for child in root.children(recursive=True):
            try:
                total += child.memory_info().rss
            except psutil.Error:
                pass
        return total // (1024 * 1024)
    except psutil.Error:
        return 0


def main() -> int:
    domain = 150
    logs = Path("/tmp/loadtest")
    logs.mkdir(exist_ok=True)
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 150

    procs: dict[str, subprocess.Popen] = {}
    print("== full-stack load test ==", flush=True)
    procs["camera"] = spawn("camera", [
        "ros2", "launch", "rak_bringup", "camera.launch.py",
        f"camera_info_url:={CAM_INFO}"], domain, logs / "camera.log")
    time.sleep(8)
    procs["roadseg"] = spawn("roadseg", [
        "ros2", "run", "rak_road_perception", "roadseg_node", "--ros-args",
        "-p", f"engine_path:={ENGINE}",
        "-p", "input_mode:=ros_image",
        "-p", "image_topic:=/camera/image_raw"], domain, logs / "roadseg.log")
    procs["apriltag"] = spawn("apriltag", [
        "ros2", "run", "apriltag_ros", "apriltag_node", "--ros-args",
        "-r", "image_rect:=/camera/image_raw",
        "-r", "camera_info:=/camera/camera_info",
        "-p", "size:=0.16", "-p", "family:=36h11",
        "-p", "qos_profile:=sensor_data"], domain, logs / "apriltag.log")
    procs["rtabmap"] = spawn("rtabmap", [
        "ros2", "launch", "rtabmap_launch", "rtabmap.launch.py",
        "localization:=true", "database_path:=/tmp/loadtest/load.db",
        "frame_id:=base_footprint", "stereo:=false", "subscribe_rgb:=true",
        "subscribe_depth:=false", "subscribe_rgbd:=false",
        "rgb_topic:=/camera/image_raw", "camera_info_topic:=/camera/camera_info",
        "visual_odometry:=false", "rtabmap_viz:=false", "rviz:=false",
        "approx_sync:=true"], domain, logs / "rtabmap.log")
    # Nav2's costmaps need base_link -> odom. On the robot that comes from the EKF
    # (ekf.yaml); this test supplies a static identity so the stack actually activates
    # (measured: without it, lifecycle bringup aborts and every server stays inactive).
    # Two static transforms stand in for what the real robot gets from the EKF
    # (odom->base_footprint) and from rtabmap localization (map->odom). Neither exists
    # on this board yet: no map has been built, and the EKF is not part of this test.
    # Without both, Nav2 lifecycle bringup aborts and the servers stay inactive, which
    # would make the load numbers meaningless.
    procs["tf"] = spawn("tf", [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--frame-id", "odom", "--child-frame-id", "base_footprint"], domain, logs / "tf.log")
    procs["tf_map"] = spawn("tf_map", [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--frame-id", "map", "--child-frame-id", "odom"], domain, logs / "tf_map.log")
    procs["nav2"] = spawn("nav2", [
        "ros2", "launch", "rak_bringup", "nav2.launch.py",
        "sensors:=camera", "odom_source:=none", "motion_backend:=none",
        "publish_robot_tf:=true"], domain, logs / "nav2.log")
    procs["tts"] = tts_worker("前方道路畅通，可以出发。", logs / "tts.log")
    procs["asr"] = asr_worker(logs / "asr.log")
    procs["laya"] = laya_worker(logs / "laya.log")

    monitor = subprocess.Popen(["bash", str(MONITOR), "/tmp/loadtest/system.log",
                                str(duration)])
    print(f"  sampling for {duration}s ...", flush=True)

    peak = {name: 0 for name in procs}
    deadline = time.time() + duration
    while time.time() < deadline:
        for name, proc in procs.items():
            peak[name] = max(peak[name], tree_rss(proc))
        time.sleep(2)

    monitor.wait(timeout=30)
    print("\n== per-component peak RSS ==")
    for name, mb in sorted(peak.items(), key=lambda kv: -kv[1]):
        alive = procs[name].poll() is None
        print(f"  {name:12s} {mb:5d} MB  {'running' if alive else 'EXITED'}")
    total = sum(peak.values())
    print(f"  {'TOTAL':12s} {total:5d} MB")

    for proc in procs.values():
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(3)
    for proc in procs.values():
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    print("\n== failures ==")
    failed = False
    for name in procs:
        log = (logs / f"{name}.log").read_text(errors="replace")
        bad = [ln for ln in log.splitlines()
               if re.search(r"\b(ERROR|FATAL|Traceback|died|Failed)\b", ln)]
        if bad:
            failed = True
            print(f"  {name}: {len(bad)} error lines, first: {bad[0][:120]}")
    if not failed:
        print("  none")
    return 0


if __name__ == "__main__":
    sys.exit(main())
