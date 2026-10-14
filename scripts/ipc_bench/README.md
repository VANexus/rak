# ipc_bench — IPC 基线基准（AI 性能工作, 2026-10-10）

合成大载荷 Image 的发布/订阅基准，用于量化「跨进程 DDS 传输」与「进程内零拷贝」的
吞吐、延迟、丢帧、CPU、内存。结果与结论见 [`docs/perf/ipc_baseline.md`](../../docs/perf/ipc_baseline.md)。
**不作为 colcon 包留在 `src/`**（避免触碰 21 包门禁）；需要时复制到独立工作区编译。

## 构建

```bash
mkdir -p ~/ipc_bench_ws/src/ipc_bench/src
cp src/ipc_bench.cpp ~/ipc_bench_ws/src/ipc_bench/src/
cp CMakeLists.txt package.xml ~/ipc_bench_ws/src/ipc_bench/
cd ~/ipc_bench_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select ipc_bench
```

## 用法

```bash
BIN=~/ipc_bench_ws/install/ipc_bench/lib/ipc_bench/ipc_bench
$BIN --mode pub  --topic /ipcbench/x --w 1280 --h 800 --bpp 2 --fps 30 --secs 8 --qos 5 --be 1
$BIN --mode sub  --topic /ipcbench/x --secs 12
$BIN --mode both --intra 1 --topic /ipcbench/x --w 1280 --h 800 --bpp 2 --fps 30 --secs 8   # 进程内零拷贝
```

- `--bpp 2` = 16UC1（深度规格 61 MB/s）；`--bpp 3` = rgb8（彩色规格 92 MB/s）
- `--be 1` = best-effort；`--qos N` = KEEP_LAST(N)
- **跑跨进程用例时先起订阅端再起发布端**（否则测的是时间窗重叠，会产生假丢帧）

## 内容

| 文件 | 说明 |
|---|---|
| `src/ipc_bench.cpp` | 基准程序（pub/sub/both + intra 开关） |
| `run_matrix.sh` | 完整矩阵（A 61MB/s / B 92MB/s / 单进程 intra） |
| `case_udp.sh` | 单用例 + 内核 UDP 计数器前后差（定位丢包层） |
| `cx_t1.xml` | CycloneDDS 变体：`<Internal><MaxMessageSize>65500`（已实测：p50 −30%） |
| `cx_lo.xml` / `cx_lo_nomc.xml` | 对照变体（lo 绑定 / 关多播）——实测**无收益**，不建议采用 |

> ⚠️ `SocketReceiveBufferSize` 在 CycloneDDS 0.10.5 的两种写法（扁平 / `<Internal><Socket>` 子结构）
> 都被 schema 拒（后者报 "ambiguous element prefix"）；改用内核 `net.core.rmem_max`（见报告 §3.2）。