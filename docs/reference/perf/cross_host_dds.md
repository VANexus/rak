# 跨主机的 DDS / 网络通信优化（Jetson ↔ 本机，802.11ax WiFi）

> 状态：现行 · 最后更新：2026-11-09

> 日期：2026-11-09。针对 Jetson Orin Nano（`ssh orin`）与工作机
> （`ssh nexus`，路由器/网关）之间的 RViz / 感知数据流。
> 结论是**测出来的**，不是一个形而上清单——见下。所有测量在当天两条链路上现测。

## TL;DR

- **远程 RViz 放在工作机（本机）跑，不在 Jetson 上跑。** Jetson 本就该 headless
  （CMA 冲突），RViz 只是订阅者，消息定义两代一致，本机直接订阅板上话题即可。
- DDS 跨机已通：两边对齐 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` +
  相同 `ROS_DOMAIN_ID` + `CYCLONEDDS_URI` 即可（板上有 env-hook 自动设；
  本机见文末一键脚本）。
- **增大 Cyclone 收发包缓冲（`Socket{Recv,Send}BufferSize`）没有可复现提升。**
  在本机干净 A/B（唯一变量 = `CYCLONEDDS_URI`）：19.9 vs 20.2 fps——小于波动。
  **已回滚**，不在团队默认配置里加。下结论的测量看「实测」节。
- **Wi-Fi power save 是一个待验证的假设，尚未干净测试。** 早期一个不严格的对照
  （开启→关闭那一步同时改了缓冲和推送频率）暗示关掉有收益，但和后面干净的 A/B
  对不上。它停在「候选旋钮」的档位，见「待验证」。

## 链路现状（2026-11-09）

| 项 | 本机（Shizurak） | Jetson（orin） |
|---|---|---|
| 接口 | `wlp0s20f3` | `wlP1p1s0` |
| 关联 | Rak @ 5260 MHz, 802.11ax, **2401 Mbit/s** (HE-MCS11 NSS2) | Rak @ 5260 MHz, 802.11ax, **866/433 Mbit/s** (VHT-MCS9) |
| 信号 | -43 dBm | -42 dBm |
| **power save** | 默认 **on**（关掉有收益） | 已 **off** |
| 内核缓冲 | `rmem_max/wmem_max` 16 MB（`/etc/sysctl.d/99-rak-dds.conf`） | 16 MB |

链路本身健康：`ping` 2000 包 **0% loss，avg 1.4 ms，max 19 ms**。延迟噪声是 WiFi
本身的，不是接口调优能改的。

## 实测

`dds_bench.py`：发 `sensor_msgs/Image`（640×480 16UC1，614 KB/帧）跨 WiFi 30–40fps，
测对端收帧率（最接近 RViz 看相机图的实际负载）。

| 配置 | 收帧率 | 有效性 |
|---|---|---|
| 基线（未设 `CYCLONEDDS_URI`，wifi power save off） | 19.9 fps | ✅ 干净 |
| 仓库配置（+8 MB 收发包缓冲） | 20.2 fps | ✅ 干净——与基线**无显著差异** → 回滚 |
| 早期不严格对照（关 power save 的同时改缓冲/频率） | 17.5 → 24 fps | ⚠️ 变量混淆，作废 |

同时段 `iperf3`：TCP **407 Mbit/s**、UDP 200 M 丢包 **0.15%**、jitter <0.1 ms；
`ping` 2000 包 **0% loss, avg 1.4 ms**。WiFi 链路的带宽 / 丢包都不是瓶颈。

## 结论与建议

1. **远程预览**：本机 `source /opt/ros/lyrical/setup.bash` 后跑
   `rviz2 -d .../sensors_rviz.rviz`。板子只跑 headless 数据栈。
2. **不要**把 `Socket{Recv,Send}BufferSize` 加进团队 `cyclonedds.xml`——干净 A/B
   无收益，而它是对每个加载该配置的节点生效的全局改动。
3. 板端 `power_save off`、16 MB 内核缓冲已就位；仓库无需改动
   （`cyclonedds.xml` 已回滚）。

## 待验证（下次正确测量）

- **Wi-Fi power save** 是否是接收端丢帧的主因。做法：固定推送频率和缓冲设置不变，
  只切 `iw dev wlp0s20f3 set power_save on|off`，各测一次做严格对照。
  当前证据只够把它列为候选，不够下结论。
- 若 dips 确由调度/掉电引起，再考虑 `SocketReceiveBufferSize`——但那也要先证明
  buff 真的有帮助，再进团队配置。

## 一键在本机开远程 RViz

```sh
bash src/rak_bringup/scripts/run_remote_rviz.sh   # 详见该文件头注释
```

（需要本机 ROS 已装并 `source`；`RMW_IMPLEMENTATION`/`CYCLONEDDS_URI`/`ROS_DOMAIN_ID`
按板上保持一致，默认 `rmw_cyclonedds_cpp` + domain 0。）