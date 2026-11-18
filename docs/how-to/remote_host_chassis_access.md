# 局域网上位机接入底盘驱动层 runbook

> 状态：现行 · 最后更新：2026-11-17

目标：同局域网的 PC 不碰 CAN、不装任何本仓代码，只通过 ROS 2 标准接口
开发上位机逻辑（遥控、导航、可视化、回归测试）。Jetson 侧驱动层必须先过
[chassis_canopen_bringup.md](chassis_canopen_bringup.md) 的 G0–G2；本文不含运动许可判定。

## 1. Jetson 侧一次性确认

```bash
ip -br addr show wlP1p1s0            # 当前实测 192.168.5.215/24
echo $ROS_DOMAIN_ID                   # 运行时域 = 47（~/.bashrc 钉死）
```

DDS = CycloneDDS（`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`），多播开启
（`src/rak_bringup/config/cyclonedds.xml`，`AllowMulticast=true`）。
**上位机不需要这份 XML**：域 47 的私有参数（MaxMessageSize 等）只影响本机吞吐，
双方用默认 Cyclone 配置即可互通；要零拷贝大图像再考虑同配置。

## 2. 上位机最小环境

```bash
sudo apt install ros-jazzy-desktop          # 或任意含 rclpy 的发行
export ROS_DOMAIN_ID=47
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep joint_states          # 出得来 = 通
```

要求：与 Jetson 同二层网段、防火墙放行多播 239.255.0.1 与 UDP 7411+。
不通先 `ping 192.168.5.215`，再查 AP 的"客户端隔离"。

## 3. 接口契约（上位机只许用这些）

| 方向 | 话题 / 动作 | 类型 | 说明 |
| --- | --- | --- | --- |
| 发 | `/mecanum_drive_controller/reference` | `geometry_msgs/TwistStamped` | 唯一速度入口（REP-103：x 前 y 左 wz 逆时针），20 Hz 持续发；**0.5 s 无新鲜输入自动置零**（`reference_timeout`）。`frame_id=base_link` |
| 收 | `/joint_states` | `sensor_msgs/JointState` | 四轮 `velocity` = rad/s（真值，0x181 反馈直读），`name` 顺序 FL/FR/RL/RR |
| 收 | `/mecanum_drive_controller/odom` | `nav_msgs/Odometry` | 轮速里程计（不含 IMU 融合） |
| 收 | `/chassis/rpdo` | `canopen_interfaces/COData` | 驱动层原始对象（诊断/黑匣子），**只读不许写** |
| 收 | `/diagnostics`、`/rak/system_state` | `diagnostic_msgs` | 域状态与数据年龄 |

硬规则（违反任何一条 = 验收作废）：

- **禁止直连 CAN**：上位机不得出现 socketcan/candump/python-can；接口只有上表。
  这是 ADR-0016 的层边界，也是"同一总线只有一个主站"的前提。
- 速度上限：`enforce_command_limits: true`，控制器命令限幅 ±10 rad/s；
  未标定前业务侧自己先钉 0.15 m/s（`chassis_tui` 同款保守值）。
- 运动许可永远在 Jetson 侧：`allow_motion`（launch 参数，默认 false）。
  上位机发什么都绕不过它——这不是故障，是设计。

## 4. 一分钟自检

```bash
# A. 反馈在跳？（Jetson bringup 运行时）
ros2 topic hz /joint_states                     # ≈50 Hz
# B. 指令闭环？车架空或有人守急停时：
timeout 2 bash -c 'ros2 topic pub -r 20 /mecanum_drive_controller/reference \
  geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.05}}}"'
ros2 topic echo --once /joint_states | grep -A5 velocity   # 四轮应同向非零
# C. 停得下？B 结束 0.5 s 后四轮应回零（reference_timeout 兜底）。
```

标准回归（同 §3 的 G3，可在上位机跑）：

```bash
ros2 run rak_commissioning chassis_forward_test -- --yes   # 需在 Jetson 上
# 或远端等效物：按 B 的形状发 20 Hz TwistStamped，采样 /joint_states 判四轮峰值同号
```

## 5. 已知坑

- **QoS**：`/joint_states` 与 `reference` 都是 RELIABLE；用 BEST_EFFORT 订/发会静默不通。
- **两个 `/controller_manager`**：Jetson 上再起一份 `chassis_bringup`，本机的
  spawner 会连到旧的——`ros2 node list` 确认唯一。
- `reference` 只认 **Stamped**；Jazzy 的 mecanum 控制器没有未 Stamped 入口。
- 上位机 `ros2 topic pub --once` 发一帧没有意义（0.5 s 超时就归零），必须持续发。
- 时间戳：控制器用自身时钟判定新鲜度，跨机 NTP 偏差不影响 `reference`
  （只看到达时间），但会污染你自己的日志对齐——跑实验先 `chronyc tracking`。
