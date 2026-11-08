# ADR-0023: 时间基准与多源对齐 —— 采样时刻打戳，单一时钟域

- 状态: 已接受（策略部分；Allan/直行回归的数值仍待实测替换）
- 日期: 2026-10-29
- 相关: `src/rak_imu_driver/rak_imu_driver/i2c_imu.py` ·
  `src/rak_imu_driver/rak_imu_driver/spi_imu.py` ·
  `src/rak_bringup/config/ekf.yaml` · `src/rak_canopen/config/canopen_contract.yaml` ·
  `scripts/calibrate_imu.py` · `scripts/timebase_gate.py` · ADR-0002 §6 · ADR-0022

## 背景

多源融合的精度上限由**时间基准**决定，不是由滤波器阶数决定。当前有三处硬伤：

1. **打戳发生在发布时刻**。`i2c_imu.py:240` 用 `get_clock().now()` 在 `step()` 里打戳，
   而 `step()` 由 1 kHz timer 驱动。于是时间戳 = 采样时刻 + timer 抖动 + 调度延迟。
   现场实测 903.6 Hz（目标 1000 Hz）意味着平均每隔 ~11 帧就丢一拍，
   这些帧的时间戳仍然"看起来正常"——**误差不可观测**，只能表现为 EKF 的 yaw 漂移。
   ADR-0002 §6 明文要求"IMU 时间戳取采样时刻"，实现与规范相悖。
2. **CAN 侧没有时间同步**。CiA 301 的 `TIME`（COB-ID `0x100`）与对象 `0x1012`
   在 `canopen_contract.yaml` 里只是占位，Jetson 从不发、也不消费 MCU 的采样年龄。
   结果是 `/joint_states`、轮速反馈与 IMU 各说各的时钟，跨源融合只能靠"差不多同时"。
3. **协方差是占位值**。`ekf.yaml` 的 `imu0_covariance` 只有 `vyaw` 一路可信，
   数值 `0.001` 没有任何测量依据（`TODO(measured)`）。

## 决策

1. **打戳语义统一为「采样时刻」**，并在代码里固定估计方式：

   - I2C/SPI 轮询：寄存器在 ODR 节拍锁存，读到的是最近一次锁存值。
     取 `t0 = 发起读的时刻`，`stamp = t0 - 0.5/ODR`（区间中点，最大偏差 `1/ODR`）。
     1660 Hz 时最大偏差 0.6 ms，比"发布时刻"的抖动小一个量级。
   - 节点每 5 s 报 `max_sample_age_ms = now - stamp`，作为该估计的**可观测校验**；
     它若持续大于 `2/ODR + 调度余量`，说明估计模型与该硬件不符，必须重测。
   - 参数 `stamp_source` 保留 `sample`（默认）/`publish`（仅用于复现旧行为），
     `publish` 会在日志与 `/diagnostics` 里显式标注为退化模式。

2. **时钟域只有一个：Jetson 的 ROS 时钟**。MCU 不提供绝对时间，只提供
   **年龄**（`imu_age`、`cmd_age_ms`）。跨源对齐按下面的规则做，不引入第二个时基：

   | 源 | 时间信息 | 对齐方式 |
   | --- | --- | --- |
   | Jetson IMU（I2C/SPI） | 采样时刻（本 ADR §1） | 直接作为 `header.stamp` |
   | 相机 / 雷达（厂商驱动） | 驱动自己的采集时刻 | 只做**年龄**判据；跨源融合前用 `message_filters` 的近似时间 + 各自年龄上限门控 |
   | CAN 反馈（轮速/关节/IMU） | MCU 侧 `*_age`（ms） | `stamp = now - age_ms`，误差上界由 SYNC 周期（2 ms）+ 传输（<1 ms）给出 |
   | 命令（`/cmd_vel`） | 无 stamp | 以**接收时刻**计年龄（ADR-0017 的存活性判据） |

3. **CAN 侧启用 CiA 301 TIME（`0x100`）作为诊断量，不作为融合时基**。
   主站每 1 s 广播一次 `TIME`（毫秒分辨率），用途只有一个：让 MCU 的
   `imu_age/cmd_age_ms` 与 Jetson 时钟的**偏移**可被观测（进 `/rak/system_state`）。
   理由：`TIME` 只有 ms 分辨率，拿它当融合时基会引入 1 ms 量化误差，
   不如直接用 MCU 自己算的年龄。

4. **协方差必须带出处**。`ekf.yaml` 每个输入槽的协方差分三类标注：
   `datasheet`（器件噪声密度换算）/ `measured`（Allan 或直行回归）/ `placeholder`。
   只有 `measured` 允许在发布档（`jetson`）里生效；`placeholder` 由
   `scripts/timebase_gate.py --strict` 判红。

5. **测量脚本进仓库**：`scripts/calibrate_imu.py` 从 CSV/rosbag 导出计算
   Allan 偏差（角度随机游走、零偏不稳定性）与直行 2 m 的航向/横向误差回归，
   输出可直接粘贴进 `ekf.yaml` 的数值块与 `docs/perf/timebase.md` 的记录表。

## 理由

- 打戳误差是**系统性**的，滤波器无法从数据里"学"回来；它只会表现为"融合后还不如单源"。
  把打戳改到采样时刻是唯一不增加依赖的修法。
- 拒绝引入 PTP/gPTP：Orin + 两块 F407 的拓扑里没有 PTP 硬件时戳，
  软件 PTP 在有 1 kHz I2C 轮询的机器上抖动更大，收益为负。
- 用年龄而不是时钟同步：MCU 已经能算 `now - last_cmd`，把它读出来比同步两个时钟便宜且更准。

## 后果

**正面**

- IMU 时间戳误差从"毫秒级且随负载漂移"降到 `≤1/ODR`，且可被 `max_sample_age_ms` 观测。
- CAN 反馈有了统一的年龄口径，跨源融合可以与 IMU 对齐到同一个时钟域。
- 协方差的每个数字都能追到出处；占位值不再静默生效。

**负面 / 代价**

- 采样时刻是**估计**而非硬件时戳（LSM6DSR 有 DRDY 引脚，但当前未接中断）。
  真要做到硬件时戳，需要把 DRDY 接到 Jetson 并用 IIO 的 `timestamp`——
  那是下一轮的事，本 ADR 只保证"不比发布时刻更差，且误差可观测"。
- `0.5/ODR` 的偏置假设锁存相位均匀分布；若实际是固定相位（例如与 timer 同源），
  会引入固定偏置。`max_sample_age_ms` 的长期均值可以暴露这一点（期望 ≈ 0.5/ODR + 调度）。
- 多一个门禁脚本与一个测量脚本要维护。

## 被否决的方案

| 方案 | 否决理由 |
| --- | --- |
| 用 PTP/gPTP 同步 Jetson 与 MCU | F407 无硬件时戳；软件时间戳在 1 kHz I2C 负载下抖动大于收益 |
| 把 `TIME` 当融合时基 | ms 分辨率 = 1 ms 量化误差，比 MCU 自算年龄更差 |
| 在 EKF 里靠 `sensor_timeout` 吸收时间误差 | 那是丢弃策略，不是对齐策略；误差仍进入状态估计 |
| 保持发布时刻打戳，仅调大协方差 | 把系统性误差伪装成噪声，直接违反 ADR-0002 §6 |

## 待闭合项

1. **Allan 方差与直行 2 m 回归实测**：替换 `ekf.yaml` 里 `imu0_covariance[11*16]` 的
   `0.001` 与 `odom0_covariance` 的 `0.0025`，数值与命令输出一起记入
   `docs/perf/timebase.md`。
2. **CAN TIME 的主站实现**：`rak_canopen` 的 `time_cob_id` 已在契约里声明，
   主站（ros2_canopen 或过渡主站）尚未发送；偏移统计也未进 `/rak/system_state`。
3. **DRDY 硬件时戳**：接 DRDY → IIO `timestamp` → 直接作为采样时刻，
   彻底去掉 `0.5/ODR` 假设。
4. 相机/雷达驱动的采集时刻是否可信（厂商驱动多用"到达用户态"时刻），未实测。

> 本节的执行顺序、命令与判据见 [`docs/NEXT-2026-10-30.md`](../NEXT-2026-10-30.md) §6 时间基准实测。
