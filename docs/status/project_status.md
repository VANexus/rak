# 项目总体状态

> 状态：现行 · 最后更新：2026-11-18

更新日期：2026-11-18

## 2026-11-18 增量：底盘直行全链路闭合（真机验收，四轮同向 + 双传感器里程计对齐）

本轮把"前进很歪、抬起放下后乱跑"从现象追到六个独立根因并全部闭合。
每条的现场证据与复现方法在 `AGENTS.md` § Gotchas 对应条目，这里只记结论与数值。

**修复清单（主机侧）**

| # | 根因 | 修复 | 文件 |
|---|------|------|------|
| 1 | `bus.yaml` `chassis.polling: true` 让 ProxyDriver 用 SDO 轮询覆盖 TPDO 写入的 OD，ROS 侧轮位置永久冻结 | `polling: false` | `src/rak_canopen/config/bus.yaml` |
| 2 | 雷达斜装，`lidar_yaw=0.9460`（54.2°）差 90° | `-0.6561`（−37.6°，激光墙 1.02 m ↔ 深度 0.87 m 互证） | `urdf/rak_chassis.urdf.xacro` |
| 3 | 轮位置反馈没进 ros2_control（joint_states 位置 NaN → 车轮 TF 被丢 → 里程计只能积分速度） | 每关节 `aux_state_*`（0x2020.. int32，rad/count=2π/61600）+ `state_interface position` | 同上 + `rak_chassis` |
| 4 | `state_sign=-1` 与固件重复应用极性 → 直走被算成自转 | 全改 `1`（固件 `car_wheel_positions()` 已乘 `encoder_sign`） | 同上 |
| 5 | `0x2010..` 轮速反馈会冻结 | `velocity_from_position:=true` 由位置差分出速度 | `chassis_bringup.launch.py` / URDF |
| 6 | 轮命令 RPDO 事件驱动，丢一帧 OD 就停在旧值 | `command_refresh_ms=250` 强制周期重发 | `src/rak_chassis/src/canopen_bridge.cpp` |
| 7 | `wheels_radius=0.0275` 是轮盘半径，麦轮实际滚在滚子外缘 | 实测回填 `0.0309` | `config/chassis_controllers.yaml` |
| 8 | IMU 陀螺零偏是 10 月旧值，Z 轴差 0.155 °/s | 60 s 静置重标定 `0.001186,-0.007734,-0.003310` | `launch/sensors.launch.py` |

**修复清单（固件 `~/vehical_rak_motion`，openocd 实烧 Verified OK）**

| # | 根因 | 修复 |
|---|------|------|
| 1 | R38 矛盾自检撞一次墙就翻极性并落盘 | **R77**：须连续 5 拍 + 滤波转速 ≥10 counts/ms |
| 2 | `SIGN_STORE_SECTOR=FLASH_SECTOR_11`，本板 512 KB 根本没有 → 擦除永远失败、每次开机载入最早那条 | **R78**：改 `FLASH_SECTOR_7` |
| 3 | 「有电不转 2 s 清极性重学」会把车踢飞（清 0 → gate 走 `encoder_uninitialized` → 75% 开环直驱） | **R80 删除**；极性只留 Flash 预载 + kick 自学 |
| 4 | PI 实际跑在 1 kHz（`if (++win_cnt >= 25U)`），旧增益只有 0.85 %/s，爬满要 100 s | **R81** `SPEED_KI_NUM/DEN=1/2` + 按毫秒折算 + 抗饱和 |
| 5 | 摩擦前馈阈值公式写反（目标的 2% 而非 20%），静止时 `same_dir` 又判成反向 → 前馈永不触发 | **R79/R80** 相对阈值 + 静止算同向 |
| 6 | 板载 IMU 倾角按"竖装重力 +Y"写，实测平装重力 +Z → pitch −89.8° | **R78** 改 `atan2(-ax,az)`/`atan2(-ay,az)`，实测 −2.14°/−0.21° |

**主机侧新增**：`rak_imu_driver` 数据冻结看门狗（LSM6DSR 偶发掉回 power-down，I2C 仍
ACK、寄存器回读正常，只能从数据发现；连续 50 帧同值即重配 CTRL1_XL/CTRL2_G 并报 WARN。
实测 2.5 s 内触发并自愈）。

**验收数据（真机落地，墙前，雷达 + 深度双独立真值）**

| 测试 | 结果 |
|------|------|
| 0.5 m @0.12 m/s 直行 | 轮系 0.523 / 雷达 0.519 / 深度 0.525 m，**轮系/雷达 = 1.007** |
| 四轮转角 | +17.44/+16.53/+16.63/+17.04 rad，同号、极差 5.3%（直线，非自转） |
| 速度稳态跟踪 | 0.05 m/s → 94–107%；0.10 m/s → 98%（轮系）/88%（雷达） |
| 航向漂移 | 稳定器关 **−2.0 °/m** → 开（`stabilizer:=true`，`cmd_flags.bit1` 实测 0x3）**+0.42 °/m** |
| 极性持久化 | Flash `seq=1 signs=(-1,1,1,-1)`，kick 自学结果一致，`reset run` 后自动载入 |
| 雷达/深度一致性 | 前方墙：雷达 1.02 m ↔ 深度 0.87 m（同一面墙，装位差）；行驶中两路差 <5% |

**已知未闭合（诚实记录）**

- rf2o 位姿与航向不自洽（位移方向 +37.9°≈lidar_yaw 量值，航向差 177°；航向变化量准）。
  接 EKF 只可取位姿、禁 yaw —— `ekf.yaml` 已按此设计，但 **EKF 真机融合尚未验收**。
- `0x2010..` 轮速反馈仍会在空闲时读到陈旧值（已由位置差分绕开，根因在固件上报路径，未修）。
- `/chassis/rpdo` 话题 ~77 Hz 而线上 99 Hz（best_effort KEEP_LAST(1) + 驱动 drop 路径），不影响功能。
- 板载 IMU 的 yaw 只用于固件稳定器；与 Jetson IMU 的航向互校还没做。
- 打滑检测（DOB/滑移降权）只有固件骨架（`SLIP_*`），主机侧 `slip_watch` 未接真机。

## 2026-11-15 增量：4D 融合视觉导航 v2.0（ADR-0027，软件门禁全绿 / 真机未验证）

一组"复用现成库 + 接线 + 门禁"的改动，落在 `src/rak_bringup`，设计见
`docs/design/vision_nav_v2.md`：

- **EKF 四路**：`config/ekf_4d.yaml`（轮式差分 / rf2o 绝对 / VO 差分 / IMU vyaw），
  `slam.launch.py ekf_params:=` 透传，默认仍指 v1 的 `ekf.yaml`。
- **VIO**：`launch/vio.launch.py`（`rtabmap_odom` rgbd|stereo → `/odometry/vio`，无 TF）。
- **4D 代价层**：`config/nav2_4d.yaml`，`rtabmap_costmap_plugins::VoxelLayer`
  （局部 decay 5 s / 全局持久），三源 `/scan` + `/camera/depth/points` + `/stereo/points`。
- **全向寻路**：`SmacPlannerLattice` + 上游 omni 原语；`MPPI` 权重不动。
- **门禁**：`test_4d_nav_config.py` 8 项离线契约；`scripts/verify_4d_fusion.py` 6 道运行时门
  （2026-11-15 本机 ALL PASSED：G3 合成障碍标 1549 格、G5 rtabmap VO 出 `/odometry/vio`、
  G6 `ComputePathToPose` SUCCEEDED 且路径 202 点）。

**实测否掉的**：`spatio_temporal_voxel_layer` 2.5.5 在同一份合成数据上不标记（max_cost=0）；
`SmacPlannerHybrid` 1.3.13 不接受 `Omni`；rcl 参数解析要求协方差单行 flat + 全浮点。

**未验证**：深度相机/雷达/双目均未上机；EKF 协方差仍是占位值；`/stereo/points` 等双目标定。

更新日期：2026-11-13

## 2026-11-13 增量：新机器基线与工具链闭合（无功能变更）

这一轮**没有加功能**，做的是"让这台机器从'能开机'变成'能当机器人核心'"，
以及把长期带红的测试套件修绿。逐条都可在本机复现。

### 机器侧（新增 `scripts/system/rak-system-baseline.sh`，详见 `docs/reference/machine_baseline.md`）

- **SSH 只允许公钥**：`sshd -T` 实测 `passwordauthentication no`、
  `authenticationmethods publickey`、`permitrootlogin no`、`allowusers xrak`。
  两把 ed25519（`rak.pub`/`xrak.pub`）；镜像自带的那把 `ssh-rsa` 已移除，
  备份在 `~/.ssh/authorized_keys.bak-20261113`。
  实测 `ssh -o PubkeyAuthentication=no ...` → `Permission denied (publickey)`。
- **CMA 从 1.3 MiB 恢复到 ~49 MiB**（256 MiB 池）：gdm 停止并 mask。桌面把池子吃干后
  `VIDIOC_REQBUFS` 失败，表现为"相机坏了"。
- **`jetson_clocks` 锁频**：`scaling_min_freq == scaling_max_freq == 1510 MHz`，
  GPU 624.75 MHz、EMC 2133 MHz。新增 `rak-clocks.service`。
- **修掉开机一条红**：JetPack 自带 `nvpmodel.service` 在"已是 10W"时仍要求交互确认，
  systemd 下无 stdin → `status=234`。换成非交互的 `rak-nvpmodel.sh`。
- **停用 20+ 冗余服务**（打印、桌面设施、cloud-init 四单元、rsyslog、apport、
  lttng、openvpn 等），理由逐条写在基线脚本里。
  **明确保留** `docker`/`containerd`（容器化开发）、`bluetooth`（外设）、
  `avahi-daemon`（`<host>.local` 互访）——它们与 `netclient` 一起进了
  `PROTECTED_UNITS`，写回禁用清单会被脚本拒绝。
- `limits.d`（rtprio 95 / memlock unlimited）、journald 封顶 256 MiB、
  关闭 unattended-upgrades、启用 fstrim。

### 仓库侧（修的都是"测试从来没跑"和"文档说 A 代码是 B"）

- **`package.xml` 里 13 个包把 `ament_python` 写成 `<buildtool_depend>`** ——
  这个键没有 rosdep 定义，导致 `rosdep install` 每次都以非零退出码结束（看起来像
  依赖装失败，实际大部分都装上了）。`ament_python` 只应出现在
  `<export><build_type>`。
- **`rak_lifecycle` / `rak_canopen` 缺 `extras_require={"test": ["pytest"]}`** ——
  `colcon test` 报 "NO TESTS RAN" + exit 5。看起来像测试失败，其实是
  **测试从来没跑过**。补上后两个包分别跑出 46 项和 20 项。
- **URDF 单段 vs C++ 双插件不一致**：C++ 侧 `RakChassisHardware`/`RakArmHardware`
  与 12 项 gtest 早已落地并全绿，`rak_chassis.urdf.xacro` 却还是单段
  `RakSystemHardware`。按 ADR-0019 拆成两段后 `validate_chassis_urdf.py` 全绿。
- **`slip_watch` 是 managed 节点但不在清单里**：`rak_metrics.launch.py` 用 `managed()`
  起它，清单却没有对应条目 → 没有域声明"谁激活它"。已补进 observability 域。
  这正是 ADR-0020 §3 点名的"运行中、看着配好了、完全惰性"。
- **`build_workspace.py` 的 CUDA 检测永远为假**：
  `Path("/usr/local/cuda/bin").with_name("nvcc")` 得到的是 `/usr/local/cuda/nvcc`
  （`with_name` 替换最后一段，不是追加），于是 `CUDACXX`/`PATH` 从未被设置，
  `find_package(CUDAToolkit)` 反推出的根是 `/usr/local`，找不到 `libcudart`。
  修正后 GPU 包正常构建。
- **`build_workspace.py` 的门禁看不见自己刚构建的东西**：不 source 就直接跑，
  门禁报 `package not found, searching: ['/opt/ros/jazzy']`。改为脚本自己拼
  `AMENT_PREFIX_PATH`/`PYTHONPATH`/`LD_LIBRARY_PATH`。
- **`rak_astra_driver` 硬编码 OrbbecSDK 1.10.18 路径**：新机器上只有 apt 的 2.9.3，
  构建直接 FATAL。改为按优先级自动发现（`-DOrbbecSDK_DIR` → `/opt/OrbbecSDK_*` →
  仓库历史 `tools/camera` → `/opt/ros/$ROS_DISTRO`），RUNPATH 跟着实际路径走。
- **`rak_road_perception` 在模块顶层 `import torch`**：torch 在 JetPack 上不是必然
  存在（apt 无 arm64 候选），顶层导入让"可选推理依赖"变成"包坏了"。改为延迟导入，
  与本仓其余三个用 torch 的包一致。现在不装 torch 也能 `import`。
- **测试自身的缺陷**（不是被测代码的问题）：
  `test_system_manifest.py` 拿夹具清单（只声明 1 个 managed 组件）去校验整个 src/
  的所有 launch，于是真机清单里的受管节点全被误判；`test_tts_layers.py` 要求
  `tools/qwentts.cpp` 存在，而 `tools/` 是 gitignored 本地目录；`test_asr.py` 用
  `__new__` 绕过 `__init__` 却没设 `_engine`。三处都已修正。
- **`rak_stereo_driver` 的 uncrustify 关掉**：仓库没有任何格式化器配置
  （CLAUDE.md 明写），现有 C++ 统一用"大括号同行"，而 ament_uncrustify 默认要 Allman。
  这不是"代码不合规"，是"没有约定"。保留 cpplint（查可移植性/正确性，当前全绿）。
- 清理：`.claude/worktrees/*` 曾被误提交为 **gitlink**（mode 160000，子模块指针），
  工作树里只是空目录 → fresh clone 后永远"脏"。已从索引移除并写进 `.gitignore`；
  `.superpowers/` 过程日志一并移除。

### 实测结果（本机 Orin，2026-11-13）

| 项 | 结果 |
|---|---|
| `build_workspace.py --profile jetson` | 30 个包构建成功 |
| `check_workspace.py --installed` | **27 个自有包**，依赖图与 launch 引用全绿 |
| `colcon test`（15 个包） | **216 项，0 失败，3 跳过**（此前 186 项、8 失败） |
| `rosdep install` | EXIT=0（"All required rosdeps installed successfully"） |

**仍未闭合**（不要当成已完成）：CANopen v5 真机链路（ADR-0024 主站未实机验证）、
EKF 协方差标定、轮径/lx/ly 实测回填、臂 J2–J6 运动与限位验证、
`rak_wbc` 的 2 项 HQP 数值测试（2026-10-29 已记录的既有失败，本轮未动）。

## 2026-10-29 增量：运行期编排与受管生命周期（ADR-0020）+ v5 事实核对（ADR-0021）

**先说影响最大的一条**：下位机已按 ADR-0021 切到 **v5 = CANopen**（在板 R73/A14），而本仓
`src/rak_chassis` 的 `CanSession`/`can_decode` 仍说 v4 帧 → **真机链路当前不可用**，
台架只能用伞仓 `tools/canopen_bringup.py` 验下位机；生产主站是待办 **P3（lely-core）**。
本轮**没有**做链路迁移，只做了运行期编排层。

新增 `src/rak_lifecycle`（基础设施包，不依赖任何自有包）+ `rak_bringup` 编排接线：

- 清单 = 故障域/组件/所有权/判据的唯一来源；`system_manager` 按依赖闭包拉起、稳态按新鲜度降级，
  **只降级自己拥有的组件**；`rak_system status|start|stop|recover` 是操作员入口。
- 真机清单 + **软件夹具清单**两套（`profile:=fixture` 不碰硬件，专用于验证编排本身）。
- 转换了两个自有节点：`rak_task_executor`（未激活没有 action server；配置失败从崩进程变成
  可读 FAULTED）与 `rak_wbc`（configure=复合 URDF+HQP，activate=100 Hz 环；停用不补发零速，
  停车靠 `mecanum_drive_controller.reference_timeout` 与臂 lease）。
- 阈值有出处：`/imu/data_raw` 903.6 Hz、`/odometry/filtered` 30.0 Hz、`/tf` 83.3 Hz
  是 2026-10-29 在本机 `ros2 topic hz` 实测（清单里标 `basis: measured`）；雷达 10 Hz 仍标
  `nominal`（现场那套栈 `lidar:=false`，没测到）。

**实测过的**（Orin 独立 worktree `~/rak-lifecycle`，`ROS_DOMAIN_ID=77`，全程未触碰硬件与
现场那棵有未提交在制品的工作树）：23 包 `colcon build` 成功；`rak_lifecycle` **27/27**、
`rak_task_executor` **4/4**；9 个新增/修改 launch `--show-args` 全绿；结构门禁
`check_workspace.py`（非 installed 模式）23 包通过。

**未跑/未验证**（本机→Orin 链路中断：`orin.xrak` 不解析、100.121.213.20 无路由）：
全量 `check_workspace.py --installed`、`colcon test --packages-select rak_bringup`
（清单↔launch 一致性门禁）、`profile:=fixture` 的实机编排演示（拉起/降级/单域停用）。
这三项跑绿之前，编排层**不得称"已验证"**。

**已知先存在性问题**（不是本轮引入，已在未改动的现场树上复现同样失败）：
`rak_wbc/test/test_hqp_numerical.py` 的 `test_support_cbf_locks_com_when_margin_consumed` 与
`test_solver_matches_true_optimum_across_objective_change` 2 项失败。

## 2026-10-29 续：CANopen v5 的 Jetson 侧契约层（ADR-0022，离线可验证部分已落地）

新包 `src/rak_canopen`（基础设施，不依赖任何自有包；24 包结构门禁已过）：

- **不重写协议**：生产主站是 lely-core（ADR-0021 已选）。本层把下位机 SSOT
  （`config/chassis.json` / `arm.json`，副本 + sha256 pin）派生成可执行事实：
  COB-ID、PDO 字节布局与 8 字节上限、偏移、速率（由 `tpdo_types × SYNC 2 ms` 算出：
  底盘 100 Hz，臂 50/100/10 Hz）、量化范围与越界行为、build_tag SDO 解码、命令活性三条件。
- **单位/因子单独一张表且每条带出处**（SSOT 里没有这些信息——这正是 v4 量错 10 倍的土壤）；
  语义名与 JSON 双向闭合，多/少/拼错都装载失败。
- **"禁止同总线混跑 v4/v5" 现在是算术**：v4 的 ID 从本仓 v4 后端源码正则读出，与派生 v5 ID 求交集，
  实测冲突 **0x201 / 0x202**（与规范件的说法对上）。
- **EKF 第一次有显式协方差**（此前 `grep -c covariance` = 0）+ `print_diagnostics: true`；
  协方差按 `*_config` 融合位生成并被测试钉住。过程中抓到两处真错：PyYAML 把 `1e6`/`1e+06`
  读成**字符串**、以及 `imu0_config` 开 vyaw 而方差却写在 yaw 位。
- 已知不自洽**未擅自改**、记录待裁定：`odom0` 位姿与速度同开 vs ADR-0002 约束 #3；
  `i2c_imu.py:240` 用发布时刻而非采样时刻（他人在制品，本轮不动）。
- 实测：`rak_canopen` 契约测试 **16/16**、`rak_bringup` EKF 契约 **7/7**（本机离线，
  无需 ROS/硬件/Orin）。**字节序 `little` 仍是推断**，明日首帧必须核对；
  实机项见 ADR-0022「明日实机门」。

下一步顺序（按用户定案）：先把本层收尾合入 → 再做 P3 CANopen 主站（届时 `link` 域整段重声明，
拥有者改为 v5 主站自带的网络管理，遵守 ADR-0020 §3 单一所有者）。

---

## 历史（2026-10-15 及更早）

## 2026-10-15 增量：机械臂几何并入整车 URDF + MoveIt 真机档

- **几何单点化**：臂的运动链（7 link + 6 joint）抽到
  `src/rak_description/urdf/macros/arm_chain.xacro` 的 `<xacro:rak_arm_chain/>` 宏，
  由 `rak_arm.urdf.xacro`（MoveIt 用）与 `rak_chassis.urdf.xacro`（真机 CM 用）
  **共用**。此前两边各写一份：整车那份是零长度控制桩，没有几何。
- **RSP 现在会发臂的 TF**：ADR-0015 待闭合项 #4 随之闭合。本机实测
  `tf2` 查 `arm_base_link→tool0` = (0.10, 0, 0.33)（Home 位）。
  → RViz / 任何外部消费者这才**看得到**机械臂（rviz 的 RobotModel 是逐 link 查 TF 的）。
- **契约不变**：`<ros2_control>` 仍是 11 关节（4 轮 + j1..j6 + gripper）、顺序与接口
  种类不变；臂的 `<limit>` 与 command min/max 改为引用 `properties_arm.xacro`
  的 `${jN_lower/upper}`（消掉 12 个魔法数字），有效限幅恒等于契约值。
  新增 `src/rak_chassis/test/validate_chassis_urdf.py` 把整套契约钉进 `colcon test`。
- **夹爪**：`tool0` 之后补了一个**无几何**的 `gripper_link` 帧（`gripper_joint`，
  revolute，限位与旧桩同值）——只为满足 component_parser 的"joint 必须在树里"，
  **不编造未实测的几何**（夹爪支架没量过）。
- **MoveIt 真机档**：`rak_moveit_config/launch/rak_arm.launch.py` 新增
  `use_hardware`（默认 **false**）。`true` 时**不再起**本地
  `controller_manager` / spawner / `robot_state_publisher`，只起 `move_group`(+RViz)，
  对接真机 `chassis_bringup` 起的同名 `arm_controller`。
  本机用 mock 替身验证：规划 69 点、Execute `error_code=0`、零
  "action client not connected"。仿真档（默认）行为不变。
- **仍未做**：真机的 J2–J6 运动/限位/急停验证（Arm Unlock Gates，未解除）；
  臂末端**没有急停**（契约 §2.5：急停只作用于轮驱动）；舵机无编码器，
  反馈是指令回声，MoveIt 无法知道真机是否真的到位。

## 2026-10-14 增量：删除 USB v2 链路（ADR-0016）

- **传输**：CAN v4（SocketCAN `can0`）成为唯一传输；`transport`/`device` 参数与
  `SerialSession`、`rak_base_driver`、`rak_serial_link`、直连硬件的示教/驾驶 TUI、
  6 个串口脚本与 `docs/protocol/` 全部删除（删除前快照：git tag `pre-usb-removal-20261013`）。
- **入口**：`task_navigation.launch.py` 的 `motion_backend` 只剩 `sim|stub`
  （`hardware` 选项删除）；真底盘经 `chassis_bringup.launch.py` →
  `mecanum_drive_controller` → `rak_chassis` → `can0`（`allow_motion` 默认 false）。
  基础层解除服务由 `/base_uart_driver/arm` 改名 `/base/arm`。
- **工具原则**：工具只经 topic/action/service 与驱动节点对话，禁止直连硬件。
- **链路验收（2026-10-13）**：H/L 对调后 `scripts/can_link_smoke.py can0` 连续
  10/10 通过（BAT≈12500 mV、FAULT=0、BUSOFF=0）。

## 2026-10-11 增量：全机坐标系分层（ADR-0015）

- **坐标系**：确立全机 TF 树 `world → map → odom → base_footprint → base_link → {4 轮, laser_link, mic_array_link, camera_link, imu_link, arm_base_link}`；采用 REP-103 + REP-120，`world` 作无限开放世界的无界全局根（现为静态恒等占位）。
- **几何**：车体实测轮距 27 cm / 轴距 25 cm 入 `rak_chassis.urdf.xacro`；雷达（前·中轴）、6 阵列麦克风（雷达上方）、深度相机（车头最前）、机械臂（后·中轴）挂载帧同处定义，高度/前后距离以 `TODO(measured)` 占位。
- **发布者收敛**：静态几何单一来源交给 `robot_state_publisher`；`slam_toolbox` 独占 `map→odom`，EKF 独占 `odom→base_footprint`；移除 launch 里 `base_link→laser_link`/`imu_link` 的临时静态 TF。
- **本机验证边界**：x86/IDE 侧已过 `xacro` 解析与 `check_workspace.py` 结构检查；`colcon build` 与上车 TF 复测在 Jetson 收口（见 TODO P1-6）。

## 2026-10-09 增量：CAN v4 主链路 + 工作空间重组（ADR-0012 / ADR-0013）

- **传输层**：合入上游 `Session` 抽象 + `CanSession`（SocketCAN CAN v4 主）
  后端；协议事实源升到下位仓 v4。取代 ADR-0007 的
  "USB 唯一链路"（**USB 备用链路部分已按 ADR-0016 于 2026-10-14 删除，见上**）。
- **工作空间**：按子系统归堆 23 → 18 构建包：`rak_hardware`→`rak_chassis`、新建
  `rak_commissioning`（双 TUI + chassis_key_test 转正 entry point）与 `rak_intent`
  （task_intent 自 brain 抽出）；`rak_bringup` 吸收 sensor_bringup/navigation/
  chassis_bringup 全部 launch/config；`brain/` 迁入 `src/rak_agent`（模块 `rak_ai`→
  `rak_agent`，语料迁仓库根 `data/episodes/`）；CAN v3 栈归档 `attic/`（2026-10-10 本地
  归档已删，仅存 git 历史）。
- **规范守护**：`check_workspace.py` 重写（EXPECTED 18 包、依赖倒置守卫
  `dependents(rak_agent) ⊆ {rak_bringup}`、launch 引用检查），本机结构检查全绿；
  并由此暴露并修复历史欠账 `nav2_lifecycle_manager` 未声明。
- **文档**：ADR-0012/0013、索引、README/architecture/AGENTS/CLAUDE/CONSTRUCTION/
  migration（新映射表）/TODO 同步完成。
- **本机验证边界**：x86 开发机（无 Jazzy/JetPack）——已跑通纯 Python 测试
  （`test_ai.py`、`test_task_intent.py`、模块导入与语料路径断言）；
  `colcon build/test` 与 `check_workspace --installed` 需在 Jetson 上首次收口
  （先 `rm -rf build/architecture install/architecture`）。

## 结论

项目已经从包重命名和依赖整理，推进到一个可运行的软件任务闭环：可以用文字提交已配置地点任务，在 fake Nav2 上验证导航成功、暂停、恢复和取消；CAN v4 链路已通过冒烟验收（2026-10-13），六轴机械臂动作夹具也有软件测试。

当前交付仍是**软件原型，不是可直接上车运行的整机系统**。真实传感器 Nav2 组合、collision monitor 停车效果、F407 固件互操作、物理急停和机械臂硬件执行都没有完成验收。Jev 与任务执行器之间的共享任务仲裁**已在 ADR-0010 中定义**（新包 `src/rak_harness` + 传递式提案缝），但 Harness 本体尚未实现，RAI 慢路径仍不是 Harness 持续任务系统。

## 能力状态

| 领域 | 当前状态 | 尚未证明 |
|---|---|---|
| ROS 包架构 | 统一 `rak_` 包迁移完成；依赖边界和安装态检查已覆盖 | 后续包边界变化仍需按架构文档评审 |
| 文字导航任务 | 本地解析、配置地点白名单、任务 Action/状态、fake Nav2 成功/暂停/恢复/取消已实现 | 到达与暂停竞态、各类超时、迟到慢规划结果、重启策略仍需补测 |
| RAI 慢规划 | 可选路径只能选配置的 waypoint 别名 | 不等同于 Laya/Jev System 1，也没有 Harness 的持续任务调度（ADR-0010 已定义缝，未实现） |
| Nav2 | Omni MPPI、速度平滑器、collision monitor 和末端适配器配置已接入 | 真实传感器/定位下的 lifecycle、碰撞停止行为、速度链单写者尚未端到端验证 |
| 底盘链路 | CAN v4（SocketCAN）冒烟验收通过（2026-10-13：`can_link_smoke.py can0` 10/10，BAT≈12500 mV / FAULT=0 / BUSOFF=0）；`rak_chassis` 导出 4 轮接口，底盘冒烟过（固件 R37） | 真实 encoder odom、E-stop、时序与整车验收未完成；CAN 侧 7 轴位置接口未实现（ADR-0012/0016 待闭合项） |
| 硬件启动路径 | 真底盘经 `chassis_bringup.launch.py` → `mecanum_drive_controller` → `rak_chassis` → CAN；`task_navigation.launch.py` 的 `motion_backend` 只有 `sim\|stub` | 尚未在真实传感器或 F407 上完成整车运行验收；`allow_motion` 默认 false 拒绝运动 |
| 机械臂 | 六关节加独立夹爪的软件动作夹具和 MoveIt 配置已实现 | 模型、限位是测试占位；F407 仅有六路 PWM、无关节反馈，夹爪通道和硬件控制器未完成 |
| 感知与语音 | 雷达/深度相机已有驱动或测试基础；语音链已通（2026-10-10：XFM-DP → `audio_capture` → `/audio` → Vosk 转写/意图；2026-10-11：板端唤醒/方位 `vtn_node` → `/speech/wake`、离线 TTS `tts_node` → `/audio_out` → `audio_play` → USB 喇叭，电气链路与真人唤醒均已实测）；当前仍以文字入口验证任务 | 传感器融合和完整导航输入链未验收；阵列声学段需真人说话验收；识别解码 0.84–1.01× 实时，余量不足；TTS 合成约 0.75× 实时（kokoro 24 kHz），播报音色/音量待真人验收；`/speech/say` 尚无决策层消费者 |
| Jev / Harness | `rak_agent` 保留现有 S1/S2 原型与冻结的数据契约；TensorRT decider 已接入；`src/rak_harness` + `harness_node` 已实现（2026-10-15 软件闭环）：`/harness/request|proposal|event` 契约、`slow_planner:=harness`、唤醒门控语音对话环（提交任务/pause-resume-cancel/播报结果）、人格卡+车况快照、云端 client stub（无 key 时本地保底），fake-Nav2 隔离域验证通过 | 云端通路未实测（无 `ANTHROPIC_API_KEY`）；S2 回复仍不驱动执行器；持续方位流、打断、流式 TTS 未做；表达层仍是门控 no-op stub |

## 已有验证证据

本次收尾验证结果：

- 全工作区构建 17 个包完成；安装态 checker 检查 14 个自有包、依赖图和 13 个 launch。
- CTest 21 项通过；直接 Python ROS 集成测试中，任务/机械臂 4 项通过。
- `src/rak_agent/test/test_ai.py` 与 `src/rak_intent/test/test_task_intent.py` 通过。
- fake Nav2 覆盖导航成功、重复 request ID、暂停确认、恢复重规划、Service 取消和 Action 取消。
- 六轴动作测试覆盖六关节轨迹与独立夹爪 Action。

以上都是软件构建、静态 launch 检查和仿真结果，不代表真实传感器、F407 固件、
底盘运动、物理急停或碰撞停止距离通过验收。

## 硬件门控现状

- `chassis_bringup.launch.py` 的 `allow_motion` 默认 **false**：硬件只握手+遥测、
  只发零目标（DISARMED）；`can_channel` 默认 `can0`，接口身份须由硬件负责人确认。
- 控制器命令限幅由 `controller_manager` 强制（`enforce_command_limits: true`）；
  轮几何与限速当前是占位值，实测替换前不得提速。
- `task_navigation.launch.py` 的 `motion_backend` 只有 `sim|stub`，不证明 F407 固件
  兼容或提供安全保证；真底盘只能经 `chassis_bringup.launch.py` 走 CAN（ADR-0016）。
- 物理底盘保持锁定，直到完成 [硬件验收门槛](../reference/hardware_acceptance.md)；六轴手臂继续只用软件夹具。

## 建议的下一步顺序

1. 在非硬件后端验证传感器/Nav2 组合：确认 lifecycle、定位输入、collision monitor 输出和
   `/cmd_vel` 单写者；不能用 fake 遥测替代真实反馈验收。
2. 补足任务状态机的暂停-到达竞态、goal response/任务超时、慢规划迟到回复和重启测试。
3. 完成 CAN 侧 7 轴位置接口 + arm controller + 夹爪轴（ADR-0012/0016 待闭合项），
   再按硬件验收文档逐阶段测试。
4. 获得机械臂真实几何、关节限位、反馈与 PWM 通道标定后，再解锁真实 arm driver；届时再连接任务层与
   Jev/Harness 仲裁，并规划语音接入。
5. 按 ADR-0010 实现 `src/rak_harness`（软件可仿真，不依赖硬件门）：先落 `harness_node` +
   契约 + `NoOpExpressionSink`，再实测 DeepSeek Anthropic 端点与 `anthropic` SDK 安装，
   最后补 `/harness/proposal` 的陈旧/过期/预算竞态测试。

在这些门槛完成前，推荐使用 task simulation 和 CAN 链路冒烟（`scripts/can_link_smoke.py`）做软件演示；不要执行真实底盘或机械臂动作。

## 2026-10-30 工程化阶段收口（第二轮）

本轮（2026-10-29/30）把四条结构问题收敛成正式档，全部有 ADR 与门禁背书：

- **组装层拆分**（ADR-0026）：`rak_orchestration`（契约，纯 x86 可建）+ `rak_bringup`（硬件全栈）。
  档位是数据（`build_profiles.yaml` + `launch_contracts.yaml`），
  `python3 scripts/build_workspace.py --profile desktop|jetson` 一条命令。
- **生命周期全覆盖**（ADR-0020 修订 d）：自有节点全部 managed（25/36 组件），
  三个驱动初始化顺序缺陷同批修掉；launch 统一走 `managed()` 助手，`supervised` 全链贯通。
- **时间基准**（ADR-0023）：IMU 采样时刻打戳（含自校验 `max_sample_age_ms`）；
  测量工具 `scripts/calibrate_imu.py`、占位值门禁 `scripts/timebase_gate.py`。
- **观测**（ADR-0020 §9 的观测半边）：新包 `rak_metrics`（p50/p95/p99/丢帧 + `/rak/metrics` + HTTP）
  与 rosbag 录制白名单 `scripts/record_session.py`。
- **CANopen v5 主站选型**（ADR-0024）：ros2_canopen（Jazzy），配置/launch/首帧核对工具已就位；
  **未编译未实机验证**——这是下一步的第一优先级。
- **行为层**（ADR-0025）：只出两级架构与迁移路径（BT.CPP 技能层 / SMACC2 任务层），代码未交付。
- **下位机能力规格**：`docs/reference/firmware_capability_spec.md`——四路里程计的职责边界、
  EKF/Nav2 的八条隐性需求、固件补齐清单（F1–F10，含电池上报/EMCY 分类/参数持久化）。

**未验证清单（下次上机照 `docs/status/next_2026-10-30.md` 逐条证伪）**：
C++ lifecycle 改造未编译；ros2_canopen 未安装；managed 全栈未真机激活；字节序待首帧；
EKF 协方差待 Allan/直行实测。

**环境事实**：桌面 Windows 机无 ROS/Tailscale，无法直达 Orin；本轮所有验证限于
纯 Python 门禁（清单↔launch 一致性、档位契约、家目录扫描——全部绿）。
