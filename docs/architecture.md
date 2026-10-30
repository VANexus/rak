# 分层具身智能架构

状态：统一包迁移与软件任务闭环已实现。文字任务、fake Nav2
暂停/恢复/取消以及隔离六轴动作夹具均有软件实现。CAN v4（SocketCAN）链路已通过
冒烟验收（2026-10-13，`scripts/can_link_smoke.py can0` 10/10；USB v2 链路已按
ADR-0016 删除），但传感器 Nav2 和整车安全验收仍未完成。README 是工程入口，当前进度汇总见
[项目总体状态](project_status.md)。

## 1. 目标与硬件事实

第一条产品闭环：对话下达导航任务，移动过程中可打断，并在重新验证条件后恢复。
最终扩展到机械臂操作、持续任务理解和自然表达。

- 上位机：Jetson Orin Nano，ROS 2 Jazzy，JetPack 7.2。
- 已有雷达和深度相机测试代码，不能等同于完成工程化接入。
- 底盘：F407 自行完成麦轮运动学与四轮 25 kHz 闭环，上位机给 `(vx, vy, wz)`。
- 底盘有独立 IMU 做稳定，但是否提供 ROS 可消费的 IMU/协方差/时间戳还需对接。
- 机械臂目标：六个定位关节，夹爪独立。当前旧 URDF 是五轴演示，禁止当作实物模型。
- 语音 ASR 已接入硬件（2026-10-10：讯飞 XFM-DP 六麦阵列 → `audio_capture` →
  `/audio` → Vosk 转写，见 `docs/microphone_array.md`）；2026-10-11 起板端
  唤醒/方位（`vtn_node` → `/speech/wake`）与离线 TTS 播报（`tts_node` →
  `/audio_out` → `audio_play` → USB 喇叭）也已电气链路实测；任务控制
  仍以文字事件验证，声学段需真人说话验收。
- 叶节点旧协议记录不能证明新的六轴机械臂已经具备相应通道、反馈和限位。

> **2026-10-08 增补（定义性约束，全文适用）**：本机是**手搓平台**——无 CAD 级几何/质量，
> 机械臂是**普通 PWM 位置舵机**（位置指令、**无关节位置反馈**、机械带宽几 Hz），底盘几何与
> 臂质量仍多为占位值。**这不是待克服的缺陷，而是架构的定义性约束**：正因硬件不完美，
> 每一层必须能**独立降级**——HQP 失败→零速+臂保持；lease/E-stop 在 F407 独立兜底；
> Harness 永不赋权；需要精度的地方（末端）用视觉闭环"买精度"，而不是赌模型精度。
> 由此：**物理数字孪生（Gazebo/Isaac Sim 级）不适用也不必要**；替代物是"**软件夹具**"
> （fake controllers：验证契约/时序/状态机，不管物理保真）+ **廉价实测标定** + **台架阶梯**。
> 本节上方部分条目（底盘 IK 归属、五轴 URDF、v2 未接入等）是 10-04 口径，已被
> ADR-0007/0008 与 2026-10-08 实机标定/烟测取代；以 `docs/adr/README.md` 索引为准。

系统采用分层选择和技能执行，追求类似 VLA 的交互体验，但当前 Laya 是文本选择模型，
不是端到端视觉动作模型。感知、几何规划、执行和反馈不能由“选择一个标签”替代。

## 2. 包设计原则

1. 自有包使用 `rak_` 前缀、snake_case 和职责名；不要求上游 vendor 改名。
2. 包是构建、依赖、部署和测试边界，不是每个类或每个节点一个包。
3. `*_driver` 面向设备；`*_description` 表达物理结构；
   `*_config` 表达工具栈配置；`*_bringup` 组合启动，不放决策算法。
4. 消息独立于 GPU、模型和厂商 SDK。底层包不能依赖整机 bringup。
5. 用 ament index 找安装资源，禁止依赖当前工作目录或旧源码绝对路径。
6. 节点名、话题名、ROS 包名、Python 模块名不是同一个概念；
   本次统一包名，保留已有话题、运行节点名和数据契约。
7. 未实现能力先定义设计与验收，不为“看起来完整”创建空包。

参考：
- [ROS 2 Jazzy ament CMake](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html)
- [ROS 包命名 REP-144](https://www.ros.org/reps/rep-0144.html)
- [MoveIt 独立配置包](https://moveit.picknik.ai/main/doc/how_to_guides/moveit_configuration/moveit_configuration_tutorial.html)

上述参考提供机制与约定，不意味着 ROS 官方规定必须使用本项目的这些包名。
MoveIt 文档页面跟随主线；实际 API 仍以本机 Jazzy 安装版本验证。

## 3. 当前包边界

| 包 | 拥有的内容 | 不应承担 |
|---|---|---|
| `rak_interfaces` | 感知/决策消息、任务 Action/Service、底盘遥测 | 模型、CUDA、总线私有帧格式 |
| `rak_gpu` | GPU RAII、TRT/VPI 工具 | 消息生成、机器人策略 |
| `rak_stereo_driver` | V4L2、nvJPEG、图像发布 | 目标识别、导航目标选择 |
| `rak_astra_driver` | Astra SDK 与 ROS 数据转换 | 世界状态、抓取规划 |
| `rak_pointcloud` | GPU 点云节点 | 感知策略、任务规划 |
| `rak_grasp_perception` | RGB-D 快照、离线候选、RViz 选择、只规划的六轴夹具预览 | 实体执行、在线模型服务、夹持力推断 |
| `rak_description` | URDF、几何、惯量与可视化 | 导航参数、模型策略 |
| `rak_intent` | 共享版本化文本→意图契约（stdlib-only） | 决策、动作执行、RAI 会话 |
| `rak_task_executor` | 单一任务状态机、Nav2 Action 生命周期；意图解析来自 `rak_intent` | 原始底盘/关节命令、感知算法 |
| `rak_speech` | 离线 ASR（Vosk）：输入为 audio_common `/audio`（`audio_capture` 采阵列）或 arecord 直采 → 转写 + 意图发布；板端唤醒/方位桥（`vtn_node` → `/speech/wake`）；离线 TTS（`tts_node` → `/audio_out` → `audio_play`） | 下发目标、速度/关节命令 |
| `rak_wbc`（ADR-0011） | 全身运动仲裁：100 Hz 速度级 HQP（Pinocchio + OSQP）、底盘与臂的单一常规写入者；数值层桌面可回归 | 安全裁决、任务生命周期、直接读感知/决策 |
| `rak_chassis` | 两个 `ros2_control` SystemInterface：`RakChassisHardware`（4 轮，0x030/0x200-202/0x220/0x032）与 `RakArmHardware`（6 定位轴 + 夹爪，0x031/0x210/0x211/0x221/0x222）；两者共享同一个 `CanSession`（`can_session_pool`），CAN v4 唯一传输（ADR-0012/0016/0019） | 运动学仲裁、轮/臂语义重定义、上层传输分支 |
| `rak_commissioning` | `chassis_key_test` + `chassis_test.launch.py` | 生产运动接口、安全认证 |
| `rak_arm_simulation` | 软件轨迹与夹爪 Action server、估算关节状态 | 物理仿真、真实硬件接口 |
| `rak_moveit_config` | 旧 demo 与独立六轴仿真配置 | 电机闭环、实机几何声明 |
| `rak_motion_stub` | 明确标注的测试转发/运动估算 | 硬件驱动、安全保证 |
| `rak_agent` | 当前 S1/S2 认知原型 | 电机驱动、同步等待云端后控制 |
| `rak_harness`（**待创建**，ADR-0010） | Harness 慢路径：任务上下文、工具白名单、预算/超时/事件摘要/失败恢复；独立 `anthropic` 客户端 | 速度/关节命令、第二生命周期所有者、实时安全裁决 |
| `rak_orchestration` | **组装契约层**（ADR-0026）：系统清单（故障域/所有权/判据）、构建档位（desktop/jetson）、launch 归属契约、编排入口 `supervisor.launch.py`、`managed()` launch 助手 | 硬件驱动、GPU、任何 `rak_gpu`/厂商包；算法 |
| `rak_metrics` | 命令年龄/反馈年龄/各流 `now-stamp` 的 p50/p95/p99/丢帧估计，发 `/rak/metrics`（`diagnostic_msgs`）+ 轻 HTTP；rosbag 录制白名单 | 运动控制、任务决策、新增消息契约 |
| `rak_bringup` | 硬件全栈组合层：全部硬件 launch（雷达/相机/slam/nav2/chassis_bringup/canopen_bringup/agent/speech）、参数、udev、Cyclone DDS、操作入口 | 组装契约（在 rak_orchestration）、通用算法或传感器 SDK |

退役 CAN v3 host 栈（`rak_can_driver`、`rak_serial_link`，ADR-0012/0013）的本地 `attic/`
归档已于 2026-10-10 删除，代码仅存 git 历史。USB v2 传输链路及其包、直连 TUI、
串口脚本与 `docs/protocol/` 已按 **ADR-0016**（2026-10-14）删除，删除前状态在
git tag `pre-usb-removal-20261013`。

关键依赖方向（箭头表示“依赖”）：

```text
rak_agent ------------> rak_interfaces ---> 标准 ROS 消息
rak_stereo_driver ----> rak_gpu ----------> JetPack SDK
rak_task_executor ----> rak_interfaces / rak_intent / Nav2 Action
rak_speech -----------> rak_intent / rak_interfaces / audio_common_msgs（/audio、/audio_out）
rak_commissioning ----> 标准 ROS 消息（Twist；经驱动节点对话，不直连硬件）
rak_arm_simulation --> standard trajectory/gripper actions
rak_moveit_config ----> rak_description / rak_arm_simulation (simulation only)
rak_wbc -------------> rak_description（臂 URDF 合并） / mecanum reference / 臂位置控制器（待建）
rak_chassis ----------> CAN v4（SocketCAN） — Session 抽象之后上层无感；v5 走 ros2_canopen（ADR-0024）
rak_orchestration ----> 清单 / 档位 / 编排入口（不依赖任何硬件包）
rak_bringup ----------> rak_orchestration + 各能力启动入口（含 rak_agent，唯一被允许的决策层依赖者）
```

所有可构建包位于 `src/`（ADR-0013），ROS 包名 `rak_agent`、Python 模块名 `rak_agent`。
它由 `colcon` 标准构建、安装为 `ament_python` 包，并通过 console entry
point 使用 ROS 系统 Python；没有独立 venv 或 Laya 推理子进程。episode 保存在
仓库根 `data/episodes/`。Jev 的 TensorRT 原生后端目标是在该节点进程内加载 Jetson
系统 TensorRT Python binding；当前需先确认 ONNX、tokenizer 和输出语义，后端尚未实现。

## 4. 运行架构与实现状态

```text
设备数据 / 语音 / 执行反馈
          |
     状态与事件层
          |
   rak_agent：现有 Jev/S1 与 RAI/S2 原型
          | (尚未接入 task executor 的 S1 任务仲裁)
  rak_task_executor：本地文字解析 / 可选 RAI 地点选择
          | 技能调用、取消、进度、结果
   Nav2 / MoveIt / 对话 / 表达
          |
   确定性安全裁决与硬件适配
          |
         F407
```

### 认知侧

Jev 在当前任务允许的候选动作中做选择，保持同进程直接函数调用。
置信度不是安全许可；异常输出走可预测的降级路径。

Harness 只在慢路径管理任务上下文、工具白名单、预算、超时、事件摘要和失败恢复。
它输出结构化目标或计划修订，不直接发布速度或关节命令。
持续思考由任务进展、失败和环境变化触发，并合并重复事件；
不是每帧调用云端，也不让一条迟到回复覆盖更新的用户意图。

当前实现边界：`rak_task_executor` 默认只做本地命令解析和已配置地点别名查找；
未知地点可选调用 RAI 结构化输出，但结果必须是 waypoint 白名单成员。该慢路径
不是 Laya/Jev 的 System 1，也没有 Harness 持续任务调度。**ADR-0010 已定义**
`rak_agent`（Jev）与 task executor 之间的共享仲裁缝：Jev → `/brain/decision` →
Harness（`/harness/request` / `/harness/proposal`）→ 执行器（校验 `task_id`/`revision`/
`expires_at` 与白名单后接受）。执行器**不**直连 S1，Jev 的影响是传递式、非权威的。
但 Harness 本体尚未实现（`src/rak_harness` 未创建），所以现在仍**不能**描述为完整的
Jev + Harness VLA。

`rak_ai` 保持在 `rak_agent` 包内；推理适配器不得另起服务或改变 `resolve()`、
安全裁决或训练契约。当前 TensorRT 适配仍待模型资产和数值 parity 验收。
只有独立部署或资源隔离确有必要时才新增进程。

### 执行侧

`rak_task_executor` 已拥有单一导航任务生命周期，通过 `/task/execute` Action、
`/task/control` Service、`/task/status` 状态和 Nav2 `NavigateToPose` Action 工作。
当前不拥有机械臂任务执行，也不直接发速度或关节命令。

`src/rak_wbc`（ADR-0011，2026-10-08 已接受）是底盘与臂之间的**运动仲裁层**：
100 Hz 速度级 HQP（Pinocchio + OSQP），常规运动的唯一写入者——底盘写
`mecanum_drive_controller` 的 reference，臂写位置流（伺服模式）。Nav2 的 `/cmd_vel`
降级为其 P2 输入任务；臂的轨迹模式（MoveIt → JTC）与伺服模式互斥，由技能层仲裁。
**Harness 与它没有直连**：提案经执行器/技能层转成 `/wbc/*` 结构化目标
（ADR-0011 D8）；`/wbc/posture_target` 是「呼吸感」表达层的预留缝（门未开、无实现）。
当前交付为骨架 + 桌面数值回归；硬件接入前置（`rak_hardware` 臂接口、臂质量与底盘几何
实测）见该 ADR 待闭合项。

每个技能必须定义前置条件、输入、反馈、完成条件、超时、取消和失败语义。
初期导航技能包括前往配置地点或 map-frame pose、暂停、恢复和取消；状态通过
`/task/status` 发布，控制台提供文字入口。
底盘同一时刻只有一个有效控制权持有者，手动接管和急停必须能撤销自治许可。

任务协议另行定义版本，至少包含：

| 字段 | 用途 |
|---|---|
| `task_id` / `revision` | 识别任务与意图版本，拒绝旧回复 |
| `command_id` | 去重，防止重试变成重复行动 |
| `goal` / `frame_id` | 带明确坐标系的目标或已解析地点 |
| `expires_at` | 过期后不再执行 |
| `status` / `reason` | 明确的执行反馈，而不是模型猜测“完成了” |

建议状态流：

```text
IDLE -> ACCEPTED -> RUNNING -> SUCCEEDED / FAILED / CANCELED
                       |
                   PAUSING -> PAUSED -> RESUMING -> RUNNING
任意活动状态 -> 安全阻断或人工接管 -> 等待明确的恢复许可
```

“暂停”不是直接保存上一条 `cmd_vel`：
请求取消当前 Nav2 goal，确认取消/终态，并在新鲜、有效、无故障的底盘遥测上连续
观察到静止后才报告 PAUSED。fake Nav2 fixture 的反馈是命令运动估算，不是真实遥测。
超时进入失败或安全阻断，不伪报暂停成功。
“恢复”保留任务语义目标，从当前位姿重新规划；重新检查地图、定位、障碍、
用户意图版本和安全许可，不回放旧速度或旧轨迹。
若暂停与到达同时发生，按 Nav2 终态和任务版本收敛到唯一结果。

### 安全与硬件侧

保持确定性 `guard()` 的最终否决原则；未来工程化需让安全检查的调度不等待模型。
当前 `brain` 回调先推理再 resolve，不能作为实时防碰撞保证。
安全许可应落实到命令出口，MCU 还要独立处理断联、使能和超时。

底盘链路的唯一传输是 CAN v4（SocketCAN，ADR-0012/0016）：真底盘经
`chassis_bringup.launch.py` 启动 `controller_manager` → `mecanum_drive_controller` →
`rak_chassis`（`CanSession`）→ `can0` → F407；`rak_navigation` 不依赖上层 bringup。
`task_navigation.launch.py` 的 `motion_backend` 只有 `sim`/`stub` 两个值，默认 `sim`，
组合 launch 本身不连接任何设备。链路安全门是 `chassis_bringup.launch.py` 的
`allow_motion`（默认 false，硬件只握手+遥测、零目标），`can_channel` 默认 `can0`。
另有 `wheel_sign`（默认 `-1 1 -1 1`，顺序 FL FR RL RR）承载**实测的每轮极性装配事实**：
2026-10-14 标定出左侧 FL/RL 电机装向与右侧相反，`rak_system.cpp` 对该轮的命令与反馈
成对取反，使 ROS 的"正速度 = 向前滚"成立。代码层兜底保持中性 `1 1 1 1`，不写死某台车的
接线；换轮或重接线后要重新标定。
工具与上层只准经 topic/action/service 与驱动节点对话，不得直连硬件（ADR-0016）；
链路冒烟用 `scripts/can_link_smoke.py can0`，限速、断联/E-stop 与人工监督低速测试
仍按 `hardware_acceptance.md` 执行，launch 可解析不构成安全证明。

机械臂推荐对接 `ros2_control` 的硬件接口与轨迹控制器，
但插值位置、刷新率和反馈协议必须按实际固件确定，不能把 25 kHz 电机环搬到 ROS。
夹爪使用独立执行接口；失联时的保持/制动按机械安全设计处理，不能统一断电。

不以一个“动作保守程度排序”代替多维安全约束：
机械臂与底盘是否允许运动，应由当前几何、速度、有效状态与许可共同决定。

### 对话与表达侧

未来对话适配器负责流式 ASR/TTS、端点检测、回声处理、打断和轮次状态；
导航任务提交给执行器，对话不能声称执行器尚未确认的成功。
普通打断语音需要快速本地识别路径，不等待云端完整推理；它仍不是硬件急停替代品。

“呼吸感”首先来自及时确认、自然停顿、连贯动作和可解释的恢复。
待机姿态、朝向关注、语音节奏等表达只使用空闲资源和有效许可，
不叠加到活动的导航/抓取控制流上，也不靠随机晃动掩盖系统迟延。
**ADR-0010** 把这条拆成两层：软件可表达（`/harness/event` 的确认、事件合并、
可解释恢复 —— 无运动、无 TTS）与硬件可表达（`ExpressionIntent` / `ExpressionGate` /
`ExpressionSink`，本轮唯一实现是 `NoOpExpressionSink`，门全关）。TTS 引擎尚未选型。

## 5. 状态、时间与资源

- 感知输出需保留采集时间、坐标系、有效性和数据年龄。不要用“刚处理完”冒充“刚采集”。
- ROS 底盘坐标惯例是 x 前、y 左；旧 `SceneState` 使用相机 x 右、z 前。
  在适配层显式变换，不改旧 corpus 的坐标语义。
- 同一个 TF 子坐标系只能有一个权威发布者；全机模型落地后统一安装外参。
- 坐标系分层（**ADR-0015**，2026-10-11 接受）：`world → map → odom → base_footprint → base_link → {4 轮, laser_link, mic_array_link, camera_link, imu_link, arm_base_link}`。
  全程 REP-103（X 前 / Y 左 / Z 上）+ REP-120（`base_footprint` 恒在地面作导航参考，`base_link` 为轮系几何中心）；`world` 是无限开放世界的无界全局根（现为静态恒等占位）。
  静态几何单一来源 = `rak_chassis.urdf.xacro`（`robot_state_publisher` 发布）；`slam_toolbox` 独占 `map→odom`，EKF 独占 `odom→base_footprint`，传感器挂载帧是 URDF 里的 fixed joint，**不再放 launch 的 `static_transform_publisher`**。
- 观测优先最新帧；任务与取消命令必须确认、幂等，不能用无限队列保存过期动作。
- TTL 用清晰的时钟语义；本机超时使用单调时钟，仿真遵循 ROS 时钟，不混用。
- S1 历史值约 158 ms，S2 约 12–30 s，均不是服务等级保证。
  实机需测端到端 p50/p95/p99，而不只测一次模型调用。
- 约 3.4 GB 可用总内存的设备不能默认同时常驻所有模型与 GUI；
  用运行配置选择能力，按场景测 RSS、GPU/CMA、CPU、丢帧和命令年龄。

## 6. 分阶段验收

| 阶段 | 状态与交付 | 必须通过 |
|---|---|---|
| A：包迁移 | 已完成，详见 `docs/migration.md` | 全工作区构建、安装检查、旧 AI 回归 |
| B：任务内核 | 软件实现；fake Nav2 覆盖成功、重复请求、暂停/确认/恢复、服务取消和 Action 取消 | 到达/暂停竞态、各种超时、慢规划迟到、重启等剩余竞态仍需扩测 |
| C：Nav2 接入 | Action、waypoint 和全向参数已接入 | 有效传感器/定位环境下验证 lifecycle、碰撞监控输出和速度链；确认每种 backend 单写者 |
| D：硬件闭环 | CAN v4 链路冒烟通过（2026-10-13：`can_link_smoke.py can0` 10/10，BAT≈12500 mV、FAULT=0、BUSOFF=0）；`rak_chassis` 四轮接口过底盘冒烟（固件 R37）；7 轴位置接口 + `arm_controller`/`gripper_controller` 已在实机带起，j1 点动 + Home 往返实动验收通过（2026-10-14：`allow_motion:=true` 下 ARM_CMD 帧级验证，0.1° 量化爬升/回落，回读跟随，无故障） | 里程计/IMU、断联/E-stop、限速与人工监督低速测试；**J2–J6 实动、运动中限位与失联停机、夹爪**（无硬件通道） |
| E：语音与智能体 | 文字 CLI 已有；ASR 已接阵列硬件（2026-10-10：阵列 → `/audio` → Vosk，实测解码 0.84–1.01× 实时，超预算淘汰最旧音频），TTS 未接入；task 与 S1/Harness 仲裁**已定义（ADR-0010）但未实现** | 阵列声学段真人说话验收、无网络本地打断、旧提案拒绝、误识别处理、harness 路径端到端延迟与表达 stub 落地 |
| F：机械臂 | 六轴隔离夹具和软件动作服务器已实现 | 真实几何/限位/映射/失联策略确认后再做实机适配 |

## 7. 未因本次重构而解决的问题

- 深度/检测/跟踪融合尚未形成实际 `TrackedObjects` 生产链。
- 旧 S2 `_drain()` 的 `applied` 是日志标记，没有真正应用到执行器。
- Nav2 Omni、速度平滑器和 collision monitor 配置已更新，但真实 LaserScan 停止响应、
  lifecycle 与末端单写者尚未做运行验收；`sim` 输出只记录估算。
- 当前 `odom_source:=ekf` 预留选项不会自动启动 EKF；仍需外部提供有效 TF。
- 相机标定、深度与彩色对齐、硬件 TF、真实里程计质量都需要工程验证。
- 六轴模型尺寸/限位是测试夹具占位值；没有真实六轴实机支持。
- `rak_task_executor` 的慢规划目前只是可选 RAI 地点选择。**ADR-0010 已定义** Jev/Harness
  的统一仲裁缝与契约，但 `src/rak_harness` 与 `harness_node` 尚未实现，
  所以这条现在仍是缺口，只是从"没有设计"变成了"有设计、待实现"。
- `scripts/serial_link_test.py` 与 `scripts/serial_100hz.py` 已随 USB v2 链路删除
  （ADR-0016）；链路排障改用 `scripts/can_link_smoke.py`、
  `ip -details -statistics link show can0` 与 `dmesg | grep -i can`。

这些缺口必须在对应阶段关闭；不能以构建、launch 可解析或仿真测试通过作为实车安全证明。
具体门槛见 [硬件验收清单](hardware_acceptance.md)。
