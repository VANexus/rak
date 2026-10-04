# 分层具身智能架构

状态：2026-10-04 统一包迁移与软件任务闭环已实现。文字任务、fake Nav2
暂停/恢复/取消、UART v1 主机端和隔离六轴动作夹具均有实现。硬件 backend launch
已通过安装态解析和模式映射检查，但未做设备运行验证；真实传感器 Nav2、F407 固件
互操作和整车安全验收仍未完成。README 是工程入口，当前进度汇总见
[项目总体状态](project_status.md)。

## 1. 目标与硬件事实

第一条产品闭环：对话下达导航任务，移动过程中可打断，并在重新验证条件后恢复。
最终扩展到机械臂操作、持续任务理解和自然表达。

- 上位机：Jetson Orin Nano，ROS 2 Jazzy，JetPack 7.2。
- 已有雷达和深度相机测试代码，不能等同于完成工程化接入。
- 底盘：F407 自行完成麦轮运动学与四轮 25 kHz 闭环，上位机给 `(vx, vy, wz)`。
- 底盘有独立 IMU 做稳定，但是否提供 ROS 可消费的 IMU/协方差/时间戳还需对接。
- 机械臂目标：六个定位关节，夹爪独立。当前旧 URDF 是五轴演示，禁止当作实物模型。
- 语音硬件尚未接入。先以文字事件验证任务控制，再接 ASR/TTS，避免两类问题混在一起。
- 叶节点旧协议记录不能证明新的六轴机械臂已经具备相应通道、反馈和限位。

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
| `rak_sensor_bringup` | 设备启动、滤波和设备参数 | 任务策略、整机状态机 |
| `rak_description` | URDF、几何、惯量与可视化 | 导航参数、模型策略 |
| `rak_navigation` | SLAM、全向 MPPI、smoother/collision-monitor 配置 | 对话理解、LLM 调用 |
| `rak_task_executor` | 单一任务状态机、文字意图、Nav2 Action 生命周期 | 原始底盘/关节命令、感知算法 |
| `rak_base_driver` | Jetson-F407 UART v1 编解码、遥测与显式使能门控 | F407 固件、下游 CAN、硬实时安全 |
| `rak_moveit_config` | 旧 demo 与独立六轴仿真配置 | 电机闭环、实机几何声明 |
| `rak_arm_simulation` | 软件轨迹与夹爪 Action server、估算关节状态 | 物理仿真、真实硬件接口 |
| `rak_motion_stub` | 明确标注的测试转发/运动估算 | 硬件驱动、安全保证 |
| `rak_agent` | 当前 S1/S2 认知原型 | 电机驱动、同步等待云端后控制 |
| `rak_bringup` | 整机组合、DDS、操作入口 | 通用算法或传感器 SDK |

关键依赖方向（箭头表示“依赖”）：

```text
rak_agent ------------> rak_interfaces ---> 标准 ROS 消息
rak_stereo_driver ----> rak_gpu ----------> JetPack SDK
rak_task_executor ----> rak_interfaces / rak_agent(task intent) / Nav2 Action
rak_base_driver -----> rak_interfaces / serial
rak_arm_simulation --> standard trajectory/gripper actions
rak_moveit_config ----> rak_description / rak_arm_simulation (simulation only)
rak_navigation -------> rak_sensor_bringup / rak_motion_stub / Nav2 / SLAM
rak_sensor_bringup ----> 自有相机驱动 / 上游雷达 / laser_filters
rak_bringup ----------> 各能力的启动入口
```

`brain/` 是有意保留的源码位置例外，ROS 包名为 `rak_agent`，pip 名为 `rak-brain`，
Python import 为 `rak_ai`。保持已有 venv 和 episode 路径，避免移动虚拟环境后 shebang、
editable install 和数据定位同时失效。今后若统一物理位置，应独立迁移数据目录与部署环境。

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
不是 Laya/Jev 的 System 1，也没有 Harness 持续任务调度。现有 `rak_agent` 与
task executor 尚未通过共享任务仲裁接口联动；不能描述为完整的 Jev + Harness VLA。

先保留 `rak_ai` 的现有文件，不在本次包迁移中改写 `resolve()` 或训练契约。
未来可在该 Python 包内逐步分出 `state`、`policy`、`harness` 模块，
只有独立部署或资源隔离确有必要时才新增进程。

### 执行侧

`rak_task_executor` 已拥有单一导航任务生命周期，通过 `/task/execute` Action、
`/task/control` Service、`/task/status` 状态和 Nav2 `NavigateToPose` Action 工作。
当前不拥有机械臂任务执行，也不直接发速度或关节命令。

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

`rak_base_driver` 已实现提议中的 UART v1 主机端编解码、100 Hz 命令节拍、状态发布
和显式 ARM 服务，但默认不打开串口、零速度限值且 DISARMED。仓库没有相匹配的
F407 固件；UART 文档/PTY 测试不证明互操作。底盘运动学仍留在 MCU，MCU 必须独立
实现命令租约、断联停止、E-stop 和故障锁存。

顶层 `rak_bringup` 的 launch 提供 `motion_backend:=hardware`，并在 bringup 层启动 UART
节点；`rak_navigation` 不依赖上层 bringup。该路径已通过安装态解析和模式映射检查，
但没有运行 Nav2 或连接设备。组合 launch 默认 `sim`；
硬件路径的 `hardware_enable` 默认 false，节点不会打开已配置的串口。直接启动
`BaseUartNode` 时默认无端口；bringup launch 则默认传入稳定别名
`/dev/wheel_motor`，但在硬件开关关闭时不会访问设备。驱动的三个速度限值必须都是
有限正数才允许 ARM；`0.0` 不是不限速或透传。

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

## 5. 状态、时间与资源

- 感知输出需保留采集时间、坐标系、有效性和数据年龄。不要用“刚处理完”冒充“刚采集”。
- ROS 底盘坐标惯例是 x 前、y 左；旧 `SceneState` 使用相机 x 右、z 前。
  在适配层显式变换，不改旧 corpus 的坐标语义。
- 同一个 TF 子坐标系只能有一个权威发布者；全机模型落地后统一安装外参。
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
| C：Nav2 接入 | Action、waypoint 和全向参数已接入；hardware 末端 launch 静态检查通过 | 有效传感器/定位环境下验证 lifecycle、碰撞监控输出和速度链；确认每种 backend 单写者 |
| D：硬件闭环 | UART v1 host codec/driver 已实现，F407 固件未实现/未核验 | F407 匹配、断联/E-stop、里程计/IMU、限速与人工监督低速测试 |
| E：语音与智能体 | 文字 CLI 已有；ASR/TTS、task 与 S1/Harness 仲裁未接入 | 无网络本地打断、旧提案拒绝、误识别处理与端到端延迟 |
| F：机械臂 | 六轴隔离夹具和软件动作服务器已实现 | 真实几何/限位/映射/失联策略确认后再做实机适配 |

## 7. 未因本次重构而解决的问题

- 深度/检测/跟踪融合尚未形成实际 `TrackedObjects` 生产链。
- 旧 S2 `_drain()` 的 `applied` 是日志标记，没有真正应用到执行器。
- Nav2 Omni、速度平滑器和 collision monitor 配置已更新，但真实 LaserScan 停止响应、
  lifecycle 与末端单写者尚未做运行验收；`sim` 输出只记录估算。
- 当前 `odom_source:=ekf` 预留选项不会自动启动 EKF；仍需外部提供有效 TF。
- 相机标定、深度与彩色对齐、硬件 TF、真实里程计质量都需要工程验证。
- 六轴模型尺寸/限位是测试夹具占位值；没有真实六轴实机支持。
- `rak_task_executor` 的慢规划是可选 RAI 地点选择，不是当前 Jev/Harness 的统一任务推理。
- `scripts/serial_link_test.py` 是拒绝执行旧 CRC-8 实体串口协议的兼容入口；
  UART v1 PTY-only 测试入口是 `scripts/serial_100hz.py`。

这些缺口必须在对应阶段关闭；不能以构建、launch 可解析或仿真测试通过作为实车安全证明。
具体门槛见 [硬件验收清单](hardware_acceptance.md)。
