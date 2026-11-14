# ADR-0016: 删除 USB v2 链路 —— CAN v4 是唯一传输，工具只走 ROS 接口

- **状态**: 已接受（2026-10-14）
- **日期**: 2026-10-14
- **相关**: `src/rak_chassis`（`session.hpp` / `can_session`）· `../vehical_rak_chassis/docs/PROTOCOL.md`
  （v4，协议事实源）· `scripts/can_link_smoke.py`（链路冒烟）· ADR-0013
- **取代**: **ADR-0012 的决策 1 中"USB v2 是冻结的备用链路"部分，以及其"被否决的方案"里
  "USB 退役删除"这一条**。ADR-0012 的其余内容（`Session` 抽象、上层禁分支传输、协议事实源在下位仓）
  **继续有效且被本篇加强**。
- **快照**: 删除前的完整状态在 git tag `pre-usb-removal-20261013`（指向 `0996b94`）。

---

## 背景

- ADR-0012（2026-10-09）确立 CAN v4 为主链路、USB v2 为**冻结备用**，并把"退役删除 USB"
  明确写进被否决项，理由是"无收发器/总线故障时的唯一救急路径，且 `SerialSession` 是
  `CanSession` 的行为参照实现（测试对拍用），删除得不偿失"。
- 2026-10-13 CAN 链路验收：物理根因是 **H/L 接线对调**（症状链 `berr-counter tx 159` →
  qdisc 丢 46711 帧 → `send()` 报 `OSError 105`），对调后 `scripts/can_link_smoke.py can0`
  连续两次 **10/10 通过**（BAT≈12500 mV、FAULT=0、BUSOFF=0）。CAN 链路已自证可用。
- 同时暴露出**两套控制底盘的路径并存**：`chassis_key_test` 发 Twist →
  `mecanum_drive_controller` → `rak_chassis` → CAN（正规 ros2_control 链路）；
  而 `drive_tui` / `servo_tui` 用 pyserial **直连 `/dev/ttyACM0`**，完全绕过驱动节点。
  后者是 USB 时代的标定工具，早于 `Session` 抽象，从未被收编——它正是
  ADR-0012 决策 2「禁止在任何上层代码里判断或分支传输类型」的违规样本。
- 工作区里与 USB/串口耦合的还有：`rak_base_driver`（USB v2 编解码 + UART v1 驱动节点）、
  `rak_serial_link`（已退役的串口链路节点）、`base_uart.launch.py`、
  `task_navigation.launch.py` 的 `motion_backend:=hardware` 分支，以及 6 个串口脚本。

## 决策

1. **CAN v4（SocketCAN）是唯一传输。** `hardware_parameters` 的 `transport` 字段、
   `device` 字段、URDF/xacro 的同名参数、launch 的 `transport:=usb|can` 选项**全部删除**；
   `rak_system.cpp` 的后端工厂收敛为一行 `make_unique<CanSession>(...)`。
   `Session` 抽象本身保留——它是上层与下位机的接缝，只是现在只有一个实现。
2. **任何工具不得直连硬件，只准通过标准 ROS 接口与驱动节点对话。**
   通信链路（USB/CAN）是驱动节点**下面**的适配细节，不是上层的可选分支：
   ```
   TUI/标定工具  ──topic/action/service──►  驱动节点(rak_chassis / ros2_control)
                                             └──Session(CanSession)──► can0 ──► F407
   ```
   由此删除 pyserial 直连的 `drive_tui`、`servo_tui`。底盘的 ROS 化输入源已存在
   （`chassis_key_test`，发布 `Twist`），按本原则保留。
3. **连带删除的包与脚本**（全部可从 tag `pre-usb-removal-20261013` 取回）：
   | 对象 | 内容 |
   |---|---|
   | `src/rak_chassis` | `serial_session.cpp` / `serial_session.hpp` / `pty_peer_test.cpp` |
   | `src/rak_base_driver` | 整包（USB v2 codec 1370 行 + UART v1 驱动节点） |
   | `src/rak_serial_link` | 整包（已退役的串口链路节点） |
   | `src/rak_commissioning` | `drive_tui.py`、`servo_tui.py` 及其 `console_scripts` 入口 |
   | `src/rak_bringup` | `base_uart.launch.py`、`task_navigation.launch.py` 的 `hardware` 后端分支 |
   | `scripts/` | `mecanum_tui_v2.py` `pty_f407_sim.py` `serial_100hz.py` `serial_100hz_test.py` `serial_link_test.py` `uart_v1_test.py` |
   | `docs/protocol/` | 整目录（`usb_serial_v2.md` `jetson_f407_uart_v1.md` `jetson_f407_uart_v1.1.md` `PROTOCOL_ALIGN_PACKET.md`） |
4. **服务/话题命名去掉传输痕迹**：`rak_task_executor` 的基础层解除服务由
   `/base_uart_driver/arm` 改为 `/base/arm`，与既有的 `/base/telemetry` 一致。
   生产者（`sim_nav.py`）与消费者（`node.py`）同包，无外部调用方。
5. **传感器侧的 USB 不在本篇范围。** 雷达 `serial_port_=/dev/ttyUSB0`、USB 音箱
   （`plughw:CARD=`）、麦克风阵列的 USB 接口都是**设备的连接方式**，不是 Jetson↔MCU 链路，
   保持原样。同理 `rak_can_driver`（python-can / SocketCAN）是 CAN，不属本篇删除范围。

## 被否决的方案

- **保留 USB 作为救急链路**（ADR-0012 的原立场）：CAN 链路已通过冒烟验收，且"救急"价值
  换来的是**两条可被上层误用的控制路径**——`drive_tui` 直连硬件就是代价的实例。
  维护成本并非 ≈0：`check_workspace`、`package.xml` 依赖、launch 参数、验收文档都要跟着它走。
- **先在 CAN 侧实现 7 轴 `ARM_COMMAND` 接口 + CAN 版示教 TUI，再删 USB**：
  工作量大得多，且臂的硬件门是关着的（ADR-0012 待闭合项 #1 至今未闭合：`rak_chassis`
  仍只导出 4 轮接口，arm controller、夹爪轴都不存在）。用一个已关的门阻塞底盘链路收口不成立。

## 后果

- **机械臂暂时没有交互示教工具。** `servo_tui` 是唯一的手动示教入口，删除后
  `can_full_motion_test.cpp` 仍只做自动扫描。臂的接口按本篇决策 2 的原则重建：
  在 `rak_chassis` 导出 7 轴位置接口后，示教工具发 `JointTrajectory`/position 命令，
  绝不再直连 CAN。**这是 ADR-0012 待闭合项 #1 的直接依赖，本篇不解决它。**
- **少了一条故障排查路径。** 无收发器、CAN 控制器异常时不再有串口兜底；
  现阶段的替代是 `scripts/can_link_smoke.py`（协议层）+ `ip -details -statistics link show can0`
  （物理层）+ `dmesg | grep -i can`（驱动层）。
- **测试对拍能力下降。** `pty_peer_test` 随 `SerialSession` 删除，`CanSession` 不再有
  同 API 的参照实现可比对；其回归改由 `scripts/can_link_smoke.py` 与
  `system_interface_test.cpp` 承担。
- **`check_workspace.py` 的 `EXPECTED` 集合由 23 降为 21**（去掉 `rak_base_driver`、
  `rak_serial_link`），并删除"commissioning 依赖 base_driver 编解码"这条断言。

## 待闭合项

1. ADR-0012 待闭合项 #1（7 轴 `ARM_COMMAND` 位置接口 + arm controller + 夹爪轴）未闭合，
   本篇删除 `servo_tui` 使它从"缺实现"升级为"缺实现且缺工具"。
2. `docs/reference/hardware_acceptance.md` 的第 1 步原为 `scripts/serial_100hz.py`（PTY-only UART 回归），
   已随本篇删除，需改写为 CAN 冒烟步骤。
3. README / `docs/explanation/architecture.md` / `docs/tutorials/construction.md` / `docs/status/project_status.md` /
   `docs/archive/handoff_agent_2026-10-09.md` 中的 USB 与 `motion_backend:=hardware` 描述需与本篇同步。
