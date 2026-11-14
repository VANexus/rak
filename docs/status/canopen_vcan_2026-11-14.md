# vcan0 上的 ros2_canopen 主站验证（无下位机）

> 状态：已验证（2026-11-15 复测闭合 SYNC） · 最后更新：2026-11-15

日期：2026-11-14。环境：Jetson Orin，aarch64，ROS 2 Jazzy；apt 包 `ros-jazzy-canopen-core` /
`canopen-master-driver` / `canopen-proxy-driver` / `canopen-base-driver` / `canopen-402-driver` /
`canopen-interfaces` / `canopen-ros2-control` / `canopen-fake-slaves`，均为 **0.3.4**。
总线只有 vcan0（虚拟回环），**没有接真实下位机，也没有动 can0**。

## 结论

主站在 vcan0 + `canopen_fake_slaves` 上跑通：NMT reset → 从站 boot-up → SDO 读配置 →
NMT start 全部上总线，主站进程不 abort；`ros2 service call /master/sdo_read` 读 0x4001
返回 `success=True`。

**此前的 abort 不是 lifecycle 问题**：`device_container_node` 是
`rclcpp_components::ComponentManager`，**没有** configure/activate 生命周期；`ros2 lifecycle
set /device_container ...` 对它不适用（连 `ros2 lifecycle get` 都会说节点不存在）。
abort 的根因是 `master_config` 传错了文件（见下节）。

## 根因与 workaround

- `master_config` 必须是 **dcfgen 生成的 master DCF**，其中带 CiA 302 网络表：
  `1F80 NMT startup`、`1F81 NMT slave assignment`（本网表 `2=0x00000005`）、
  `1F85 Vendor-ID`（`2=0x00000360`）。lely 主站按这张表去 boot 从站。
- 传从站 `simple_slave.eds` 也能"加载"（lely 把它当主站 OD 读，不报错），但没有网络表 →
  主站不发 NMT reset（实测 vcan 上只看到 `701#00`，没有 `000#82xx`）→ 代理驱动
  `Wait for device to boot...` 1 s 超时 → 走上游 retry 分支
  `Sending NMT reset before retrying boot` → `nmt_command()` 里
  `master.Command()` 抛未捕获的 `std::system_error`，`what(): Command: Operation not permitted`，
  进程 abort。
- 这是 **workaround（换正确输入），不是上游修复**。0.3.4 的 retry 路径仍会抛未捕获异常，
  触发条件（首次 boot 失败）只是被正确的 master DCF 绕开了。EPERM 的确切来源未定位。

正例/反例对照（同一 vcan0、同一 fake slave）：

```text
# 反例 master_config=simple_slave.eds：没有 NMT reset，1 s 后 abort
(1794669220.485280) vcan0 702#00      # 从站 boot-up
(1794669224.342586) vcan0 701#00      # 主站 boot-up
# 随后：Boot attempt 1 failed: Boot Timeout ... Command: Operation not permitted → Aborted

# 正例 master_config=dcfgen 生成的 master.dcf
```

## 可用启动序列

任意目录（下列文件放 /tmp）：

```yaml
# /tmp/bus.yaml —— 扁平 schema：master 一段 + 每个从站一个顶层键
options:
  dcf_path: "/opt/ros/jazzy/share/canopen_fake_slaves/config"
master:
  node_id: 1
  driver: "ros2_canopen::MasterDriver"
  package: "canopen_master_driver"
canopen_chassis:
  node_id: 2
  driver: "ros2_canopen::ProxyDriver"
  package: "canopen_proxy_driver"
  dcf: "simple_slave.eds"
```

`options.dcf_path` 是 EDS 目录，从站 `dcf` 写**文件名**（驱动内部按
`dcf_path + "/" + dcf` 拼路径；`dcf` 写绝对路径会拼成
`.../config//opt/.../simple_slave.eds` 并在启动时 abort）。`dcf_path` 只在 dcfgen 和
bus_config 解析时生效；主站自己的 `master_config` 是独立参数。

```bash
# 1) 生成主站 DCF（dcfgen 的输入就是 bus.yaml；从站 dcf 是相对名，所以在 EDS 目录里跑）
cd /opt/ros/jazzy/share/canopen_fake_slaves/config
source /opt/ros/jazzy/setup.bash
dcfgen -d /tmp/dcf /tmp/bus.yaml        # 产出 /tmp/dcf/master.dcf

# 2) 先起 fake slave（内部是 Launch 自动 configure + activate，约 1-2 s 生效）
ros2 launch canopen_fake_slaves basic_slave.launch.py can_interface_name:=vcan0 node_id:=2

# 3) 再起主站（master_config 必填，空串会被 device_container 拒绝）
ros2 run canopen_core device_container_node --ros-args \
  -p bus_config:=/tmp/bus.yaml \
  -p can_interface_name:=vcan0 \
  -p master_config:=/tmp/dcf/master.dcf
```

无需先 configure/activate。`boot_timeout` 不传时主站用 2000 ms，代理驱动侧默认 1000 ms，
boot 在干净 vcan 上 <10 ms 完成，调它不解决"没有网络表"的问题。

一条命令复现 + 断言：

```bash
python3 scripts/verify_canopen_vcan.py    # exit 0 = 通过；1 = 未通过；失败时保留 /tmp/verify_canopen_* 日志
```

## 实测帧证据

`candump -L vcan0`，取自 `python3 scripts/verify_canopen_vcan.py` 通过的那次运行：

```text
(1794671888.063512) vcan0 702#00                # 从站 boot-up
(1794671888.685810) vcan0 701#00                # 主站 boot-up
(1794671888.685892) vcan0 000#8200             # NMT reset node 2（0x82 + node id）
(1794671888.686274) vcan0 702#00                # 从站重新 boot-up
(1794671888.686393) vcan0 602#4000100000000000  # SDO 读 0x1000/0
(1794671888.686436) vcan0 582#4300100000000000  # 响应：4 字节，0x00000000
(1794671888.686474) vcan0 602#4018100100000000  # SDO 读 0x1018/1（Vendor-ID）
(1794671888.686506) vcan0 582#4318100160030000  # 响应：0x00000360
(1794671888.686647) vcan0 000#0102             # NMT start node 2
(1794671892.818650) vcan0 602#4001400000000000  # service call 读 0x4001/0
(1794671892.818757) vcan0 582#4301400000000000  # 响应数据 0x00000000
```

服务侧证据：

```text
$ ros2 service call /master/sdo_read canopen_interfaces/srv/COReadID \
    "{nodeid: 2, index: 16385, subindex: 0, canopen_datatype: 7}"
response:
canopen_interfaces.srv.COReadID_Response(success=True, data=0)
```

节点/服务现场（同一次运行）：`/master`、`/canopen_chassis`、`/device_container` 在
`ros2 node list` 里；`/master/sdo_read`、`/master/sdo_write`、`/canopen_chassis/sdo_read`、
`/canopen_chassis/nmt_reset_node`、`/canopen_chassis/nmt_start_node` 都在。

## 未闭合项（没跑过，不算验证）

- **ros2_control**：`canopen_ros2_control/CanopenSystem` 能不能被 `ros2_control_node` 加载、
  能不能 export 接口，本轮**没做**。下一步是 /tmp 下写带 `<ros2_control>` 的最小 URDF +
  controller_manager 起一次，只看加载与接口导出，不发运动命令。
- **SDO 写**：只验证了读（0x1000 / 0x1018:1 / 0x4001）。写路径、写失败时的 abort 码未测。
- **SYNC 周期精度**：从站 EDS 里 0x1005 = 0x80，但主站侧没有 SYNC 生产者配置，
  实测一帧 `080#` 都没抓到；周期抖动/精度未测。
- **心跳**：`simple_slave.eds` 的 0x1017 存在但没有 `DefaultValue`（=0），fake slave 只在
  boot 时发一帧 `702#00`，**没有周期心跳**；心跳超时/EMCY 行为未测。
- **PDO**：TPDO 映射对象 0x4001 存在，但本轮没有触发 PDO 收发（polling 模式下只走了 SDO），
  PDO 映射/传输类型未验证。
- **P3 的真实总线**：真实 can0（R73/A14 板）完全没碰；vcan 上通过不等于真机通过。
- **EPERM 根因**：只观察到 retry 路径触发即 abort，没有定位 `Command: Operation not permitted`
  是哪个 errno 来源；上游 0.3.4 未修。
- `docs/README.md` 索引**未加**（本仓 docs 规范要求新增文档同时进索引；按本次任务约束，
  `docs/README.md` 正在并行重构，留给 lead 补，否则 `python3 scripts/check_docs.py` 会报缺项）。

## 相关文件

- `scripts/verify_canopen_vcan.py` —— 零外部依赖（bash + 标准库 + ros2 CLI），自建 vcan0、
  起 slave 与 master、抓帧、调 SDO 读、断言、清理；exit 0/1。
- 上游参考（0.3.4）：`canopen_tests/launch/proxy_setup.launch.py` 用
  `canopen_tests/config/simple/master.dcf`（同目录 `bus.yml` 经 `generate_dcf` 生成），
  而不是从站 EDS —— 与本验证结论一致。

## 2026-11-15 复测（闭合 SYNC 未验证项）

`python3 scripts/verify_canopen_vcan.py`（脚本已补 `sync_period: 2000` + 两道新断言）：

```text
[ok] fake slave boot-up: 702#00
[ok] NMT reset 000#8202 -> boot-up -> SDO config -> NMT start 000#0102
[ok] SYNC 080# producing at 500 Hz (0x1006 = 2 ms)
[ok] SDO read 0x4001: success=True data=0 (wire 602#4001.. -> 582#43..)
[pass] ros2_canopen master works on vcan0 with a fake slave; no abort
```

结论：**上一轮「主站不发 SYNC」是误判**——根因是验证脚本的临时 bus.yaml 没写
`sync_period`，dcfgen 生成的 master.dcf `0x1006=0`，lely 主站自然不产 SYNC。修复后
`080#` 实测稳定 500 Hz，与 [V5] §3 的 2 ms 一致。ADR-0024 三次修订同步记录。

同步落地（同日，ADR-0024 三次修订）：

- bus.yaml 驱动切回**非 lifecycle** `MasterDriver` / `ProxyDriver`；chassis_bringup 删
  6 s / 9 s lifecycle 定时，controller_manager 6 s 起。台架复测硬件组件与四控制器全激活。
- 薄桥 rpdo 订阅改 best-effort keep-last-1（100 Hz 状态流不积压旧帧）。
- `canopen_contract.yaml` 移除固件上不存在且会 HardFault 的 0x1008/0x1009/0x100A；
  `manifest_sw_version` 更新为 R76 / A36（固件仓 HEAD），在板版本以 SDO 实读为准。
- `src/rak_canopen/config/chassis.json` 换成与 MCU SSOT 逐字节一致（LF）的副本；
  此前 CRLF 行尾导致 sha256 与源仓不同。

仍然未闭合（与上节一致，新增第 2 条的台架范围）：

- ros2_control 硬件组件在**有真实 TPDO 映射的从站**上的 read/write——fake slave 的
  EDS 不是我们的对象表，台架只能验证到「驱动激活 + 硬件激活 + startup_grace 后判死」。
- SDO 写路径、心跳消费者 500 ms、EMCY 降级：未测。
- 真机 can0（R73/A14 或更新）：完全没碰。
