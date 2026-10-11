# ADR-0014: RGB-D 相机契约 —— 深度流必修、多后端同构、外参不得臆造

- **状态**: 已接受（2026-10-10）
- **日期**: 2026-10-10
- **相关**: `docs/cameras.md`（契约细节与实测数据）· `src/rak_bringup/launch/depth_camera.launch.py`
  （后端选择入口）· `src/rak_bringup/launch/astra.launch.py` · `src/rak_bringup/launch/gemini335l.launch.py` ·
  `src/rak_astra_driver` · ADR-0006（3D 避障）· ADR-0008（末端/感知）· ADR-0015（坐标系：`camera_link` 挂载与外参）
- **说明**: 本记录原文件此前缺失，2026-10-11 依据 `docs/cameras.md` 与工程会话记录补录，正文不改动既有约定。

---

## 背景

- 机器人有两条深度相机路线：Orbbec **Astra**（`rak_astra_driver`，SDK v1）与 Orbbec **Gemini 335L**
  （上游 `orbbec_camera`，SDK v2）。两者不能同时启动，且各自的 SDK 不得渗进导航/感知消费者。
- 导航代价地图、3D 避障（ADR-0006）、点云管线等消费者只关心"有一套带标定的深度流"，
  不应关心背后是哪台相机、哪个 SDK——这是 ADR-0013「同需传感器发布相同消息集、靠 launch 参数切换」原则
  在相机上的具体化。
- 深度与彩色**未必配准**，且相机到 `base_link` 的安装外参在装车前无法得知。

## 决策

1. **深度流是必修项**：任一深度后端必须发布
   `/camera/depth/image_raw`（`16UC1`）+ `/camera/depth/camera_info`。
   **彩色是可选项**：有则发 `/camera/color/image_raw` + `/camera/color/camera_info`，无则不发。
2. **单一后端、参数切换**：`depth_camera.launch.py` 一次只起一个后端，`camera:=astra|gemini335l`
   （默认 Gemini 335L）选择后端；聚焦入口 `astra.launch.py` / `gemini335l.launch.py` 单独可用，
   **不得与 `depth_camera.launch.py` 并用**。
3. **SDK 隔离**：相机 SDK 只存在于对应驱动/适配层，导航与感知消费者只依赖上面的深度话题与
   `CameraInfo`，不 import 任何相机 SDK。
4. **红外立体相机**（本仓 `rak_stereo_driver`）**不属于**本契约：它保留历史的 `eye/*` 话题名与
   `SensorDataQoS`，不改名、不并入 `/camera/*`。
5. **外参不得臆造**：`base_link→camera_link` 的安装标定必须实测（见 ADR-0015）；相机内部
   `camera_color_optical_frame` / `camera_depth_optical_frame` 由驱动发布，本仓不重建。
   **深度—彩色配准若未启用，消费者须各自读取 `CameraInfo` 与 frame_id，不得假设已对齐。**

## 理由

- 消费者只需一套稳定契约：把选择权放在 launch 的 `camera:=` 上，上层零改动即可换相机，
  符合 ADR-0013 的分层规则，也让 Astra/Gemini 的差异化留在各自驱动内。
- 深度为必修、彩色为可选：导航与避障吃的就是深度；彩色可能缺失（仅深度后端、带宽取舍），
  若把彩色设为必修会把可选硬件变成硬依赖。
- 外参/配准不臆造：错误的深度外参是"小而静默的永久误差"（与 `lidar_z` 同类），宁缺勿错。

## 后果

- **正面**：换相机 = 换一个 launch 参数；消费者与 SDK 彻底解耦；深度话题名统一，代价地图/点云
  配置无需按相机分支。
- **负面**：
  - 两个后端长期并存（Astra 需 SDK v1.10.18，本机未装则该后端不可构建，见 AGENTS 构建注意）；
  - "彩色可选"要求消费者真正做缺失处理，而不是默认彩色总在；
  - 契约只约束话题与帧，不约束编码/时间戳/配准——这些仍需逐后端实测记忆（`docs/cameras.md`）。

## 被否决的方案

- **为每台相机定义一套话题**（如 `/gemini/depth`、`/astra/depth`）：消费者被迫分支，
  违背 ADR-0013 的"同需同消息集"。
- **把 `rak_stereo_driver` 的 `eye/*` 并入 `/camera/*` 契约**：立体相机是独立感知源（双目本征标定、
  另一个 SDK），强行统一会污染两边的帧与 QoS 语义；保留其历史话题名。
- **在 URDF/本仓定义相机光学帧**：与 Orbbec 驱动自发布冲突（造出双发布者），且外参未标定
  （ADR-0015 已否决）。

## 待闭合项

1. `docs/cameras.md` 仍以旧包名 `rak_sensor_bringup` 记述（该包已并入 `rak_bringup`，ADR-0013）——
   路径/包名需更新。
2. `base_link→camera_link` 外参未实测（TODO P1-6）；深度—彩色配准是否启用的结论未落到运行时断言。
3. 契约验证器（话题/编码/CameraInfo 存在性检查）尚未落地为脚本。
4. Astra 后端在本机因缺 Orbbec SDK v1.10.18 无法构建/联调。
