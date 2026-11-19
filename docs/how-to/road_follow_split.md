# 纯视觉寻路：控制层用现成 Nav2，判别层留 40 行

> 状态：现行 · 最后更新：2026-11-19（控制层改走 nav2 现成件；轻量直发档保留为降级）

> 2026-11-19 用户决策：**不要造轮子**。转弯、原地旋转、路径跟踪全部交给已装的
> `nav2_rotation_shim_controller` + `nav2_mppi_controller`；本仓只保留生态里
> **没有现成件**的那一层——从路面 mask 判出"路往哪拐"并产出一条会拐弯的折线。

## 1. 两层分工

| 层 | 谁做 | 现成件 |
|---|---|---|
| 分割 | `roadseg_node`（板上，TensorRT） | 模型自备（TwinLiteNet），框架是 `rclpy` |
| **判别**：路往哪拐 | `trace_centerline` + `corner_signal` | **无现成件**（`opennav_following` 是跟人不跟路） |
| 投影到地面 | `ground_points`（`road_goal_node`） | 无现成件（单目 + 标定） |
| 跟踪 / 转弯 / 原地转 | Nav2 | `nav2_rotation_shim_controller`、`nav2_mppi_controller` |
| 选左右（固定赛道） | Nav2 | `nav2_route` Route Graph（GeoJSON） |

## 2. 唯一入口

**只有一个 launch 文件**：`road_follow.launch.py`。里面有 `controller:=direct|nav2`
两档执行层，其余（相机、分割、`/cmd_vel` 桥、RViz）全部共用。

```bash
# 驱动端（Jetson，headless）
ros2 launch rak_bringup road_follow.launch.py role:=driver

# 逻辑端（工作站）
ros2 launch rak_bringup road_follow.launch.py role:=logic controller:=direct enabled:=true

# 单机 / 真机（相机+分割+控制全在板上）
ros2 launch rak_bringup road_follow.launch.py role:=both enabled:=true allow_motion:=true
```

两档执行层：

| `controller:=` | 节点 | 需要什么 |
|---|---|---|
| `direct`（默认） | `road_follow_node` 直发 `/cmd_vel` | 无 TF、无 Nav2 |
| `nav2` | `road_goal_node` 发 `FollowPath`，RotationShim + MPPI 跟踪 | `odom -> base_footprint` |

**`/cmd_vel` 到执行器只有一条路**：`rak_motion_stub/cmd_vel_omni`。它携带实测的
滚子排布取反（`invert_angular_z=true`，2026-11-18：`angular.z=+0.30` → IMU 实测
−37°），`allow_motion:=true` 时由 launch 自动拉起。**任何其他节点都不得直接写
`mecanum_drive_controller/reference`**——绕过它就绕过了这个符号修正。

薄封装（只做清理 + `exec ros2 launch`，接线一律回 launch）：
`scripts/start_road_follow.sh`（单机）、`scripts/start_road_mppi_sim.sh`（桌面夹具）、
`scripts/start_road_real.sh` + `scripts/stop_road_real.sh`（真机/急停）。

关键参数在 `config/vision_follow_params.yaml`（覆盖层，**不改** `nav2_params.yaml`）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `angular_dist_threshold` | 0.785 (45°) | 路径朝向与车头差超过它就**先原地转** |
| `angular_disengage_threshold` | 0.35 | 转到这个夹角内才交还 MPPI（迟滞） |
| `rotate_to_heading_angular_vel` | 0.8 | 原地转角速度 |
| `use_path_orientations` | false | 视觉路径的 `PoseStamped` 全是 `w=1`，当朝向读会得到 0 |

MPPI 那半是 `nav2_params.yaml` 的**逐字镜像**（旋转闸门按同名节点把参数交给副驾，
没有独立命名空间）。两边漂了会被 `test_vision_follow_params.py` 抓到。

**Twist → TwistStamped 别自己写**：`mecanum_drive_controller` 的 `~/reference`
要 `geometry_msgs/TwistStamped`，板上没装 `twist_mux`，装 `ros-jazzy-twist-stamper`
用它即可。轻量档的 `road_follow_node` 则直接双发，不依赖这个包。

## 3. 降级档：轻量直发（无 Nav2、无 TF）

```bash
ros2 launch rak_bringup road_follow.launch.py role:=logic enabled:=true
```

不背 Nav2 的 12+ 进程、不需要 `odom -> base_footprint`。代价：跟踪质量就是手写
P 控制的水平，弯角靠 `follow_command` 的横移+偏航硬凑。**这不是主用档**，只在
「Nav2 起不来 / 只想看一眼 mask 和折线」时用。

## 4. 判别层到底改了什么（三个症状）

### 4.1 转弯转不过去

- 现成件：`nav2_rotation_shim_controller` 在路径朝向和车头差 > 45° 时先原地转。
- 前提是**路径本身要拐**。旧 `trace_centerline` 在弯角被 `wide_run_frac=0.45`
  判成"路口，保持航向"，把直角弯拉直成一条直线 —— 那样 shim 也救不了。
- 现在：trace 提前中断时调 `corner_signal()`，再调 `_corner_arm()` 把出射臂续进
  折线。路径拐了，shim 就会转。

### 4.2 直角弯分不清左右

- 生态里**没有**包能从 mask 判断"路往哪拐"，这一段必须自己写。
- `corner_signal()`：只在这条 trace **提前中断**（没到画面上部）时才看上方波段的
  路面质量落在哪侧 —— 这个前置条件保证缓弯不会被误判成直角弯。
- 对称的 T 形路口返回 `ambiguous`，`auto` **不猜**（宁可不转也不乱拐）。
  固定赛道请用 `nav2_route`，或显式 `turn_policy:=left|right`。
- `turn_policy:=left` 在没有左侧路面的地方**不会凭空造路**（有测试钉住）。

### 4.3 偏航、走不到路中间

- 现成件：MPPI 是轨迹优化器，**手写 P 控制的稳态误差在它那里不成立**。
- 轻量档仍然是 P，所以这一症状在降级档里依旧存在——这是选它的代价。
- 另一个真实原因**现在就该修**：板上 `~/.ros/camera_info/front_camera.yaml` 是
  1280×720 标的，驱动跑 640×480，且没传 `camera_info_url` → **畸变完全没矫正**。
  用 `image_proc`（apt）接上标定即可；不改的话 `e` 会有一个随位置变化的偏差，
  积分项也压不住。

## 5. 实测（2026-11-19，真机链路）

| 项 | 结果 |
|---|---|
| 板上分割 | 22–29 FPS，`road=20–57%` |
| mask 跨 WiFi 到工位 | **18–27 Hz**（640×480 mono8） |
| 板上分割 | 26–29 FPS（矫正链 26.4 FPS） |
| 轻量档控制 | 20.0 Hz，`std dev 0.0001 s` |
| RViz | overlay / road_mask / centerline_debug / vision_path 四路 match |
| 单元测试 | `test_road_follow.py` 34 passed |
| 参数漂移门禁 | `test_vision_follow_params.py` 3 passed |
| 结构 / 文档门禁 | `check_workspace.py` OK、`check_docs.py` 通过 |

## 6. 赛道几何与相机标定（2026-11-19，由比赛地图反解）

地图 `0.地图文件.pdf` 内嵌的原图是 13605×10204 px，正好 **4.8 × 3.6 m** →
**0.3528 mm/px**；实测车道内部 282–287 px + 两侧黄线 ≈ 319 px ≈ **450 mm**，
与"车道含黄线约 45 cm"吻合。这张图的价值不是建图（用户已明确不需要），而是
**一个已知长度的标尺**。

用它反解相机外参（`scripts/fit_camera_mount.py`，网格搜索，无 scipy）：

| 参数 | 旧占位值 | 拟合值 | 残差 |
|---|---|---|---|
| `cam_height_m` | 0.30 | **0.347** | 380 px → **15–22 px** |
| `cam_pitch_rad` | 0.0 | **0.4825**（27.6° 下俯） | 同上 |

四个帧互相一致（h ±0.005 m、pitch ±0.3°）。**旧占位值把每个远点放在了错误距离上**，
这是 MPPI 路径"看着像路但用不了"的直接原因。

### 内参：640×480 是裁剪，不是缩放

同一场景在 1280×720 与 640×480 下做特征匹配，NCC 结果**0.988 vs 0.326**：
640×480 与 1280×720 **垂直视场相同、水平被居中裁剪**。所以

```
fx_640 = 844.40 × (480/720) = 562.9      ← 正确
fx_640 = 844.40 × (640/1280) = 422.2     ← 错 33%
```

`road_follow_math.intrinsics()` 按裁剪模型推导，并有回归测试钉住。直接跟线档
不用 `fx`，所以这个错**不会在那条链上暴露**——只在米制投影里静默缩小所有距离。

### 畸变矫正：默认关闭，且必须成对使用

k1=0.24 / k2=-1.44 不能忽略，但 `image_proc/rectify_node` 用 `P = K` 输出时
**会改变有效视场**：底行车道跨度 636 → 478 px，拟合出的高度从 0.347 漂到 0.52 m。
正确做法是用 `cv2.getOptimalNewCameraMatrix` 生成保视场的 P。在那之前：

- `rectify:=false`（默认）与当前 `GOAL_DEFAULTS` 的拟合值**成对**；
- `rectify:=true` 必须先重生成 camera_info 并重跑 `fit_camera_mount.py`。

`scripts/fit_camera_mount.py` 现在带**姿态质量门**：车没摆正（横向偏 > 6% 画幅
或航向斜 > 1.2e-3/行）直接拒绝该帧，宁可报错也不给一个"看着很真"的错数——
这个门禁是踩过坑加的（板子重启后车换了位置，同样代码给出 0.52 m / 34.4°）。

### 相机安装偏航（`lane_center_bias`）

相机不精确指向车体正前方时，车道中心在图像里有一个**恒定偏移**。不修的话控制器会一直
追这个偏移，最终**停在车道偏一侧**——这正是"走不到路中间"的残余。

实测（2026-11-19，车停在车道正中且摆正，6 帧极差 0.0016）：

| | 值 |
|---|---|
| trace 稳定段 `x_norm` | **+0.0320** |
| 相机偏航 | **≈2.1°** |
| 修正前 `vy` / `wz` | −0.0246 / −0.0192（持续右修） |
| 修正后 `vy` / `wz` | **+0.0042 / +0.0031** |

定义在 `road_follow_math.LANE_CENTER_BIAS`（**一个常量**，direct 与 nav2 两档共用，
防止两档对"车道中心在哪"各说各话）。launch 参数 `lane_center_bias` 可在不重编时覆盖。

**重标方法**：车停在直道正中、摆正 → 看 RViz 的 Centerline 显示（或读 trace）→
读稳定段的 `x_norm` → 填进 `lane_center_bias`。换相机或重新安装后必须重做。

### 控制参数的数据依据（2026-11-19，8 帧真实 mask）

赛道地毯是**印刷实景图**，图里本身就画着马路，分割器会把它们也点成路面——这些假
路面出现在画面**上部**。所以"前瞻点取多高"直接决定指令噪声：

| `lookahead_frac` | wz 标准差 | 前瞻点 x 标准差 |
|---|---|---|
| 0.20 | 0.0004 | 0.0006 |
| 0.40 | 0.0022 | 0.0031 |
| **0.50** | **0.0013** | **0.0018** |
| 0.60 | 0.0208 | 0.0296 |
| 0.70（原值） | 0.0195 | 0.0277 |

0.60 以上噪声跳 16 倍。取 **0.50**（约 0.9 m 前瞻），并把 `k_lateral` 从 0.7
提到 **1.4** 补回拉近前瞻损失的转向权威：折弯响应 +0.060（原 +0.073），而指令
噪声 σ 从 0.0195 降到 0.0025（**低 8 倍**）。

另有单帧尖峰（一帧丢结构会让 wz 甩到 6 倍），用 `cmd_median_window=3` 的中值
滤波压掉，代价一帧延迟。

## 7. 未验证 / 待办

- **Nav2 档没在真机上跑过**：需要 `chassis_bringup` 提供 `odom -> base_footprint`
  （EKF），目前没有这条 TF。控制层配置是离线门禁验证的，不是跑通验证。
- `corner_mass_ratio`(1.8) / `corner_arm_max_rows`(24) 只在合成 mask 上调过，
  要在赛道上回填。
- 相机外参需要用**摆正的车**再采一轮（现在这组是 4 帧一致，但仍建议复核）。
- 保视场的 camera_info + 矫正后的重拟合（见 §6）。
