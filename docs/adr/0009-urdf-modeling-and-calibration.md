# ADR-0009: 没有精确 CAD 时怎么建 URDF —— 参数化 + 图元近似 + 标定精化

- **状态**: 已接受
- **日期**: 2026-10-05
- **相关**: `src/rak_description/urdf/six_axis_sim.urdf.xacro` · `src/rak_description/urdf/properties.xacro` · `src/rak_description/urdf/macros/inertial_macros.xacro` · ADR-0008

---

## 背景

用户 2026-10-05：

> "URDF 和建模咋弄？**只能给参数**，另外整机物理尺寸参数能**量**，但**无法精确建模**。"
> "手眼标定用 **chArUco**。"

现状：

- 臂**尚未成形**，没有 CAD。
- 已有 `six_axis_sim.urdf.xacro`：**六轴关节链结构正确**，但文件头明确写着
  *"Simulation fixture only. Dimensions, masses and limits are invented test values."*
- `properties.xacro` 是**空的有意为之**：注释说明属性必须写在顶层 URDF，
  否则 `link_length:=0.40` 这类 CLI 覆盖够不到（spec §7.3 / 裁定 R4.1）。

---

## 决策

### D1. **不需要精确 CAD。** URDF 的规划用途只要运动学。

URDF 里的几何分三类，**精度要求完全不同**：

| 类别 | 用途 | 精度要求 | 来源 |
|---|---|---|---|
| **运动学**（joint origin / axis / limits） | **IK、规划、TF** | **必须准** —— 这是唯一真正影响规划的 | 量测 + 标定 |
| **碰撞体**（collision） | MoveIt 碰撞检查 | **保守近似即可**：稍大安全，**偏小危险** | 图元包络 |
| **惯量**（inertial） | 动力学 / Gazebo 仿真 | MoveIt **纯规划用不到** | 占位值可 |
| **视觉体**（visual） | 只在 RViz 里好看 | **与规划完全无关** | 图元或后补 STL |

→ **"无法精确建模"不构成障碍**：精度通过**标定**获得，不通过 CAD。

### D2. 碰撞体用**图元包络**，宁可稍大

连杆用 `box`/`cylinder`、关节用 `sphere` 包住实际外形即可（现有模型正是这么做的）。
**宁可稍大**：碰撞体偏小会让 MoveIt 规划出实际会撞的轨迹。
STL 网格是**视觉**的事，可以最后再补（`meshes/arm/README.md` 现在就是占位）。

### D3. 沿用现有的参数化约定

- 可调参数写成**顶层 URDF 的 `<xacro:property>`**，以便 CLI 覆盖（`link_length:=0.40`）。
  这是 R4.1 的裁定，**不要**把它们挪进 `properties.xacro`，否则覆盖失效。
- 关节限位用 `properties.xacro` 里的**宏**（`shoulder_pitch_lower_value` 等），
  这样覆盖限位不需要重定义宏。
- 惯量用 `macros/inertial_macros.xacro` 的 `box_inertial` / `cylinder_inertial`，
  **不要手写裸 `<inertia ixx=...>`**。

### D4. 迭代精化，而不是一次建对

```
① 量测粗值  →  ② MoveIt 能规划、RViz 看着对  →  ③ chArUco 手眼标定 + 关节零位标定  →  ④ 精化
```

第 ① 步就能让整条链路跑起来；**第 ③ 步才决定实际抓取精度**。

### D5. 手眼标定用 **chArUco**（用户指定）

- **chArUco = 棋盘格 + ArUco 标记**：比纯棋盘格鲁棒（部分遮挡仍能定位角点），
  且**同一块板可同时标定相机内参和外参**。
- **eye-in-hand**（相机装在 `tool0`）：标定板固定在工作区，机械臂走多个位姿采图，解 `AX = XB`。
- 工具：OpenCV `cv2.aruco.calibrateHandEye`；ROS 2 侧可用 `easy_handeye2` 等。
- **前提**：先标定**相机内参**（同一块 chArUco 板即可），并保证 `tool0` 的 TF 与 URDF 一致。

---

## 需要你提供的参数清单

按重要性排序 —— **只有第 1–3 项是阻塞 MoveIt 的**：

| # | 参数 | 说明 | 阻塞? |
|---|---|---|---|
| 1 | **每个关节的 origin**（相对上一关节的 xyz 偏移） | 即各连杆长度与横向偏置，卷尺可量 | **是** |
| 2 | **每个关节的 axis 方向** | 转动轴指向（现有模型是 z/y/y/z/y/x 的常规构型） | **是** |
| 3 | **每个关节的限位**（lower/upper） | 物理硬限位，量角器/目测 + 实测修正 | **是** |
| 4 | `tool0` 相对最后关节的偏移 | 法兰位置 | 否（可先估） |
| 5 | 夹爪：钳口开度范围、TCP 偏移 | 现模型为 prismatic 0–0.05 m | 否 |
| 6 | 各连杆质量（可选） | 只影响仿真，规划不需要 | 否 |
| 7 | 末端相机相对 `tool0` 的安装偏移 | **由 chArUco 标定直接得出**，不必手量 | 否 |

> 只要给出 1–3，就能把 `six_axis_sim.urdf.xacro` 的虚构值换成真值，
> MoveIt 立刻能规划。4–7 可以后补。

---

## 理由

1. **规划只需要运动学**：MoveIt 的 IK 与碰撞检查用的是 joint origin/axis 和 collision geometry，
   **视觉体与惯量都不参与**。把精力全放在第 1–3 项上。
2. **保守的碰撞体是安全的**：稍大会让规划更保守，稍小会让规划出错。所以"不精确"在**偏大**方向是安全的。
3. **标定比 CAD 便宜**：chArUco 标定只需一块打印的板和一段采集流程，
   得到的外参精度直接反映真实装配，**比 CAD 更可信**（CAD 与实际装配总有偏差）。
4. **现有骨架省事**：六轴关节链、命名、轴向都已就位，改数值即可。

---

## 后果

### 正面

- **不需要 CAD 也能开工** —— 给参数就能推进，不被机械设计阻塞。
- 精度来源明确：标定（而非建模）。
- 复用现有 `six_axis_sim.urdf.xacro` 结构，改动集中在数值。

### 负面（代价，必须承认）

- **运动学参数测不准 → IK 会系统性地偏**。卷尺量连杆长度有 mm 级误差，
  最终要靠标定修正；在此之前抓取精度不可信。
- **关节零位**（舵机上电位置 ↔ URDF 的零位）**必须单独标定**，否则整个 TF 树是偏的。
  这与手眼标定是**两件事**，都要做。
- **视觉几何不精确**会让 RViz 里的画面与实际不符，调试时容易误判（**但这是观感问题，不是规划问题**）。
- 碰撞体偏保守会**缩小可行的规划空间**（某些贴边姿态被误判为碰撞）。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| 等 CAD 出来再建 URDF | 不必要地阻塞；规划只需要运动学，参数足够 |
| 用手写裸 `<inertia>` 精确建模 | 规划用不到惯量；且现有 `inertial_macros.xacro` 已规定用宏 |
| 把参数塞进 `properties.xacro` | 会破坏 CLI 覆盖（R4.1 已裁定） |
| 手量末端相机外参 | chArUco 标定更准，且顺带标定内参 |
| 纯棋盘格标定 | 用户指定 chArUco；chArUco 对遮挡更鲁棒，且能同时标内参 |

---

## 待闭合项

1. **第 1–3 项参数未提供**（关节 origin / axis / limits）—— 阻塞 MoveIt 重建。
2. **关节零位标定流程未定**（与手眼标定是两件事）。
3. **chArUco 板规格未定**（格数、方格边长、ArUco 字典、打印精度）。
4. **末端相机型号未定** → 内参标定与 URDF link 都待定（ADR-0008）。
5. **`six_axis_sim.urdf.xacro` 的定位**：它是"仿真夹具"，重建后应改名/合并，
   与真实模型区分开（现有五轴 `rak.urdf.xacro` 是更早的 demo）。

---

## 参考

- `src/rak_description/urdf/six_axis_sim.urdf.xacro`（六轴骨架，数值虚构）
- `src/rak_description/urdf/properties.xacro`（R4.1：属性必须在顶层才能 CLI 覆盖）
- `src/rak_description/urdf/macros/inertial_macros.xacro`（惯量宏，禁止裸 inertia）
- `docs/superpowers/specs/2026-10-03-rak-moveit2-integration-design.md` §7.2–7.3（连杆表与参数化策略）
- OpenCV chArUco + `cv2.aruco.calibrateHandEye`（eye-in-hand 的 `AX=XB`）
