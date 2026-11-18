#!/usr/bin/env python3
"""rf2o 坐标系语义审计（P1-2）—— 纯 Python stdlib，不启动 ROS、不依赖真机。

背景（2026-11-18 实测 + vendor/rf2o 源码盘点）：
  rf2o 发布的 /odom 位姿不自洽：0.453 m 直行时位移方向 +37.9°，而
  |lidar_yaw| = 37.59°（URDF laser_mount_joint = -0.6561 rad），量值相等、
  符号相反 —— 即位移仍处在**激光系**（mount 旋转没有被还原到 base 系）；
  绝对航向差 177°，但航向"变化量"是准的。

数学模型（共轭关系，本脚本用 2x2 旋转矩阵直接演示）：
  rf2o 内部 robot_pose_ = laser_pose_ ∘ mount⁻¹。设 mount 旋转 = α：
  - 纯平移时激光系位移 = R(-α)·d_base —— 方向误差恰为 -α（本例 +37.59°）；
  - 旋转共轭不变：M⁻¹·Rot(θ)·M = Rot(θ) —— 航向"变化量"准的来源；
  - 纯旋转下 base 位姿 = M·L·M⁻¹ 会多出圆弧项 (I-Rot(θ))·t_m（t_m = 雷达
    挂载平移，laser_x=0.11, y=0；模长 = 2·|t_m|·sin(θ/2)）：90° 转向 ≈ 0.16 m
    —— 这就是静态偏置（launch 里换中间帧 / 改 base_frame_id）修不了旋转的
    量化依据。

判据：
  位移方向误差 e = 发布位移方向 − base 系真实位移方向：
      e ≈ -lidar_yaw  → 位移在激光系（本例实测 +37.9° ≈ +37.59°，命中）
      e ≈ 0           → 位移在 base 系（健康）
      其余            → unknown（先查雷达安装/驱动坐标系）
  绝对航向偏移 δ = 发布 yaw − base 真实 yaw（同一时刻）：
      δ ≈ 0°        → 绝对 yaw 可用
      δ ≈ ±180°    → 疑似航向轴翻转；差分增量的**符号**存疑，必须旋转测试裁决
      其余          → 常数安装偏移：绝对 yaw 不可融，差分可用（偏移自动消掉）
  旋转测试（真机）：原地 +90°，比较 rf2o yaw 变化量与 IMU vyaw 积分：
      同号同量级 → 差分 yaw 可开（config/ekf_rf2o_yaw_only.yaml）
      反号       → rf2o 航向彻底不可用（默认档保持 odom0 全关）

用法：
  自检（内置 2026-11-18 实测数据，assert 全绿即判据自洽）：
      python3 src/rak_bringup/scripts/rf2o_frame_audit.py
  拿实测数据出结论：
      python3 src/rak_bringup/scripts/rf2o_frame_audit.py \
          --direction-err-deg 37.9 --yaw-offset-deg 177 --rotation-test untested
  可选：--lidar-yaw-deg（默认 -37.59）、--direction-deg/--expected-deg
  （给绝对方向而不是误差）、--json（机器可读输出）。
"""

import argparse
import json
import math
import sys

# 默认值 = 当前 URDF 事实（rak_chassis.urdf.xacro：laser_mount_joint rpy z =
# lidar_yaw = -0.6561 rad；laser_x = 0.11，y = 0）。
DEFAULT_LIDAR_YAW_DEG = -37.59
DEFAULT_MOUNT_XY = (0.11, 0.0)

TRANSLATION_TOL_DEG = 5.0   # 位移方向误差的判据容差（实测 37.9 vs 模型 37.59）
YAW_FLIP_TOL_DEG = 10.0     # ±180° 翻转判据容差


def rot2(deg: float):
    """2x2 旋转矩阵（度）。"""
    t = math.radians(deg)
    return ((math.cos(t), -math.sin(t)), (math.sin(t), math.cos(t)))


def mat_vec(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1], m[1][0] * v[0] + m[1][1] * v[1])


def angle_of(v) -> float:
    """向量方向角（度，[-180, 180)）。"""
    return math.degrees(math.atan2(v[1], v[0]))


def wrap180(deg: float) -> float:
    """归一化到 [-180, 180)。"""
    return (deg + 180.0) % 360.0 - 180.0


def classify_translation(dir_err_deg: float, lidar_yaw_deg: float,
                         tol: float = TRANSLATION_TOL_DEG) -> str:
    """位移方向误差 → 'laser_frame' | 'base_frame' | 'unknown'。"""
    model = wrap180(-lidar_yaw_deg)  # 激光系泄漏的预测方向误差
    if abs(wrap180(dir_err_deg)) <= tol:
        return "base_frame"
    if abs(wrap180(dir_err_deg - model)) <= tol:
        return "laser_frame"
    return "unknown"


def classify_yaw(yaw_offset_deg: float, tol: float = YAW_FLIP_TOL_DEG) -> str:
    """绝对航向偏移 → 'aligned' | 'anti'（≈±180°） | 'offset'（常数安装偏） | 'unknown'。"""
    d = wrap180(yaw_offset_deg)
    if abs(d) <= tol:
        return "aligned"
    if abs(abs(d) - 180.0) <= tol:
        return "anti"
    return "offset"


def laser_frame_direction(lidar_yaw_deg: float, base_displacement=(1.0, 0.0)):
    """共轭模型预测：纯平移 d_base 在激光系坐标里的方向（= R(-lidar_yaw)·d）。"""
    return mat_vec(rot2(-lidar_yaw_deg), base_displacement)


def rotation_arc_error(mount_xy, turn_deg: float) -> float:
    """纯旋转 θ 时静态偏置修不掉的圆弧项模长 |(I - R(θ))·t_m|。"""
    t = rot2(turn_deg)
    fixed = (t[0][0] * mount_xy[0] + t[0][1] * mount_xy[1],
             t[1][0] * mount_xy[0] + t[1][1] * mount_xy[1])
    return math.hypot(mount_xy[0] - fixed[0], mount_xy[1] - fixed[1])


def recommend(trans_class: str, yaw_class: str, rotation_test: str) -> dict:
    """组合三类证据 → 结论与配置建议（ekf.yaml / ekf_rf2o_yaw_only.yaml）。"""
    fusion = {"odom0_x_y": False, "odom0_yaw": False, "mode": "default_off",
              "file": "config/ekf.yaml (默认档, odom0 全关)", "note": ""}
    if trans_class == "base_frame" and yaw_class == "aligned":
        fusion = {"odom0_x_y": True, "odom0_yaw": True, "mode": "healthy",
                  "file": "可在 ekf.yaml 恢复 odom0 位姿融合（先复核协方差）",
                  "note": "两路判据都过：rf2o 位姿在 base 系且航向对齐。"}
    elif yaw_class in ("anti", "offset"):
        if rotation_test == "same":
            fusion = {"odom0_x_y": False, "odom0_yaw": True, "mode": "yaw_differential_ok",
                      "file": "config/ekf_rf2o_yaw_only.yaml（经 ekf_params 切换）",
                      "note": "旋转测试同号同量级：常数偏移/翻转下差分航向可用；"
                              "x/y 因位移在激光系（或未验证）保持关闭。"}
        elif rotation_test == "flipped":
            fusion["note"] = ("旋转测试反号：差分增量符号是反的（177° 实为 -183°），"
                              "rf2o 航向彻底不可用，保持默认档 odom0 全关。")
        else:
            fusion["note"] = ("待测量裁决：需真机原地 +90° 旋转测试分辨差分符号"
                              "（177° 无法纸面区分 +177 与 -183）。")
    elif trans_class == "laser_frame":
        fusion["note"] = ("位移在激光系（方向误差 ≈ -lidar_yaw）：x/y/vx/vy 不可融；"
                          "航向判据未命中已知档，先核对 yaw 参考系再裁决。")
    else:
        fusion["note"] = ("判据均未命中：检查雷达安装角是否与 URDF laser_mount_joint "
                          "一致、驱动 scan 的 frame_id 是否 laser_link。")
    return fusion


def self_test() -> None:
    """内置 2026-11-18 实测数据（0.453 m 直行）+ 共轭模型断言，全部必须绿。"""
    lidar_yaw = DEFAULT_LIDAR_YAW_DEG

    # 1) 共轭模型：纯平移在激光系的方向 = R(-lidar_yaw)·d → 预测 +37.59°
    v = laser_frame_direction(lidar_yaw)
    predicted = angle_of(v)
    assert abs(wrap180(predicted - (-lidar_yaw))) < 1e-6, predicted

    # 2) 模型预测 vs 实测：+37.59° vs +37.9°（差 0.31°，远小于容差）——命中
    #    "位移在激光系"的判据。
    measured_err = 37.9
    assert abs(wrap180(measured_err - predicted)) < 1.0

    # 3) 分类器：实测误差 → laser_frame；健康数据 → base_frame；乱数 → unknown
    assert classify_translation(measured_err, lidar_yaw) == "laser_frame"
    assert classify_translation(0.5, lidar_yaw) == "base_frame"
    assert classify_translation(90.0, lidar_yaw) == "unknown"

    # 4) 旋转共轭不变（航向变化量准的来源）+ 圆弧项量级（雷达挂载 0.11, 0）
    assert abs(rotation_arc_error((0.0, 0.0), 90.0)) < 1e-9      # 雷达在轴上无弧项
    arc = rotation_arc_error(DEFAULT_MOUNT_XY, 90.0)
    assert 0.14 < arc < 0.17, arc                                # 2·|t_m|·sin(45°) ≈ 0.156 m @ 90°
    assert rotation_arc_error(DEFAULT_MOUNT_XY, 0.0) < 1e-9      # 不转就没有弧项

    # 5) 航向分类：实测 177° → anti（翻转嫌疑）；0° → aligned；小常数 → offset
    assert classify_yaw(177.0) == "anti"
    assert classify_yaw(2.0) == "aligned"
    assert classify_yaw(30.0) == "offset"

    # 6) 建议矩阵：实测组合 → 默认档全关 + 待旋转测试；翻转确认 → 保持全关
    rec = recommend("laser_frame", "anti", "untested")
    assert rec["odom0_x_y"] is False and rec["odom0_yaw"] is False
    assert "待测量裁决" in rec["note"]
    assert recommend("laser_frame", "anti", "flipped")["odom0_yaw"] is False
    assert recommend("laser_frame", "anti", "same")["odom0_yaw"] is True
    assert recommend("laser_frame", "anti", "same")["file"].startswith("config/ekf_rf2o_yaw_only.yaml")
    assert recommend("base_frame", "aligned", "untested")["mode"] == "healthy"

    print("[self-test] 7/7 组断言全绿：判据、共轭模型与 2026-11-18 实测一致。")
    print("[self-test] 结论复述：位移在激光系（+37.9° ≈ -lidar_yaw），绝对航向差 177°；"
          "默认档 odom0 全关，yaw 差分待真机旋转测试裁决。")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="rf2o 坐标系语义审计（P1-2）：给实测数据，出判定与 EKF 配置建议。")
    ap.add_argument("--lidar-yaw-deg", type=float, default=DEFAULT_LIDAR_YAW_DEG,
                    help="URDF laser_mount_joint 的 yaw（度，默认 -37.59）")
    ap.add_argument("--direction-err-deg", type=float, default=None,
                    help="实测位移方向误差（度）= 发布位移方向 − base 真实方向")
    ap.add_argument("--direction-deg", type=float, default=None,
                    help="实测发布位移方向（度）；直行时配 --expected-deg（默认 0）")
    ap.add_argument("--expected-deg", type=float, default=0.0,
                    help="直行期望的 base 系位移方向（度，默认 0 = 正前）")
    ap.add_argument("--yaw-offset-deg", type=float, default=None,
                    help="实测绝对航向偏移（度）= 发布 yaw − base 真实 yaw")
    ap.add_argument("--rotation-test", choices=["same", "flipped", "untested"],
                    default="untested", help="原地 +90° 旋转测试结果（rf2o vs IMU）")
    ap.add_argument("--json", action="store_true", help="输出机器可读 JSON")
    args = ap.parse_args()

    if args.direction_err_deg is None and args.direction_deg is None \
            and args.yaw_offset_deg is None:
        self_test()
        return 0

    if args.direction_err_deg is None:
        base_dir = args.direction_deg if args.direction_deg is not None else 0.0
        dir_err = wrap180(base_dir - args.expected_deg)
    else:
        dir_err = wrap180(args.direction_err_deg)

    trans = classify_translation(dir_err, args.lidar_yaw_deg)
    yawc = classify_yaw(args.yaw_offset_deg) if args.yaw_offset_deg is not None \
        else "unknown"
    rec = recommend(trans, yawc, args.rotation_test)
    out = {
        "lidar_yaw_deg": args.lidar_yaw_deg,
        "translation": {"dir_err_deg": dir_err, "class": trans,
                        "model_laser_frame_deg": wrap180(-args.lidar_yaw_deg)},
        "yaw": {"offset_deg": args.yaw_offset_deg, "class": yawc},
        "rotation_test": args.rotation_test,
        "recommendation": rec,
        "next_measurement": ("原地 +90° 旋转测试：rf2o yaw 变化量 vs /imu/data_raw "
                             "vyaw 积分，同号=切 ekf_rf2o_yaw_only.yaml，反号=保持全关")
                            if yawc == "anti" and args.rotation_test == "untested" else None,
    }
    if args.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0

    print(f"位移方向误差 {dir_err:+.1f}° (模型激光系预测 {out['translation']['model_laser_frame_deg']:+.1f}°)"
          f" → {trans}")
    if args.yaw_offset_deg is not None:
        print(f"绝对航向偏移 {args.yaw_offset_deg:+.1f}° → {yawc}")
    print(f"旋转测试: {args.rotation_test}")
    print(f"建议: {rec['file']}")
    if rec["note"]:
        print(f"说明: {rec['note']}")
    if out["next_measurement"]:
        print(f"下一步实测: {out['next_measurement']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
