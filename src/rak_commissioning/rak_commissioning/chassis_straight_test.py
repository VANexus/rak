#!/usr/bin/env python3
"""[AI] 直走位移标定（G4）：走一段直线，用三种独立方法量出真实走了多远。

为什么不是 chassis_forward_test：那个只证明"轮子转了、方向一致"（G3），量的是
10 s 内的轮速反馈。判"歪不歪""走了多少"必须用外部参考，否则闭环误差无从量化。

三个测量源（互不信任，交叉验证）：
  1. 轮系航位推算：/joint_states 四轮速度 × 轮半径积分（闭环反馈，受滑移影响）；
  2. 激光里程计：/odom（rf2o，扫描匹配，与本车运动学无关）；
  3. 深度相机：正前方墙面距离变化（Gemini335L 正对车前，直接量接近量）。
另外记录 rf2o 的航向变化 = 直走偏航（"很歪"的量化指标）与四轮速差。
报告末尾输出有效轮半径标定（雷达真值 / 轮系转角 θ）与四源换算一致性。

安全门（任一触发立刻归零退出，退出码 1）：
  - 前方净空 < min_clearance（默认 0.30 m）；
  - 开步 5 s 后四轮反馈仍全零（allow_motion=false 或未使能）；
  - 深度/雷达都拿不到净空数据（宁可不动）。

用法（bringup 已起、G0–G3 已过、前方净空 > distance + min_clearance）：
  ros2 run rak_commissioning chassis_straight_test                     # 预演，不动
  ros2 run rak_commissioning chassis_straight_test -- --yes            # 默认 0.6 m
  ros2 run rak_commissioning chassis_straight_test -- --yes -p distance:=1.0
参数: distance(m, 默认 0.6) speed(m/s, 默认 0.08) ramp(s, 默认 1.0)
      min_clearance(m, 默认 0.30) topic debug verbose(逐段详志, 默认关)
"""
import math
import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, JointState, LaserScan
from tf2_ros import Buffer, TransformListener

WHEELS = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]
WHEEL_LABELS = ["FL", "FR", "RL", "RR"]
DEFAULT_TOPIC = "/mecanum_drive_controller/reference"
WHEEL_RADIUS = 0.0275  # 与 chassis_controllers.yaml kinematics.wheels_radius 同源
ODOM_FRAMES = ("odom", "base_footprint")  # rf2o 应发布的 frame_id / child_frame_id
ODOM_MAX_STEP = 0.5      # /odom 单拍位移上限（m），超过视为坏帧剔除


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StraightTest(Node):

    def __init__(self):
        super().__init__("chassis_straight_test")
        self.declare_parameter("topic", DEFAULT_TOPIC)
        self.declare_parameter("distance", 0.6)
        self.declare_parameter("speed", 0.08)
        self.declare_parameter("ramp", 1.0)
        self.declare_parameter("min_clearance", 0.30)
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("odom_topic", "/odom")
        # --yes 之外的第二条真实运动许可（ros2 run 会把 -- 后的 -p 吞掉，见 §用法）
        self.declare_parameter("go", False)
        self.declare_parameter("debug", False)
        self.declare_parameter("verbose", False)
        # 行进方向：False=前进（深度相机正对方向），True=后退。
        self.declare_parameter("backward", False)
        # 净空门用雷达（360°，前进/后退都覆盖）；深度相机只做正前方交叉验证。
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("gate_sector", 25.0)
        self.distance = float(self.get_parameter("distance").value)
        self.speed = float(self.get_parameter("speed").value)
        self.ramp = max(0.0, float(self.get_parameter("ramp").value))
        self.min_clearance = float(self.get_parameter("min_clearance").value)
        self.backward = bool(self.get_parameter("backward").value)
        self.gate_sector = float(self.get_parameter("gate_sector").value)
        self.dir_sign = -1.0 if self.backward else 1.0
        if self.distance <= 0.0 or not 0.0 < self.speed <= 0.15:
            raise SystemExit("distance 必须 >0；speed 取 (0, 0.15] m/s（未标定保守值）")
        if self.ramp < 0.2:
            raise SystemExit("ramp 至少 0.2 s：直给阶跃会激发轮系冲击")

        self.pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("topic").value), QoSProfile(depth=10))
        self.wheel_vel = [None] * 4
        self.joints_ts = None
        self.create_subscription(JointState, "/joint_states", self._on_joints, QoSProfile(depth=10))

        self.odom = None            # (x, y, yaw) 最近一帧（已过合理性闸门）
        self.odom_count = 0
        self.odom_reject = 0
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self._on_odom, QoSProfile(depth=10))

        self.scan = None
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value),
                                 lambda m: setattr(self, "scan", m), QoSProfile(depth=2))
        self.clearance = None       # 正前方墙面距离（深度相机）
        self.depth_count = 0
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value),
                                 self._on_depth, QoSProfile(depth=2))

        self._zero()

    # ---------- 订阅 ----------
    def _on_joints(self, msg):
        self.joints_ts = time.monotonic()
        for i, name in enumerate(WHEELS):
            if name in msg.name:
                j = msg.name.index(name)
                if j < len(msg.velocity):
                    self.wheel_vel[i] = msg.velocity[j]

    def _on_odom(self, msg):
        # 合理性闸门：帧名不符或单拍位移超限的样本直接剔除，计数进报告
        if msg.header.frame_id != ODOM_FRAMES[0] or msg.child_frame_id != ODOM_FRAMES[1]:
            self.odom_reject += 1
            return
        p = msg.pose.pose
        pos = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.odom is not None:
            if math.hypot(pos[0] - self.odom[0], pos[1] - self.odom[1]) > ODOM_MAX_STEP:
                self.odom_reject += 1
                return
        self.odom = pos
        self.odom_count += 1

    def _on_depth(self, msg):
        if msg.encoding != "16UC1":
            return
        a = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, -1)
        h, w = a.shape[0], msg.width
        band = a[h // 2 - 40:h // 2 + 40, w // 2 - 60:w // 2 + 60].astype(np.float64) / 1000.0
        band = band[band > 0.05]
        if band.size >= 200:
            self.clearance = float(np.median(band))
            self.depth_count += 1

    # ---------- 工具 ----------
    def travel_clearance(self):
        """行进方向上的最近障碍（base_link 系，雷达 ±gate_sector）。None=无数据。"""
        if self.scan is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform("base_link", self.scan.header.frame_id,
                                                rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        s = self.scan
        a = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment + yaw
        b = (np.degrees(a) + 180.0) % 360.0 - 180.0
        if self.backward:
            b = (b - 180.0 + 180.0) % 360.0 - 180.0   # 折到车尾方向
        r = np.asarray(s.ranges)
        m = (np.abs(b) <= self.gate_sector) & np.isfinite(r) & (r > s.range_min)
        if not m.any():
            return None
        return float(r[m].min())

    def feedback_fresh(self, within=1.0):
        return (self.joints_ts is not None
                and time.monotonic() - self.joints_ts <= within
                and all(v is not None for v in self.wheel_vel))

    def _cmd(self, x):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = x * self.dir_sign
        self.pub.publish(msg)

    def _zero(self):
        self._cmd(0.0)

    def spin(self, seconds, tick=0.05):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=tick)

    def wait_for(self, predicate, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False


def main(args=None):
    """命令行入口。"""
    rclpy.init()
    node = StraightTest()
    log = node.get_logger()

    go = "--yes" in (sys.argv[1:] or []) or bool(node.get_parameter("go").value)
    debug = bool(node.get_parameter("debug").value)
    verbose = bool(node.get_parameter("verbose").value)
    node.spin(0.5)

    # 距离参数化梯形：T = ramp + 匀速段 + ramp；距离太短时退化为三角形
    accel = node.speed / node.ramp
    if node.distance <= accel * node.ramp ** 2:
        v_peak = math.sqrt(accel * node.distance)
        T = 2.0 * v_peak / accel
    else:
        v_peak = node.speed
        T = node.ramp + (node.distance - accel * node.ramp ** 2) / node.speed + node.ramp

    # 前置：反馈与外部参考都要就位
    if not node.wait_for(node.feedback_fresh, 3.0):
        log.error("拒绝执行：/joint_states 无四轮速度反馈（bringup 未起 / 底盘不在总线）")
        node._zero(); node.destroy_node(); rclpy.shutdown()
        return 1
    if not node.wait_for(lambda: node.travel_clearance() is not None, 3.0):
        log.error("拒绝执行：雷达拿不到行进方向净空。直走测试必须有外部安全判据。")
        node._zero(); node.destroy_node(); rclpy.shutdown()
        return 1
    log.info("行进方向 = %s；雷达净空 %.3f m（±%.0f°），深度正前方 %.3f m" % (
        "后退" if node.backward else "前进", node.travel_clearance(), node.gate_sector,
        node.clearance if node.clearance is not None else -1.0))
    if not node.odom_count:
        log.warn("/odom 无数据：激光里程计没起，位移只能用轮系反馈量。")

    if not go:
        log.info("预演模式（未给 --yes，不运动）。目标 %.2f m @ %.3f m/s，梯形 T=%.1f s。"
                 % (node.distance, node.speed, T))
        node._zero(); node.destroy_node(); rclpy.shutdown()
        return 0

    stop = False

    def on_sig(_s, _f):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    start_clearance = node.clearance
    start_travel = node.travel_clearance()
    odom0 = node.odom
    # 轮系航位推算：每一拍都用最新样本积分（不再受 feedback_fresh 门控）；
    # 符号约定：正 = 前进（ROS 反馈约定），与行进方向无关。
    wheel_odo = 0.0            # ∫ mean(v)·R dt
    wheel_sum = [0.0] * 4      # ∫ v_i dt（rad，半径折算前的原始转角）
    wheel_travel = [0.0] * 4   # ∫ v_i·R dt（m，单轮里程）
    peak = [0.0] * 4
    collect = True             # 运动段才累积统计；收尾零速段不计入
    last_t = time.monotonic()

    def step():
        """推进一步：轮系积分（每拍最新样本）+ 安全门。True=继续。"""
        nonlocal wheel_odo, wheel_sum, wheel_travel, peak, last_t, stop, collect
        now = time.monotonic()
        dt = now - last_t
        last_t = now
        if collect and dt < 0.5:
            vs = [0.0 if w is None else w for w in node.wheel_vel]
            wheel_odo += sum(vs) / 4.0 * WHEEL_RADIUS * dt
            for i in range(4):
                wheel_sum[i] += vs[i] * dt
                wheel_travel[i] += vs[i] * WHEEL_RADIUS * dt
                peak[i] = max(peak[i], abs(vs[i]))
        clr = node.travel_clearance()
        if clr is not None and clr < node.min_clearance:
            log.error("行进方向净空 %.3f m < %.2f m -> 归零退出（防撞门）"
                      % (clr, node.min_clearance))
            stop = True
            return False
        return True

    rate = 20.0
    period = 1.0 / rate
    covered = 0.0
    elapsed = 0.0
    last_phase = None
    log.info("开步：%.2f m @ %.3f m/s，加速 %.1f s，梯形总时长 T=%.1f s"
             % (node.distance, node.speed, node.ramp, T))
    try:
        # 距离参数化梯形：v(t) = clamp(accel·t, v_peak, accel·(T-t))，按时间走完全程
        while not stop and elapsed < T:
            t0 = time.monotonic()
            va = accel * elapsed
            vd = accel * (T - elapsed)
            v = max(0.0, min(va, v_peak, vd))
            phase = ("加速" if va <= min(v_peak, vd)
                     else "减速" if vd <= min(va, v_peak) else "匀速")
            node._cmd(v)
            rclpy.spin_once(node, timeout_sec=period)
            if not step():
                break
            covered = min(node.distance, covered + v * period)
            elapsed += period
            if verbose and phase != last_phase:
                log.info("[verbose] t=%.2f 进入%s段 v=%.3f covered=%.3f"
                         % (elapsed, phase, v, covered))
                last_phase = phase
            if debug and int(elapsed * 2) != int((elapsed - period) * 2):
                raw = "[" + ", ".join("None" if w is None else "%.3f" % w
                                      for w in node.wheel_vel) + "]"
                age = (time.monotonic() - node.joints_ts) if node.joints_ts is not None else -1.0
                clr = node.travel_clearance()
                log.info("t=%5.2f cmd=%+.3f raw=%s js_age=%.2f clr_travel=%s odom=%s" % (
                    elapsed, v, raw, age,
                    "None" if clr is None else "%.3f" % clr,
                    "None" if node.odom is None else str(tuple(round(x, 3) for x in node.odom))))
                if verbose:
                    log.info("[verbose] wheel_odo=%.3f theta=%.2f odom_n=%d odom_rej=%d depth_clr=%s"
                             % (wheel_odo, wheel_odo / WHEEL_RADIUS, node.odom_count,
                                node.odom_reject,
                                "None" if node.clearance is None else "%.3f" % node.clearance))
            if elapsed > 5.0 and peak == [0.0] * 4:
                log.error("开步 5 s 轮反馈仍全零 -> 归零退出（allow_motion? enable?）")
                stop = True
                break
            if elapsed > T + 5.0:
                log.error("超时保护 -> 归零退出")
                stop = True
                break
            slack = period - (time.monotonic() - t0)
            if slack > 0:
                time.sleep(slack)
    finally:
        # 减速后 1 s 零目标，再读一次净空（此段不计入轮系统计窗口）
        collect = False
        for _ in range(max(1, int(rate))):
            node._zero()
            rclpy.spin_once(node, timeout_sec=period)
            step()
        node._zero()
        node.spin(0.5)

    # ---------- 报告 ----------
    odom_disp = 0.0
    odom_dx = 0.0
    yaw_drift = 0.0
    odom_valid = odom0 is not None and node.odom is not None
    if odom_valid:
        dx = node.odom[0] - odom0[0]
        dy = node.odom[1] - odom0[1]
        odom_disp = math.hypot(dx, dy)
        c, s = math.cos(-odom0[2]), math.sin(-odom0[2])
        odom_dx = dx * c - dy * s           # 沿起始航向的位移
        yaw_drift = math.degrees(node.odom[2] - odom0[2])
        yaw_drift = (yaw_drift + 180.0) % 360.0 - 180.0
    end_travel = node.travel_clearance()
    travel_valid = start_travel is not None and end_travel is not None
    travel_move = (start_travel - end_travel) if travel_valid else 0.0
    wall_valid = start_clearance is not None and node.clearance is not None
    wall_move = (start_clearance - node.clearance) if wall_valid else 0.0

    log.info("=" * 66)
    log.info("指令位移        %+.3f m（%s %.3f m/s，梯形 T=%.1f s）" % (
        node.distance * node.dir_sign, "后退" if node.backward else "前进", node.speed, T))
    log.info("轮系推算位移    %+.3f m（每拍积分，正=前进；半径 %g）" % (wheel_odo, WHEEL_RADIUS))
    log.info("激光里程计位移  %+.3f m（沿起始航向 %+.3f m，rf2o /odom）%s" % (
        odom_disp, odom_dx, "" if odom_valid else "（无数据，按 0 计）"))
    log.info("odom 帧统计      接受 %d，剔除 %d（帧名≠%s/%s 或单拍位移>%.1f m）" % (
        node.odom_count, node.odom_reject, ODOM_FRAMES[0], ODOM_FRAMES[1], ODOM_MAX_STEP))
    log.info("雷达接近量      %+.3f m（行进方向：%.3f -> %.3f m；正=接近）%s" % (
        travel_move, start_travel if start_travel is not None else -1.0,
        end_travel if end_travel is not None else -1.0,
        "" if travel_valid else "（无数据，按 0 计）"))
    log.info("深度接近量      %+.3f m（正前方：%.3f -> %.3f m；后退时无意义）%s" % (
        wall_move, start_clearance if start_clearance is not None else -1.0,
        node.clearance if node.clearance is not None else -1.0,
        "" if wall_valid else "（无数据，按 0 计）"))
    log.info("航向漂移        %+.2f deg（rf2o，正=左转）%s" % (
        yaw_drift, "" if odom_valid else "（无数据，按 0 计）"))
    if odom_valid and odom_disp < 0.02 and travel_valid and abs(travel_move) > 0.05:
        log.warn("rf2o 疑似失效（激光里程计未跟上）：/odom 位移 %.3f m 而雷达 %.3f m" % (
            odom_disp, abs(travel_move)))
    if elapsed > 1e-9:
        mean_v = [wheel_sum[i] / elapsed for i in range(4)]
        log.info("四轮统计（均值 rad/s | 峰值 | 里程 m）  %s" % (
            "  ".join("%s %+.2f/%.2f/%.3f" % (WHEEL_LABELS[i], mean_v[i], peak[i],
                                              wheel_travel[i]) for i in range(4)),))
        active = [m for m in mean_v if abs(m) > 0.2]
        if active:
            log.info("四轮速差        %.2f rad/s（%d 轮 |均值|>0.2）：直走歪的直接来源" % (
                max(active) - min(active), len(active)))
        else:
            log.info("四轮速差        n/a（无轮 |均值| > 0.2 rad/s）")

    # 有效轮半径标定：r_eff = |雷达真值| / |θ|，θ = ∫ mean(v) dt（半径折算前）
    log.info("-- 有效轮半径标定 --")
    theta = wheel_odo / WHEEL_RADIUS
    if not travel_valid or abs(travel_move) < 1e-6:
        log.info("有效轮半径        无法标定（雷达真值无变化或无数据）")
    elif abs(theta) < 1e-6:
        log.info("有效轮半径        无法标定（轮系转角 θ≈0）")
    elif not 0.5 <= abs(wheel_odo) / abs(travel_move) <= 2.0:
        log.info("轮系/雷达 = %.2f 落在 [0.5,2.0] 外 -> 不可信：疑似轮速反馈异常或滑移（不给 r_eff）"
                 % (abs(wheel_odo) / abs(travel_move),))
    else:
        r_eff = abs(travel_move) / abs(theta)
        log.info("有效轮半径        %.3f m（雷达 %.3f m / θ %.2f rad；与标称 %.4f 偏差 %+.1f%%）" % (
            r_eff, abs(travel_move), abs(theta), WHEEL_RADIUS,
            100.0 * (r_eff - WHEEL_RADIUS) / WHEEL_RADIUS))
        if node.backward:
            log.info("深度第二真值      跳过（后退）")
        elif wall_valid and abs(wall_move) > 1e-6:
            r_eff_d = abs(wall_move) / abs(theta)
            log.info("深度第二真值      %.3f m（深度 %.3f m / θ %.2f rad；偏差 %+.1f%%）" % (
                r_eff_d, abs(wall_move), abs(theta),
                100.0 * (r_eff_d - WHEEL_RADIUS) / WHEEL_RADIUS))
        else:
            log.info("深度第二真值      无数据（深度相机未测得净空变化）")

    # 换算一致性：四源位移并列，一眼看出超调 / 欠走 / 反馈坏
    log.info("-- 换算一致性（比值 = 各源 |位移| / 雷达真值；>1 超调，<1 欠走）--")
    radar_ref = abs(travel_move) if travel_valid else 0.0

    def rat(x):
        return ("%.2f" % (x / radar_ref)) if radar_ref > 1e-6 else "n/a"

    log.info("指令  %+.3f m  比值 %s" % (node.distance * node.dir_sign, rat(float(node.distance))))
    log.info("轮系  %+.3f m  比值 %s" % (wheel_odo, rat(abs(wheel_odo))))
    log.info("雷达  %+.3f m  比值 %s%s" % (
        travel_move, rat(abs(travel_move)), "" if travel_valid else "（无数据）"))
    if node.backward:
        log.info("深度  跳过（后退，深度相机仅前进时作第二真值）")
    else:
        log.info("深度  %+.3f m  比值 %s%s" % (
            wall_move, rat(abs(wall_move)), "" if wall_valid else "（无数据）"))
    log.info("=" * 66)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if stop else 0


if __name__ == "__main__":
    sys.exit(main())
