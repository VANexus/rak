"""rak_wbc ROS 节点：100 Hz 全身运动仲裁（ADR-0011 D1/D5/D8，spec §8）。

[AI] 为什么存在：把数值层（model_builder + hqp）接到 ROS 图上，成为底盘与臂的
"唯一常规运动写入者"。它只做 I/O 与状态机，数学全部在数值层（可桌面回归）。

安全纪律（不可协商）：
- 默认 `enabled=false`：不发布任何运动命令；关闭时发一次零速后静默。
- 连续求解失败 ≥ fail_limit → `degraded`：停止发布（mecanum reference_timeout +
  臂 lease 兜底），需显式重新 enable；恢复不自动 ARM。
- 不读 /brain/decision、不解析自然语言；目标必须带 frame（经 TF 变换）。
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool
import tf2_ros

from rak_lifecycle.managed_node import ManagedNode
from rak_wbc.hqp import HqpSolver, WbcInput, WbcParams
from rak_wbc.model_builder import build_composite_urdf, WbcKinematics

PARAM_DEFAULTS = {
    "enabled": False,
    "mode": "idle",
    "rate_hz": 100.0,
    "world_frame": "map",
    "base_frame": "base_link",
    "cmd_vel_topic": "/cmd_vel",
    "cmd_timeout": 0.3,
    "ee_target_topic": "/wbc/ee_target",
    "ee_timeout": 0.5,
    "joint_states_topic": "/joint_states",
    "base_reference_topic": "/mecanum_drive_controller/reference",
    "arm_setpoints_topic": "/wbc/arm_setpoints",
    "status_topic": "/wbc/status",
    "publish_arm": False,
    "urdf_wbc_base": "",  # 空 = 解析本包 share/urdf/wbc_base.urdf.xacro
    "urdf_arm": "",  # 空 = 解析 rak_description share/urdf/rak_arm.urdf.xacro
    "arm_joint_names": [
        "j1_base",
        "j2_shoulder",
        "j3_elbow",
        "j4_forearm",
        "j5_wrist_pitch",
        "j6_wrist_roll",
    ],
    # [AI] 2026-10-16：改成**指令角域**（与 config/wbc_params.yaml 逐值相同）。
    # 旧值是 q_urdf 域，2026-10-15 统一角度域之后会让臂停在/走向错姿态
    # （home_q 会停到 J2=90° 而非 225°；arm_q_min 里的负角会被 Session 静默截 0）。
    # ⚠️ 这一组值在 yaml 与这里**各写了一遍** —— 改一处必须改两处。
    # （去重是另一个重构，见 /tmp 的审查记录，本轮不动结构。）
    # Home 安全位（ADR-0011 D4）= 固件指令 0/225/270/0/0/0°
    "home_q": [0.0, 3.9269908169872414, 4.71238898038469, 0.0, 0.0, 0.0],
    # 限位/速度：properties_arm.xacro 实测口径（TODO(measured) 带载回填）
    "arm_q_min": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "arm_q_max": [
        4.71238898038469,   # J1 270°
        4.4505895925855405,  # J2 255°（实测硬件限，不是 270°）
        4.71238898038469,   # J3 270°
        4.71238898038469,   # J4 270°
        4.71238898038469,   # J5 270°
        4.71238898038469,   # J6 270°
    ],
    "arm_qd_max": [1.57, 0.5, 0.5, 1.57, 0.5, 1.57],
    # 底盘速度盒：ADR-0006 记录的占位口径（第一道线在 mecanum 控制器参数）
    "base_vmax": [0.25, 0.15, 0.5],
    "wheel_radius": 0.0275,  # 2026-10-14 实测 Ø5.5（与 chassis_controllers.yaml 同步）
    "wheel_k": 0.26,  # hw+ht = 0.1225+0.1375（2026-10-14 实测）
    "wheel_wmax": 10.0,  # URDF command clamp 口径
    "support_half": [0.1225, 0.1375],  # 半轴距/半轮距（2026-10-14 实测）
    "support_margin": 0.02,
    # 权重档位（spec §5；P1:P2:P3 ≈ 100:10:0.2）
    "w_ee_pos": 100.0,
    "w_ee_rot": 20.0,
    "w_base_lin": 10.0,
    "w_base_yaw": 5.0,
    "w_posture": 0.2,
    "regularization": 1e-3,
    "kp_ee": 2.0,
    "kr_ee": 1.0,
    "k_post": 0.2,
    "k_joint": 3.0,
    "k_support": 1.5,
    "fail_limit": 3,
    "eps_abs": 1e-6,
    "eps_rel": 1e-6,
    "max_iter": 4000,
}

_MODES = ("idle", "nav", "servo", "hold")


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class WbcNode(ManagedNode):
    def __init__(self):
        super().__init__("wbc")

    def on_configure(self, state):
        """参数、复合 URDF、HQP 求解器与图上接口在这一装配（ADR-0020 §5）。

        100 Hz timer 刻意不在这一步：configure 成功只表示"这一层可用"，
        能不能写运动接口仍由 `enabled` + `allow_motion` 决定（§6：激活 ≠ 运动许可）。
        """
        super().on_configure(state)
        for name, value in PARAM_DEFAULTS.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self._world_frame = str(param("world_frame"))
        self._base_frame = str(param("base_frame"))
        self._rate_hz = float(param("rate_hz"))
        self._cmd_timeout = float(param("cmd_timeout"))
        self._ee_timeout = float(param("ee_timeout"))
        self._arm_stream_enabled = bool(param("publish_arm"))
        # 注意（桌面彩排抓到的 TypeError）：开关不能命名 self._publish_arm——会遮蔽同名方法
        self._fail_limit = int(param("fail_limit"))

        self._params = WbcParams.from_mapping(
            {
                "arm_q_min": param("arm_q_min"),
                "arm_q_max": param("arm_q_max"),
                "arm_qd_max": param("arm_qd_max"),
                "home_q": param("home_q"),
                "base_vmax": param("base_vmax"),
                "wheel_radius": param("wheel_radius"),
                "wheel_k": param("wheel_k"),
                "wheel_wmax": param("wheel_wmax"),
                "support_half": param("support_half"),
                "support_margin": param("support_margin"),
                "w_ee_pos": param("w_ee_pos"),
                "w_ee_rot": param("w_ee_rot"),
                "w_base_lin": param("w_base_lin"),
                "w_base_yaw": param("w_base_yaw"),
                "w_posture": param("w_posture"),
                "regularization": param("regularization"),
                "kp_ee": param("kp_ee"),
                "kr_ee": param("kr_ee"),
                "k_post": param("k_post"),
                "k_joint": param("k_joint"),
                "k_support": param("k_support"),
                "fail_limit": param("fail_limit"),
                "eps_abs": param("eps_abs"),
                "eps_rel": param("eps_rel"),
                "max_iter": param("max_iter"),
            }
        )
        try:
            self._kinematics = WbcKinematics(
                build_composite_urdf(
                    self._resolve(param("urdf_wbc_base"), "rak_wbc", "urdf/wbc_base.urdf.xacro"),
                    self._resolve(param("urdf_arm"), "rak_description", "urdf/rak_arm.urdf.xacro"),
                )
            )
            self._solver = HqpSolver(self._kinematics, self._params)
        except Exception as exc:  # noqa: BLE001 - 装配失败要变成可读的转移失败，不是崩溃
            return self.fail(f"复合 URDF / HQP 求解器装配失败：{exc!r}")
        self._arm_joint_names = [str(name) for name in param("arm_joint_names")]

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._enabled = bool(param("enabled"))
        self._mode = str(param("mode"))
        if self._mode not in _MODES:
            self.get_logger().warn(f"unknown mode {self._mode!r}; forcing 'idle'")
            self._mode = "idle"
        self._degraded = False
        self._arm_cmd = np.asarray(param("home_q"), dtype=float)
        self._q_arm = np.asarray(param("home_q"), dtype=float)
        self._cmd = np.zeros(3)
        self._cmd_stamp = 0.0
        self._ee_target = None
        self._ee_stamp = 0.0
        self._last_result = None
        self._tick_count = 0

        self._base_pub = self.lifecycle_publisher(
            TwistStamped, str(param("base_reference_topic")), 10
        )
        self._arm_pub = self.lifecycle_publisher(
            Float64MultiArray, str(param("arm_setpoints_topic")), 10
        )
        self._status_pub = self.lifecycle_publisher(String, str(param("status_topic")), 10)
        self.create_subscription(Twist, str(param("cmd_vel_topic")), self._on_cmd_vel, 10)
        self.create_subscription(PoseStamped, str(param("ee_target_topic")), self._on_ee_target, 10)
        self.create_subscription(String, "/wbc/mode", self._on_mode, 10)
        self.create_subscription(JointState, str(param("joint_states_topic")), self._on_joint_state, 10)
        self.create_service(SetBool, "/wbc/enable", self._on_enable)
        self.report(
            "configure",
            DiagnosticStatus.OK,
            f"enabled={self._enabled} mode={self._mode} publish_arm={self._arm_stream_enabled}",
            {"rate_hz": f"{self._rate_hz}"},
        )
        self.get_logger().info(
            f"rak_wbc configured: enabled={self._enabled} mode={self._mode} "
            f"publish_arm={self._arm_stream_enabled}；默认关闭，经 /wbc/enable 开启"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """激活 = 100 Hz 仲裁环开始跑；`enabled=False` 时它仍然一个命令都不发。"""
        super().on_activate(state)
        self.add_timer(1.0 / self._rate_hz, self._tick)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """停用 = 停环。**不在这里补发零速**：`on_deactivate` 阶段 LifecyclePublisher
        已经拒绝发布，硬发只会被丢弃而不是生效。真正的停车来自消费侧的
        `mecanum_drive_controller.reference_timeout` 与臂 lease —— 与 ADR-0019
        "失联兜底在控制器超时与下位机里，不在写者的退出路径上" 一致。
        """
        self._enabled = False
        super().on_deactivate(state)
        self.report(
            "degraded",
            DiagnosticStatus.WARN,
            "仲裁环已停用；底盘/臂由各自超时兜底",
        )
        return TransitionCallbackReturn.SUCCESS

    # ---- 工具 --------------------------------------------------------------

    @staticmethod
    def _resolve(configured: str, package: str, relative: str) -> str:
        if configured:
            return configured
        from ament_index_python.packages import get_package_share_directory

        return str(get_package_share_directory(package) + "/" + relative)

    def _lookup_base_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:
            return None
        rotation = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return np.array(
            [tf.transform.translation.x, tf.transform.translation.y, yaw], dtype=float
        )

    # ---- 订阅 --------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist):
        self._cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)
        self._cmd_stamp = time.monotonic()

    def _on_ee_target(self, msg: PoseStamped):
        position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        rotation = _quat_to_matrix(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        if msg.header.frame_id and msg.header.frame_id != self._world_frame:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._world_frame, msg.header.frame_id, rclpy.time.Time()
                )
            except Exception:
                self.get_logger().warn(
                    f"ee_target frame {msg.header.frame_id!r} 无法变换到 {self._world_frame!r}；丢弃",
                    throttle_duration_sec=2.0,
                )
                return
            tf_rotation = tf.transform.rotation
            rot = _quat_to_matrix(tf_rotation.x, tf_rotation.y, tf_rotation.z, tf_rotation.w)
            translation = np.array(
                [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
            )
            position = rot @ position + translation
            rotation = rot @ rotation
        self._ee_target = (position, rotation)
        self._ee_stamp = time.monotonic()

    def _on_mode(self, msg: String):
        candidate = msg.data.strip().lower()
        if candidate not in _MODES:
            self.get_logger().warn(f"拒绝未知 mode {msg.data!r}；保持 {self._mode!r}")
            return
        if candidate != self._mode:
            self.get_logger().info(f"mode: {self._mode} -> {candidate}")
        self._mode = candidate

    def _on_joint_state(self, msg: JointState):
        mapping = dict(zip(msg.name, msg.position))
        if not all(name in mapping for name in self._arm_joint_names):
            return
        self._q_arm = np.array([mapping[name] for name in self._arm_joint_names], dtype=float)

    def _on_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        # [AI] 2026-10-15 车载首跑抓到: rclpy 7.1.12 (Jazzy) 的 service 分发无条件以
        # (request, response) 两参调用回调 (executors.py _take_service), 旧式单参签名
        # 直接 TypeError 打死节点——仓库其余服务回调均为两参风格, 此处对齐。
        if not self.active:
            # ADR-0020 §5：未激活的写者不接受"打开写通道"这种请求。
            response.success = False
            response.message = f"wbc 未激活（当前 {self.state_name}），忽略 enable 请求"
            return response
        if request.data and not self._enabled:
            self._enabled = True
            self._degraded = False
            self._solver = HqpSolver(self._kinematics, self._params)  # 失败计数归零
            self.get_logger().warn("WBC enabled：HQP 成为底盘/臂常规运动唯一写入者")
        elif not request.data and self._enabled:
            self._enabled = False
            self._publish_base(np.zeros(3))  # 一次零速：显式停车请求
            self.get_logger().warn("WBC disabled：已发一次零速，之后静默")
        response.success = True
        response.message = f"enabled={self._enabled} mode={self._mode} degraded={self._degraded}"
        return response

    # ---- 发布 --------------------------------------------------------------

    def _publish_base(self, v_base):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear.x = float(v_base[0])
        msg.twist.linear.y = float(v_base[1])
        msg.twist.angular.z = float(v_base[2])
        self._base_pub.publish(msg)

    def _publish_arm(self, positions):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in positions]
        self._arm_pub.publish(msg)

    def _publish_status(self, **fields):
        payload = {
            "enabled": self._enabled,
            "mode": self._mode,
            "degraded": self._degraded,
            "publish_arm": self._arm_stream_enabled,
        }
        payload.update(fields)
        self._status_pub.publish(String(data=json.dumps(payload)))

    # ---- 主循环 ------------------------------------------------------------

    def _tick(self):
        self._tick_count += 1
        if not self._enabled or self._mode == "idle":
            return
        if self._degraded:
            return

        now = time.monotonic()
        q_base = self._lookup_base_pose()
        if q_base is None:
            # TF 丢失：无法在目标帧上求误差 —— 发零速（安全默认），不猜
            self._publish_base(np.zeros(3))
            self._publish_status(solve_status="no_tf")
            return

        cmd_fresh = (now - self._cmd_stamp) <= self._cmd_timeout
        ee_fresh = self._ee_target is not None and (now - self._ee_stamp) <= self._ee_timeout
        if not cmd_fresh:
            self._cmd = np.zeros(3)  # 超时即请求停车（spec §5）

        posture_target = self._q_arm if self._mode == "hold" else None
        result = self._solver.solve(
            WbcInput(
                q_base=q_base,
                q_arm=self._q_arm,
                v_cmd=self._cmd,
                mode=self._mode,
                ee_target=self._ee_target if (ee_fresh and self._mode == "servo") else None,
                posture_target=posture_target,
            )
        )
        self._last_result = result

        if result.status in ("solved", "solved inaccurate"):
            self._publish_base(result.v[0:3])
            if self._arm_stream_enabled:
                self._arm_cmd = np.clip(
                    self._arm_cmd + result.v[3:9] * (1.0 / self._rate_hz),
                    self._params.arm_q_min,
                    self._params.arm_q_max,
                )
                if self._tick_count % 2 == 0:  # 50 Hz：对齐舵机 PWM 帧率（spec §7）
                    self._publish_arm(self._arm_cmd)
        else:
            # 求解失败：先零速（停轮 + 臂保持）；连续失败到限 → degraded 停止发布
            self._publish_base(np.zeros(3))
            if self._solver.consecutive_failures >= self._fail_limit:
                self._degraded = True
                self.get_logger().error(
                    f"WBC degraded：连续 {self._fail_limit} 次求解失败，停止发布；"
                    "请检查后重新 enable"
                )

        if self._tick_count % 10 == 0:  # 10 Hz 状态
            self._publish_status(
                solve_status=result.status,
                solve_ms=round(result.solve_ms, 3),
                iters=result.iters,
                fallbacks=result.fallbacks,
                support_margin=round(result.support_margin, 4),
                ee_active=result.ee_active,
                cmd_fresh=cmd_fresh,
                ee_fresh=ee_fresh,
            )


def main(args=None):
    rclpy.init(args=args)
    node = WbcNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()