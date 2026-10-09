#!/usr/bin/env bash
# [AI] rak_wbc 影子冒烟（① 路线）：不起任何真实执行器，只验证节点在 ROS 图上的契约：
#   静态 TF(map→base_link) + fake 臂位置流(夹具) + rak_wbc [影子 remap] +
#   外部 /cmd_vel 与 /wbc/ee_target → 检查 /wbc/status、影子底盘输出、臂 setpoints。
#
# 用法（Jetson，已 colcon build 且 source install）：
#   scripts/wbc_shadow_smoke.sh
# 用法（未安装的机器，如 x86 彩排；需 PYTHONPATH=src/rak_wbc 与 pinocchio/osqp）：
#   scripts/wbc_shadow_smoke.sh "/tmp/wbcenv/bin/python -m rak_wbc.node" \
#                              "/tmp/wbcenv/bin/python -m rak_wbc.fake_arm_stream"
#
# 安全：底盘输出被 remap 到 /wbc/shadow_reference（绝不进 mecanum reference）；
#       臂 setpoints 只到 fake 夹具。真机上跑本脚本不会让任何执行器动。
set -euo pipefail

WBC_RUNNER=${1:-"ros2 run rak_wbc wbc_node"}
FAKE_RUNNER=${2:-"ros2 run rak_wbc fake_arm_stream"}
LOG_DIR=$(mktemp -d /tmp/wbc_smoke.XXXXXX)

cleanup() {
  # 排除自身 PID：本地彩排时 runer 字符串也出现在父 shell 的 cmdline 里，
  # 直接 pkill -f 会自杀（脱离时表现为退出码 -1）。
  local pattern="(rak_wbc\.node|rak_wbc\.fake_arm_stream|bin/wbc_node|bin/fake_arm_stream|lib/rak_wbc/wbc_node|lib/rak_wbc/fake_arm_stream|static_transform_publisher)"
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    if [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ]; then
      kill "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

echo "[smoke] 启动静态 TF map->base_link"
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link \
  >"$LOG_DIR/tf.log" 2>&1 &

echo "[smoke] 启动 fake 臂位置流（夹具，命令回显）"
$FAKE_RUNNER >"$LOG_DIR/fake.log" 2>&1 &

sleep 1
echo "[smoke] 启动 rak_wbc（enabled=true / mode=servo / 影子底盘话题 / 臂流开）"
$WBC_RUNNER --ros-args \
  -p enabled:=true -p mode:=servo \
  -p base_reference_topic:=/wbc/shadow_reference \
  -p publish_arm:=true \
  ${EXTRA_ARGS:-} \
  >"$LOG_DIR/wbc.log" 2>&1 &
sleep 4

echo "[smoke] 注入 /cmd_vel 与 /wbc/ee_target（后台持续，覆盖检查窗口）"
timeout 30 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.1}}" >/dev/null 2>&1 &
timeout 30 ros2 topic pub -r 2 /wbc/ee_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.30, y: 0.05, z: 0.60}, orientation: {w: 1.0}}}" \
  >/dev/null 2>&1 &
sleep 6  # 冷启动余量：pinocchio import + 模型构建在 Jetson 上首跑可达数秒（2026-10-08 实测首跑抖动）

fail=0
echo "== 检查 1/3：/wbc/status =="
timeout 6 ros2 topic echo --once /wbc/status || fail=1
echo "== 检查 2/3：/wbc/shadow_reference（影子底盘输出）=="
timeout 6 ros2 topic echo --once /wbc/shadow_reference || fail=1
echo "== 检查 3/3：/wbc/arm_setpoints（臂位置流）=="
timeout 6 ros2 topic echo --once /wbc/arm_setpoints || fail=1

echo "== fake 夹具回显（/joint_states 抽样）=="
timeout 6 ros2 topic echo --once /joint_states || fail=1

echo "== 节点日志尾部 =="
tail -n 6 "$LOG_DIR/wbc.log" || true
echo "== 夹具日志尾部 =="
tail -n 3 "$LOG_DIR/fake.log" || true

if [ "$fail" -ne 0 ]; then
  echo "[smoke] 失败：有话题检查未通过（日志在 $LOG_DIR）"
  exit 1
fi
echo "[smoke] 通过：契约链（cmd_vel→HQP→影子输出 / 臂流→夹具回显）成立（日志在 $LOG_DIR）"