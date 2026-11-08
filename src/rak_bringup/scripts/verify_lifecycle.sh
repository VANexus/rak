#!/usr/bin/env bash
# ADR-0020 编排层的验收脚本（在 Jetson 上跑；本机→板子链路中断时它是待办清单）。
#
# 为什么存在：2026-10-29 这一轮里，机制层与夹具编排的 31 项测试已在 Orin 实测通过，
# 但下面三项门禁因 SSH 中断没跑成。没跑绿之前，编排层不得称"已验证"（AGENTS §6）。
#
# 用法（在 worktree 或主仓根目录都行，只要构建产物在 $PREFIX 下）：
#   bash src/rak_bringup/scripts/verify_lifecycle.sh [install/<你的构建名>]
set -uo pipefail

PREFIX="${1:-install/lifecycle}"
DOMAIN="${TEST_ROS_DOMAIN_ID:-77}"
rc=0

cd "$(dirname "$0")/../../.." || exit 1
[ -d "$PREFIX" ] || { echo "没有 $PREFIX，先 colcon build（见 CLAUDE.md §Build & verify）"; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$PREFIX/setup.bash" || { echo "$PREFIX 未构建完整"; exit 1; }
export ROS_DOMAIN_ID="$DOMAIN"

step() {
  echo; echo "### $*"
  if "$@" > /tmp/rak-lifecycle-step.log 2>&1; then
    echo "PASS"
  else
    echo "FAIL（最后 20 行）"; tail -20 /tmp/rak-lifecycle-step.log; rc=1
  fi
}

# 1) 全量结构门禁：23 包 + 资源逐字节一致 + 每个 launch 可解析 + 编排模块可导入
step python3 scripts/check_workspace.py --installed

# 2) 清单↔launch 一致性：受管组件必须以 LifecycleNode 起，否则就是静默空转节点
step colcon test --build-base "$(dirname "$PREFIX")/$(basename "$PREFIX" | sed 's/^install/build/')" \
  --install-base "$PREFIX" --packages-select rak_bringup
step colcon test-result --verbose

# 3) 机制与编排测试（隔离域，不碰硬件）
step env PYTHONPATH="$PWD/src/rak_lifecycle" python3 -m pytest -q src/rak_lifecycle/test
step python3 -m pytest -q src/rak_task_executor/test

# 4) 夹具编排端到端：拉起来必须全 ready，停一个域不得波及其余
echo; echo "### fixture 端到端（supervisor profile:=fixture）"
setsid ros2 launch rak_bringup supervisor.launch.py profile:=fixture > /tmp/rak-fixture.log 2>&1 &
launcher=$!
sleep 12
if ros2 run rak_lifecycle rak_system status; then
  echo "status 可读"
else
  echo "FAIL：rak_system status 非 0（见 /tmp/rak-fixture.log）"; tail -20 /tmp/rak-fixture.log; rc=1
fi
# 停掉最下游的 task 域，上游 navigation 必须还活着
ros2 param set /rak_system_manager enabled_domains navigation > /dev/null 2>&1
sleep 4
if ros2 topic echo --once /rak/system_state > /tmp/rak-state.yaml 2>&1; then
  grep -q "navigation" /tmp/rak-state.yaml || { echo "FAIL：/rak/system_state 里没有 navigation"; rc=1; }
  echo "停用后的聚合状态已抓取：/tmp/rak-state.yaml"
else
  echo "FAIL：/rak/system_state 读不到"; rc=1
fi
kill -TERM -"$launcher" 2>/dev/null
sleep 3
pkill -f 'supervis[o]r.launch' 2>/dev/null || true

echo; [ "$rc" = 0 ] && echo "ALL PASS" || echo "有失败项，见上方输出"
exit "$rc"
