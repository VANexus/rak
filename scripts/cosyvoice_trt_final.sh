#!/bin/bash
# CosyVoice TRT 最后一搏：清场 -> 建引擎 -> 汇报。
# 用法: sudo 不需要; 直接 bash scripts/cosyvoice_trt_final.sh
# 会临时停掉 docker/x11vnc/gnome-remote-desktop，并可选杀 opencode。
set -u
PW=${SUDO_PW:-kali}
S="echo $PW | sudo -S"

echo "=== 1/5 清场 ==="
eval $S systemctl stop docker docker.socket containerd 2>/dev/null
eval $S systemctl stop x11vnc.service gnome-remote-desktop 2>/dev/null
eval $S pkill -f gnome-remote-desktop-daemon 2>/dev/null
pkill -f x11vnc 2>/dev/null
# opencode 占 ~200MB;想留着就注释掉这行
pkill -f opencode 2>/dev/null
sync; eval $S sh -c 'echo 3 > /proc/sys/vm/drop_caches'

echo "=== 2/5 zram + swap ==="
swapon --show | grep -q zram0 || eval $S swapon -p 20 /dev/zram0
swapon --show
free -m | head -2

echo "=== 3/5 fp16 ONNX (已缓存则跳过) ==="
python3 scripts/fp16_onnx_cosyvoice.py || { echo "onnx convert failed"; exit 1; }

echo "=== 4/5 构建 TRT 引擎 (10-40 分钟,别动这台机器) ==="
python3 scripts/build_cosyvoice_trt.py 2>&1 | tee /tmp/cosy_trt_final.log | grep -E "parsing|building|plan written|build failed|Succesfully"
RC=${PIPESTATUS[0]}

echo "=== 5/5 结果 ==="
MODEL=/home/xrak/rak/acoustic/models/Fun-CosyVoice3-0.5B-2512
if [ $RC -eq 0 ] && ls $MODEL/*.plan >/dev/null 2>&1; then
  echo "成功: $MODEL/flow.decoder.estimator.fp16.mygpu.plan"
  echo "无需任何配置, cosyvoice_backend 下次加载自动启用 TRT"
else
  echo "失败 (exit=$RC); 详见 /tmp/cosy_trt_final.log"
  grep -E "error 2 for|region|Requested amount" /tmp/cosy_trt_final.log | tail -3
fi

echo "=== 恢复 docker (x11vnc/gnome-remote 保持关闭) ==="
eval $S systemctl start docker docker.socket containerd 2>/dev/null
exit $RC
