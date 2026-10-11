#!/usr/bin/env bash
# voice_setup.sh —— 语音系统一键准备
# ============================================================
# 用途：每次重连 Jetson / 重插 USB 之后跑一次。
#
#   ① adb 权限      （装过 udev 规则 51-iflytek-adb.rules 就永久自动过）
#   ② adb 服务
#   ③ socket 隧道   （⑦ 方位数据的命脉，掉了就 Connection refused）
#   ④ 报告声卡      （编号会变，这里自动认名字）
#
# 用法：
#     bash ~/voice_setup.sh
# ============================================================
set -u

SERIAL=bc00143000108801ad0
VENDOR_PRODUCT=2207:0001
PORT=9100
SOCK=/tmp/vtn.sock

echo "=== 语音系统准备 ==="

# ---- ① adb 权限 ----------------------------------------------------
echo "[1/4] adb 权限 ..."
if adb devices 2>&1 | grep -q "no permissions"; then
    line=$(lsusb -d "$VENDOR_PRODUCT")
    if [ -z "$line" ]; then
        echo "    ✗ 没找到板子 ($VENDOR_PRODUCT) —— 检查 USB 是否插好"
        exit 1
    fi
    # lsusb 输出: Bus 001 Device 015: ID 2207:0001 ...
    #             $2=001        $4=015:
    node=$(printf '/dev/bus/usb/%03d/%03d' \
           "$(echo "$line" | awk '{print $2}')" \
           "$(echo "$line" | awk '{print $4}' | tr -d ':')")
    echo "    权限不足 → 临时修 $node（装过 udev 规则就不会走到这里）"
    sudo chmod 666 "$node"
else
    echo "    ✅ 权限正常（或板子还没出现，见第 ② 步）"
fi

# ---- ② adb 服务 ----------------------------------------------------
echo "[2/4] 重启 adb 服务 ..."
adb kill-server  >/dev/null 2>&1
adb start-server >/dev/null 2>&1
sleep 1
state=$(adb devices | grep "$SERIAL" | awk '{print $2}')
echo "    设备状态: ${state:-未发现}"
if [ "${state:-}" != "device" ]; then
    echo "    ✗ 板子没连上。检查：USB 插好？权限修了？"
    exit 1
fi

# ---- ③ socket 隧道（⑦ 方位）---------------------------------------
echo "[3/4] 建立 socket 隧道 ..."
if adb shell 'test -S /tmp/vtn.sock' 2>/dev/null; then
    adb forward "tcp:$PORT" "localfilesystem:$SOCK" >/dev/null
    echo "    ✅ 127.0.0.1:$PORT → $SOCK"
else
    echo "    ⚠ 板子里没有 $SOCK —— 板子没启动完，或 user_cfg.ini 的 port 被改回串口了"
fi

# ---- ④ 声卡（自动认名字，不记编号）---------------------------------
echo "[4/4] 声卡 ..."
# 讯飞板是车上唯一一块"能录音"的声卡 → arecord -l 里只有它一个
MIC_NAME=$(arecord -l 2>/dev/null | sed -n 's/^card [0-9]*: \([^ ]*\).*/\1/p' | head -1)
if [ -n "$MIC_NAME" ]; then
    echo "    讯飞麦(录音, 自认) = plughw:CARD=${MIC_NAME},DEV=0"
    echo "    → 录音: arecord -D plughw:CARD=${MIC_NAME},DEV=0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/m.wav"
else
    echo "    ⚠ 没找到能录音的声卡"
fi
PLAY_NAME=$(aplay -l 2>/dev/null | sed -n 's/^card [0-9]*: \([^ ]*\).*/\1/p' | head -1)
[ -n "$PLAY_NAME" ] && echo "    喇叭(放音, 自认) = plughw:CARD=${PLAY_NAME},DEV=0"

echo "    当前全部声卡（编号会变，仅供对照）:"
cat /proc/asound/cards 2>/dev/null | sed 's/^/      /'

echo
echo "完事。下一步:  python3 ~/vtn_angle.py   然后喊「小微小微」"
