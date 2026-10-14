#!/bin/bash
# [AI] 单用例运行 + 内核 UDP 计数器前后差 (定位丢包层)
# 用法: /tmp/case_udp.sh <name> <cyclonedds_uri|-> <rmw> <fps> [secs]
source /opt/ros/jazzy/setup.bash
source "$HOME/ipc_bench_ws/install/setup.bash"
BIN="$HOME/ipc_bench_ws/install/ipc_bench/lib/ipc_bench/ipc_bench"
name=$1; uri=$2; rmw=$3; fps=${4:-30}; secs=${5:-8}
export ROS_DOMAIN_ID=47 RMW_IMPLEMENTATION="$rmw"
[ "$uri" != "-" ] && export CYCLONEDDS_URI="$uri" || unset CYCLONEDDS_URI

snap() { awk '/^Udp:/{c++; if(c==2){print "InDatagrams="$2" InErrors="$4" OutDatagrams="$5" RcvbufErrors="$6" SndbufErrors="$7; exit}}' /proc/net/snmp; }
echo "[$name] udp before: $(snap)"
"$BIN" --mode pub --topic "/ipcbench/$name" --w 1280 --h 800 --bpp 2 --fps "$fps" --secs "$secs" --qos 5 --be 1 > /tmp/c_${name}_p.log 2>&1 &
p=$!; sleep 1.5
"$BIN" --mode sub --topic "/ipcbench/$name" --secs "$secs" --qos 5 --be 1 > /tmp/c_${name}_s.log 2>&1 &
s=$!; wait $s; sleep 0.3; kill $p 2>/dev/null; wait $p 2>/dev/null
echo "[$name] udp after : $(snap)"
grep '^\[' /tmp/c_${name}_p.log | sed 's/^/  pub /'
grep '^\[' /tmp/c_${name}_s.log | sed 's/^/  sub /'