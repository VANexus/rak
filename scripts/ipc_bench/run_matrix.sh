#!/bin/bash
# [AI] IPC 基准矩阵 v2 (L1 基线): rmw × QoS × 进程内/跨进程
# (不用 set -u: ROS setup.bash 不兼容 nounset, 团队 environment.sh 已注明)
source /opt/ros/jazzy/setup.bash
source "$HOME/ipc_bench_ws/install/setup.bash"
BIN="$HOME/ipc_bench_ws/install/ipc_bench/lib/ipc_bench/ipc_bench"
TEAMCFG="file:///home/xrak/rak/src/rak_bringup/config/cyclonedds.xml"
SECS=${SECS:-8}
export ROS_DOMAIN_ID=47

run_cross() { # name rmw uri w h bpp qos_depth be
  local name=$1 rmw=$2 uri=$3 w=$4 h=$5 bpp=$6 qd=$7 be=$8
  export RMW_IMPLEMENTATION=$rmw
  if [ -n "$uri" ]; then export CYCLONEDDS_URI=$uri; else unset CYCLONEDDS_URI; fi
  local topic=/ipcbench/$name
  "$BIN" --mode pub --topic "$topic" --w "$w" --h "$h" --bpp "$bpp" --fps 30 --secs "$SECS" \
      --qos "$qd" --be "$be" > /tmp/b_${name}_pub.log 2>&1 &
  local pp=$!
  sleep 1.5
  "$BIN" --mode sub --topic "$topic" --secs "$SECS" --qos "$qd" --be "$be" > /tmp/b_${name}_sub.log 2>&1 &
  local sp=$!
  wait $sp
  sleep 0.5; kill $pp 2>/dev/null; wait $pp 2>/dev/null
  echo "### $name   (rmw=${rmw#rmw_} cfg=${uri##*/} qos=$([ "$be" = 1 ] && echo BE || echo REL)$qd)"
  grep '^\[' /tmp/b_${name}_pub.log 2>/dev/null | sed 's/^/  pub /'
  grep '^\[' /tmp/b_${name}_sub.log 2>/dev/null | sed 's/^/  sub /'
}

run_intra() { # name intra w h bpp
  local name=$1 intra=$2 w=$3 h=$4 bpp=$5
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  "$BIN" --mode both --intra "$intra" --topic /ipcbench/$name --w "$w" --h "$h" --bpp "$bpp" \
      --fps 30 --secs "$SECS" > /tmp/b_${name}.log 2>&1
  echo "### $name   (单进程 intra=$intra)"
  grep '^\[' /tmp/b_${name}.log | sed 's/^/  /'
}

echo "########## A) 深度规格 1280x800x2 @30Hz (~61MB/s) ##########"
run_cross cx_cur_rel   rmw_cyclonedds_cpp "$TEAMCFG" 1280 800 2 10 0
run_cross cx_cur_be    rmw_cyclonedds_cpp "$TEAMCFG" 1280 800 2 5  1
run_cross cx_cur_be1   rmw_cyclonedds_cpp "$TEAMCFG" 1280 800 2 1  1
run_cross cx_lo_be     rmw_cyclonedds_cpp file:///tmp/cx_lo.xml      1280 800 2 5 1
run_cross cx_lonomc_be rmw_cyclonedds_cpp file:///tmp/cx_lo_nomc.xml 1280 800 2 5 1
run_cross fd_rel       rmw_fastrtps_cpp   "" 1280 800 2 10 0
run_cross fd_be        rmw_fastrtps_cpp   "" 1280 800 2 5  1
run_intra intra_off 0 1280 800 2
run_intra intra_on  1 1280 800 2
echo "########## B) 彩色规格 1280x800x3 @30Hz (~92MB/s) ##########"
run_cross cx_cur_be    rmw_cyclonedds_cpp "$TEAMCFG" 1280 800 3 5 1
run_cross fd_be        rmw_fastrtps_cpp   ""        1280 800 3 5 1
run_intra intra_on  1 1280 800 3
echo "########## 完成 ##########"
free -m | head -2