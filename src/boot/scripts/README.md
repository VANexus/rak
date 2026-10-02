# boot/scripts/ — operator launchers for xrak

These are the canonical wrapper scripts for running the xrak robot's ROS 2
chains on the Jetson. Each script:

1. Sets the X11 environment so the GUI lands on the Jetson local
   touchscreen (`DISPLAY=:1`, `XAUTHORITY=/run/user/1000/gdm/Xauthority`),
   not on a forwarded XQuartz display.
2. Sets the team-mandated DDS configuration: Cyclone DDS, ROS domain 45,
   config from `install/boot/share/boot/config/cyclonedds.xml`.
3. Sources the standard ROS env twice (base + workspace).
4. Patches `AMENT_PREFIX_PATH` so the `motor_stub` Python package (which
   colcon's setup.bash skips) is visible to `ros2 launch`.
5. Launches the chain via `setsid + nohup + </dev/null` so the SSH
   session that started it can exit without killing the chain.

## Scripts

| Script | What it starts | Use when |
|---|---|---|
| `run_lidar.sh` | `lslidar_driver_node` + `scan_to_scan_filter_chain` | You only need `/scan` (no SLAM, no nav2). Minimal CPU footprint. |
| `run_slam.sh` | lidar + static_tf + rf2o + slam_toolbox | You want the SLAM chain running with /map streaming, but no nav2. |
| `run_nav2.sh` | full chain: slam + cmd_vel_stub + nav2 navigation stack | You want Nav2 planning + control. Requires the team leader to be OK with motor_stub forwarding `/cmd_vel`. |
| `run_rviz_slam.sh` | RViz2 with the team's slam-aware config | Always run alongside one of the launchers above. Targets the Jetson local touchscreen, not XQuartz. |

## Install on the Jetson

These scripts assume hardcoded paths under `/home/xrak/rak/...`. To deploy
on a fresh Jetson, copy them into `/tmp/`:

```bash
scp src/boot/scripts/run_*.sh xrak:/tmp/
ssh xrak 'chmod +x /tmp/run_*.sh'
```

Then run any one of them, e.g.:

```bash
ssh xrak
/tmp/run_lidar.sh &
# /tmp/lidar_only_run.log captures the chain log
# /tmp/run_nav2.sh writes to /tmp/nav2_run.log
# /tmp/run_rviz_slam.sh writes to /tmp/rviz_slam_run.log
```

## Stopping the chain

The scripts use `setsid`, so killing the SSH session that started them
does NOT stop them. To stop:

```bash
ssh xrak
# stop everything ROS on the box:
pkill -9 -f 'ros2 launch' 2>/dev/null
pkill -9 -f lslidar_driver_node 2>/dev/null
pkill -9 -f scan_to_scan_filter_chain 2>/dev/null
pkill -9 -f rf2o_laser_odometry_node 2>/dev/null
pkill -9 -f slam_toolbox 2>/dev/null
pkill -9 -f 'nav2_' 2>/dev/null
pkill -9 -f controller_server 2>/dev/null
pkill -9 -f planner_server 2>/dev/null
pkill -9 -f bt_navigator 2>/dev/null
pkill -9 -f velocity_smoother 2>/dev/null
pkill -9 -f behavior_server 2>/dev/null
pkill -9 -f collision_monitor 2>/dev/null
pkill -9 -f smoother_server 2>/dev/null
pkill -9 -f cmd_vel_stub 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
```

## Environment knobs (override before invoking)

```bash
# Use a different RMW (the team has standardised on Cyclone, but you can override):
RMW_IMPLEMENTATION=rmw_fastrtps_cpp /tmp/run_nav2.sh

# Run on a different ROS domain (default 45):
ROS_DOMAIN_ID=0 /tmp/run_nav2.sh
```

These are runtime overrides -- they don't persist between reboots because
`/tmp/` clears on reboot. To make them permanent, edit the script in
`src/boot/scripts/` and re-push.
