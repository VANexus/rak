# 操作入口

这些脚本共享 `environment.sh`，从自身位置向上寻找仓库，或使用 `RAK_WORKSPACE`。
默认加载 ROS Jazzy 和 `install/architecture/local_setup.bash`。
不再依赖固定用户名、当前工作目录或手工添加 Python 包搜索路径。

| 脚本 | 启动内容 | 日志 |
|---|---|---|
| `run_lidar.sh` | 雷达与滤波 | `/tmp/lidar_only_run.log` |
| `run_slam.sh` | 雷达、rf2o、SLAM | `/tmp/slam_run.log` |
| `run_nav2.sh` | SLAM、Nav2、测试转发 | `/tmp/nav2_run.log` |
| `run_rviz_slam.sh` | 导航 RViz | `/tmp/rviz_slam_run.log` |
| `run_demo.sh` | 旧机械臂 MoveIt 演示 | `/tmp/demo_run.log` |
| `validate_moveit_installed.sh` | 只检查资源，不启动节点 | 标准输出 |

在仓库中直接调用，launch 参数透传：

```bash
bash src/rak_bringup/scripts/run_lidar.sh
bash src/rak_bringup/scripts/run_slam.sh odom_source:=none
ROS_DOMAIN_ID=47 bash src/rak_bringup/scripts/run_nav2.sh
```

雷达、SLAM、Nav2 不要求桌面。RViz 和 MoveIt demo 默认使用本机 `DISPLAY=:1`，
其 Xauthority 路径根据当前 UID 生成，并检查文件是否可读。
可覆盖 `DISPLAY`、`XAUTHORITY`、`QT_QPA_PLATFORM`，但桌面仍有 CMA 内存成本。

其他可覆盖参数：`RAK_WORKSPACE`、`RAK_INSTALL_PREFIX`、
`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、`CYCLONEDDS_URI`。
默认 DDS 为 Cyclone，运行 domain 为 47。集成测试使用独立 domain 77/187，
避免连接到运行中的机器人；不得让同一条运行链路意外使用不同 domain。

脚本沿用 `setsid nohup`，退出 SSH 不会自动停止节点。停止时针对对应 launch
进程组发送 SIGINT 并确认子节点退出，不要使用全机范围的 `pkill -9` 作为日常关闭方式。
需要终端可控退出时，直接执行 README 中的 `ros2 launch ...` 并使用 Ctrl-C。

不要同时启动 lidar、SLAM、Nav2 三条链路：上层已经包含下层，会重复打开设备。
不要仅把单个脚本复制到 `/tmp`，它依赖同目录的 `environment.sh`。
脚本启动真实传感器；Nav2 的输出仍接测试转发，不代表真实运动链路已具备安全保障。
