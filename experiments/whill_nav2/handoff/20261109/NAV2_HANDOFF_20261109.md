# WHILL Nav2 交接

更新时间：2026-11-09；目标机器 Jetson，ROS 2 Humble。本文描述磁盘配置和已验证边界，不代表服务当前运行，也不代表本次完成真机运动测试。

## 入口和工作区

主工作区：`/home/whill/Shaki_ws/src/WheelChair_ws`。

```bash
WS=/home/whill/Shaki_ws/src/WheelChair_ws

# 只做插件/环境检查，不连接底盘
bash "$WS/scripts/nav2.sh" turn_memory check

# 真机：先启动双雷达与 Odin，再启动 Nav2；需要相应硬件在线
bash "$WS/scripts/sensors.sh" start
bash "$WS/scripts/nav2.sh" turn_memory start

# 查看和停止；停止导航不会自动停止传感器
bash "$WS/scripts/nav2.sh" turn_memory status
bash "$WS/scripts/nav2.sh" turn_memory stop
bash "$WS/scripts/sensors.sh" status
bash "$WS/scripts/sensors.sh" stop
```

`sensors.sh` 启动传感器并执行就绪检查，不打开 RViz。`nav2.sh` 使用 `turn_memory` profile，会检查传感器和 `/dev/ble2wchair` 的占用，启动 Nav2、原底盘接口和 RViz；需要用户在设备端手动使能，再通过 RViz 的 2D Goal Pose 给目标。以上 `start` 是真实硬件入口，不是仿真，启动之前应确认底盘、现场和手动接管条件。不要和眼动/RDS 或其他 ESP32 控制进程同时运行。

实际入口为 `src/wheelchair_navigation/profiles/turn_memory/start_navigation.sh`，其运行环境还会 source `/home/whill/Shaki_ws/nav2_humble_update_20261031` 及相关 overlay。该目录是指向本工作区 `runtime/nav2_humble_update_20261031` 的符号链接；`first_run` 也是符号链接。传感器启动另外依赖 `/home/whill/eye_ctrl_wheelchair/sensor_visualization/`、`/home/whill/livox_ws/` 和 `/home/whill/odin_ws/`；不要仅复制本工作区就认定依赖完整。

## 配置与数据流

配置源：`src/wheelchair_navigation/profiles/turn_memory/navigation.yaml`，2026-11-09 实查 SHA-256 `89f607f43e419bde17a0aaeca00465befaa6c425bbdd510aca97857510ce4cc8`。运行时参数和插件以启动后 `ros2 param`、节点和日志为准。

```text
双 Mid-360 + Odin -> 点云/scan、odom 与 TF -> Nav2 costmap
2D Goal Pose -> Smac Lattice -> Rotation Shim + MPPI -> 速度平滑
            -> Collision Monitor -> /wheelchair/nav2/cmd_vel
            -> 原 ESP32/iPad 绝对位置接口 -> 底盘
```

- 规划：`nav2_smac_planner/SmacPlannerLattice`，`allow_reverse_expansion=true`、`allow_unknown=false`；运动原语文件以 YAML 的 `lattice_filepath` 为准。
- 跟踪：Rotation Shim 的主控制器是 MPPI，`DiffDrive`；控制器频率配置为 20 Hz。Spin、BackUp、AssistedTeleop 等行为插件有配置，不代表每条行为树都会调用。
- 目标检查：`StoppedGoalChecker`，位置容差 0.30 m、角度约 20 度；停止速度条件见 YAML。
- 全局/局部 costmap 的 `inflation_radius` 配置均为 0.5 m；足迹为八点轮廓，后轴 `base_link`。膨胀代价、足迹碰撞和 Collision Monitor 是不同机制，不应把膨胀区域一概当成实体墙。
- `expected_planner_frequency=10.0` 是 planner server 参数，不等于行为树已经每秒执行 10 次重规划；应测实际规划请求和耗时。
- 当前 Nav2 文件中的 MPPI 线速度约 `[-0.4037, 0.5366] m/s`、角速度上限约 `0.8109 rad/s`；速度平滑的线/角加速度配置为 `0.33 m/s²`、`0.8 rad/s²`，频率 20 Hz。它们与独立 RDS 版本不同，不能互相当作已同步。Nav2 `velocity_smoother` 的超时配置为 0.2 s，不是眼动样本超时。
- Collision Monitor 配置了 StopZone/VelocityPolygon 和 FootprintApproach，`source_timeout=0.75 s`；作用于本 Nav2 输出链，不等于眼动 RDS 自动使用它。

## 源码和验证状态

`src/navigation2` 当前 HEAD 是 `a097086719c88f781aa59788eca29ac6ca5e56db`，但工作树中 `nav2_smac_planner/src/smac_planner_lattice.cpp` 有未提交修改：对起点/终点落在地图外增加检查。不要称当前源码完全无本地改动，也不要覆盖或重置。实际加载的 overlay 二进制与该源码的对应关系尚未在本次验证。

2026-11-09 `bash "$WS/scripts/nav2.sh" turn_memory check` 成功；启动脚本语法及关键文件存在性检查通过。没有启动传感器、Nav2 或底盘，没有发送 Goal Pose；上一次真机表现不应被当成本次重启后的验证。

## 后续维护

- 要改 footprint、代价图、插件、速度和容差：先核对 `navigation.yaml` 和实际运行参数；不要覆盖其他人改动。
- 数据或日志：`WheelChair_ws/records/`；传感器、TF 与 DDS 启动文件在 `first_run` 指向的目录中。
- 旧 `docs/DEMO_HANDOFF.md` 含 2026-11-02 的历史链路和已过期入口，不能照搬。眼动接口请读 `../../eye_gaze_ws/EYE_NAV_HANDOFF_20261109.md`。
