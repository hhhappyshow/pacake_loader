# Shaki 轮椅工作空间索引

更新时间：2026-11-09，Asia/Hong_Kong。工作空间根目录为 `/home/whill/Shaki_ws/src`。
这是一份实查索引，不代替各模块的详细文档。重新上电后应重新检查服务和参数，历史记录不代表当前正在运行。

## 首先阅读

- [Nav2 交接](NAV2_HANDOFF_20261109.md)：现有导航入口、配置、传感器和执行链。
- [眼动与导航接口交接](EYE_NAV_HANDOFF_20261109.md)：眼动速度、RDS、独立监听以及眼动 Goal Pose 的边界。
- 历史共享交接位于 Jetson 的 `/home/whill/Shaki_ws/src/HANDOFF.md`；其中部分运行状态和启动说明可能过期。

## 目录分工

| 路径（均相对本目录） | 内容和地位 |
| --- | --- |
| `WheelChair_ws/` | ROS 2 Humble 轮椅 Nav2 主工作区；ROS 源码在它的 `src/` 下，`scripts/` 为入口，`docs/` 为说明。 |
| `eye_gaze_ws/` | 当前眼镜标定、双眼推理、眼动到手动速度输入的 Python 代码；不是已整理成 colcon 包的工作区。 |
| `eye_goal_ws/` | 独立眼动选点/Goal Pose 原型；曾有隔离仿真和接口测试，未完成当前界面及真机闭环验收。 |
| `eye_calibration_ws/` | 历史标定相关资料；不要因重复名称删除个人标定。 |
| `whill_rds_native_ws/` | 八点 RDS、纯 RDS 控制页面及真实输入适配。 |
| `whill_rds_recovery_ws/` | 单独的 RDS + Lattice 恢复研究、眼动监听、事件记录和相关仿真；不等同纯 RDS。 |
| `whill_mapping_ws/` | 独立二维建图与定位准备。 |
| `bac_sim_ws/`, `velocity_avoidance_ws/`, `navdp_ws/`, `opennavmap_ws/`, `glim_ws/`, `elevator_lio_ws/`, `mouse_nav_sandbox/`, `whill_rds_stage1_ws/` | 原型、对比或历史工作；不要把目录存在误认为真机已部署。 |

## 工作空间边界

- 外层 `Shaki_ws` 是项目容器；`src/` 中有多个独立工作区。不要在外层直接对所有嵌套目录执行一次 `colcon build`。
- 传感器启动仍引用 `/home/whill/eye_ctrl_wheelchair/`、`/home/whill/livox_ws/`、`/home/whill/odin_ws/`；执行器接口仍在 `/home/whill/Shaki_ws/esp32_velocity_interface/`。这些不是已经迁进 `src/` 的内容。
- 导航与手动/眼动控制可能竞争 `/dev/ble2wchair` 和轮椅命令；切换前确认只有一个执行器所有者。不得同时启动两个真实控制链。
- 本次整理只新增文档和索引，没有搬移源码、删除旧工作区、改导航参数或重启硬件。

## 本次检查

2026-11-09 约 10:57 Jetson 在线；Nav2、眼动、纯 RDS 服务在该时刻均未运行。`nav2.sh turn_memory check` 通过，相关入口 `bash -n` 通过；这不是传感器在线或真机运动验收。
`eye_gaze_ws` 的 `test_motion_bridge test_workflow` 实测 7 项中 2 项失败，详见眼动交接，尚需排查。
