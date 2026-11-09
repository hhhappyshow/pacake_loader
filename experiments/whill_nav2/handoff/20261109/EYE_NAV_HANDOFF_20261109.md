# 眼动与轮椅导航接口交接

更新时间：2026-11-09；Jetson：`/home/whill/Shaki_ws/src`。目前需要区分**眼动连续速度**和**眼动选 Goal Pose**，两条路线不能视为已经互相接通。

## 眼动连续速度与纯 RDS

当前代码：`eye_gaze_ws/eye_web.py`、`screen_gaze.py`、`steering_model.py`、`motion_bridge.py`；原有标定和 `steering_profiles/` 需保留。当前控制页面由 `whill_rds_native_ws/src/rds_runtime/live_ui/rds_live_ui.py` 提供，Jetson 地址 `http://127.0.0.1:18786/`，眼动服务为本机 `127.0.0.1:18787`。

```text
双眼相机 -> 眼动推理与个人标定 -> 连续横向意图 -> 控制页面 HTTP 输入
 -> /shaki/manual_nav2/desired -> 八点 RDS
 -> /whill_rds_timing_test/cmd_vel_50hz_shadow
 -> cmd_vel_adapter -> /whill/controller/navigation
 -> 绝对位置标定 -> ESP32 蓝牙 HID -> iPad 摇杆 -> 轮椅
```

主题名称包含 `shadow`，但在所述硬件入口中它被订阅并接向实际执行器，不能从名称推断其不会驱动车辆。鼠标和眼动共享输入接口，先选一种输入，避免多个客户端争夺；眼动看前是前进意图，不等于鼠标松开。运行前应明确用户已标定、确认输入源、手动接管和环境。

2026-11-08 的纯 RDS 入口 `whill_rds_native_ws/src/rds_runtime/live_ui/run_rds_live.sh` 配置 `use_memory=false`：八点 RDS 使用双雷达实时障碍点，不使用 OctoMap 占据格；OctoMap 可以继续在画面显示，显示不等于参与 RDS。5 cm 净空在 RDS 侧，运行时可查询实际参数。当前实时过滤框为 `base_link` 下 x `[-0.15, 0.90]` m、y `[-0.275, 0.275]` m；框内的腿部回波和真实新障碍都会被滤掉。后方未观测区域不会由记忆图补充。这个过滤框并非实际八点车体，历史 UI 的框显示可能仍使用前界 0.80 m，需核对，不能仅凭显示判断实际过滤范围。

RDS 求解目标约 20 Hz，输出保持层目标 50 Hz；保持发送不等于插值，真实跟随和平滑要看现场。RDS 不是 Nav2，也不是 Nav2 Collision Monitor；不要误以为两者必然串联。纯 RDS 在 2026-11-08 曾实传感器静态运行，真实闭环表现、制动距离、摄像头掉线恢复和持续频率仍需按具体版本复核。无论 ARM 状态如何，都不要同时启动 Nav2 和这个真实控制链。

## 曾使用的独立启动入口

以下命令会连接真实传感器/ESP32；只在用户准备进行真机操作、无其他底盘进程占用时使用。`hardware` 会初始化绝对坐标接口，不自动 ARM。脚本不保证重启后自动启动：

```bash
ROOT=/home/whill/Shaki_ws/src/whill_rds_recovery_ws/src/whill_rds_recovery
SUPPORT="$ROOT/src/real_recovery/start_support.sh"
bash "$SUPPORT" sensors
bash "$SUPPORT" perception
bash "$SUPPORT" eye
bash "$SUPPORT" hardware

# 原版纯 RDS；不是 run_real.sh（后者是 RDS + Lattice 实验）
systemd-run --user --unit=shaki-rds-only --collect \
  /bin/bash /home/whill/Shaki_ws/src/whill_rds_native_ws/src/rds_runtime/live_ui/run_rds_live.sh

# 只读眼动监听，不发布速度或 ARM
bash "$SUPPORT" watch
```

这组命令基于现存脚本和 2026-11-08 静态运行记录，2026-11-09 未实机重跑。`start_support.sh` 的 `real` 是另一个 RDS + Lattice 入口；要纯 RDS 时不要用它。旧 `WheelChair_ws/scripts/manual_nav2.sh live` 是另一条 Collision Monitor 手动控制链，会占用同一设备，不能与此并行。

实际所见：原 `rds_live_ui.py` 页面不包含 `ui_audit.py` 包装提供的“重连眼镜”和点云诊断按钮；不要声称当前纯 RDS 页面已有这些按钮。`eye_audit.py` 带重连实现，但页面包装与纯 RDS 的整合需要单独完成和验证。眼动桥目前代码对有效样本和页面活跃有超时判断；`motion_bridge.py` 的 1 s 判定、RDS 的 cloud/intent 1 s 判定、odom 0.2 s 和执行器超时分别属于不同层，修改前必须看当前实际代码/参数，不能只看旧 README。

## 独立眼动记录与现场复核

`run_eye_signal_watch.sh` 查找已运行的 audited eye 进程后启动只读监听，记录样本有效性、推理和相机错误及切换时间；不会打开串口、发送速度或刷新页面心跳。它依赖 `run_eye_audited.sh` 启动的眼动日志，不录相机图像。用于手动控制时，请标记佩戴、光照变化和行程时间；无效样本并不能单独证明一定是光照导致。

2026-11-08 的一次监听：`whill_rds_recovery_ws/src/whill_rds_recovery/records/eye_signal_20261108_155612_27407/journal.jsonl`；同日纯 RDS 数据：`.../records/rds_only_capture_20261108_1547/`，已正常关闭并有 bag metadata。bag 含点云，实测约 740 MB/分钟；纯 RDS 原入口并不保证每次自动启用该录制。上一次异常断电的更早 bag 是否可完整回放须单独核实。

人体是否完全在过滤框内尚未验证：需要在相同静止车位采集空场和坐人两组**原始、未过滤**点云，并比较新增回波与框；现场墙和其他人会干扰，遮挡部分无法保证。不要仅凭过滤后的点云为空宣称人体全部被过滤。

## 眼动 Goal Pose 原型与 Nav2

独立代码：`eye_goal_ws/src/wheelchair_gaze_goal/`，入口 `eye_goal_ws/scripts/goal.sh`。它提供地图选点、调用 `/compute_path_to_pose` 和 `/navigate_to_pose` 的原型，使用屏幕九点标定输出候选；不是把当前一维速度眼动自动变成可靠 Goal Pose。

历史记录中，domain 207 的 mock 交互与真实 planner 的隔离求路径有通过记录；2026-11-05 没有真实发送导航目标，真实眼镜标定、当前主 UI 整合、地图/TF、实际动作以及轮椅闭环尚未完成。`goal.sh navigation` 会启用目标确认，绝不可仅因源码存在就视为已验收。Jetson 上的详细记录见 `/home/whill/Shaki_ws/src/eye_goal_ws/README.md` 和同目录的 `test_results.json`；该原型源码不在本次仓库更新范围内。

## 本次离线检查

2026-11-09 执行：

```bash
cd /home/whill/Shaki_ws/src/eye_gaze_ws
PYTHONDONTWRITEBYTECODE=1 /home/whill/eye_ctrl_wheelchair/projects/EYETLE/.venv/bin/python \
  -m unittest test_motion_bridge test_workflow
```

结果：7 项中 2 项失败，分别为 `test_continuous_owner_sequence_and_loss_stop` 和 `test_waits_for_asynchronous_arm_confirmation`。测试使用本机 mock HTTP 服务，未连接底盘。检查当前代码发现桥接器的 `page_active` 初始为 false，而这两项测试没有模拟 `set_page_active(True)`；因此在当前两个失败断言处发送的是 inactive/零比例输入，而不是测试所期待的 active 输入。应先把测试与页面心跳合同对齐，再检查测试中旧的丢失处理预期是否仍适用；本次未修改代码或测试，也不能据此宣称真机通过。其余各真实服务在检查时未运行。
