# 14 箱 deploy：共享布局、完整预检与 ServoJ 回放

默认任务为 12 个静态底箱 + 2 个依次观测的待放箱。`current_profile.py`
从 `straight_place/stacked_scene.py:twelve_box_fixture()` 生成底箱，不再保存另一份旧坐标。
中列顶箱 Base_12 的横向起点为 0.565 m。实物必须对应这份布局。

## 入口

在 shiki 目录运行。以下命令只用已有观测离线规划，不连接设备：

```bash
./s20-run config --task-mode stacked14
./s20-run plan --task-mode stacked14 --stack14-region right-middle \
  --observation /绝对路径/观测目录
```

观测目录包含 `observation/observation.json`、`grasp.json`、`controller_snapshot.json`。
`--stack14-region` 支持 `right-middle`、`left-middle`、`both`；both 是允许两侧，
不保证每侧一个。普通四箱通过 `--task-mode upright` 使用保留的 upright_profile.json。
14 箱默认两轮，普通四箱默认四轮；可用 `--rounds 1` 限定一个新箱。

## 实机部署路径

online_session.py 的默认 SDK 设备目录仍是历史目录
`/home/hku_reconova/s20_contact_mixed_20261025_r2`，它与 Git 仓库目录不同。
必须把更新后的代码同步到实际设备目录，或用 `--device-root` 指向设备上具有
完整 s20-device、s20-interfaces 和依赖的 shiki 目录。
14 箱执行前会比较该目录 native_cycle_bridge.py 与本地版本的 SHA256，不一致则
在启动运动前终止。同步规划文件并不等于同步设备端桥接文件。

代码同步、离线轨迹核对及实物布局核对后，实机命令形式为：

```bash
./s20-run run --task-mode stacked14 --stack14-region right-middle --rounds 1 \
  --device-root /设备上实际的shiki绝对目录 \
  --native-backend servoj_replay --native-speed-scale 1.0 \
  --place-speed-scale 1.0 --return-speed-scale 1.0 --execute
```

开头必须显示 `Planning mode: stacked14; handoff: post_pick_lift` 和选定区域。
本次开发验证未执行任何实机运动。

## 实际流程与证据

1. 按实际观测选择抓取候选。
2. prepare_stack14_cycle.py 对候选运行真实 deploy 抓取规划，再运行放置及返回规划。
   第一个完整可行方案才会被接受；没有完整方案则不执行抓取。
3. 抓取末端停在 post_pick_lift，不执行旧 TAUGHT_ENTRY；放置从完全相同的关节状态
   和抓取段计算的箱体吸附关系起算。
4. 舱外协调翻转 → 高位进舱 → 必要时横移 → 到位下降 → 释放 → 空手返回。
5. compose_cycle.py 在抓取前验证两段衔接，生成执行配方。
6. ServoJ 使用配方内已重采样的 positions_deg，保留实际时间缩放；不把阶段再转换为
   控制器 MoveL。其他旧后端保留原逻辑；14 箱入口要求 servoj_replay。

每轮输出：
- preflight.json：生效布局、区域、各抓取候选的抓取/放置结果。
- grasp_*/pick/result.json：抓取阶段与实际计划终点；不应出现 TAUGHT_ENTRY。
- grasp_*/place/result.json：路径、候选、失败信息、有效选项。
- cycle_recipe/*.json：真正发送的轨迹点及吸附/释放事件。
- next_scene_preview.json：仅预测放置状态；实机完成后才提交占用。

## 尺寸与验证边界

14 箱使用视觉拟合实体尺寸和位姿，不将 2 cm 平面膨胀后的尺寸当成翻转后实体尺寸。
本模式 padding_xy_per_side_m=0，与仿真的实体 CAD 检查一致；这是取消旧的统一尺寸膨胀，
不是新增测量误差裕量。仍检查真实尺寸模型、底箱碰撞、进舱净空、顶盖和关节约束。
若现场视觉尺寸或底箱位置误差较大，需要先校准或另行建模误差包络，不能把离线通过
理解为已经验证实际接触与稳定性。普通四箱保留原配置中的尺寸膨胀。

关节策略为 15° 限位余量、25°/s、50°/s²；14 箱入口拒绝大于 1 的执行速度倍率。
候选选择当前是感知偏好顺序中的第一个完整可行方案，不是所有候选的全局最优解。
左中倾斜支撑是几何模型，未验证真实纸箱变形、吸盘滑移、落稳及承载能力。
