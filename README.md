# pacake_loader

S20 机械臂装箱的两套代码放在同一个仓库里。

| 目录 | 内容 |
| --- | --- |
| [shiki/](shiki/README.md) | 当前装箱主线：感知、放置规划、Newton 仿真、浏览器查看器和真机 deployment |
| [RoboticArm/](RoboticArm/README.md) | ROS Noetic + Gazebo + MoveIt 的仿真与驱动工程，来源是 [adamliaoyifan/RoboticArm](https://github.com/adamliaoyifan/RoboticArm) 的 `f9ade532` |

`shiki` 用数值逆解和 CAD 碰撞做自动装箱，可以用 `./s20-view` 在浏览器里看过程。`RoboticArm` 提供可交互的 Gazebo 场景：机械臂、集装箱和行李箱在同一个世界里，并用 MoveIt 规划运动。两边的坐标、装箱算法和执行接口并不通用。

## 浏览器里看当前仿真

在 `shiki/` 下执行：

```bash
./s20-view
```

浏览器打开 http://127.0.0.1:8765 。需要本机已经备好规划用的 Python 环境、Newton 运行时和 YOLO 权重；这些大文件不在 git 里。细节见 [shiki/experiments/s20_online_mixed8_20261024/browser_viewer/README.md](shiki/experiments/s20_online_mixed8_20261024/browser_viewer/README.md)。

## Gazebo 仿真

`RoboticArm` 面向 Ubuntu 20.04 的 ROS Noetic。在 22.04 上用它自带的 Docker：

```bash
cd RoboticArm
./docker/noetic/run.sh build
./docker/noetic/run.sh sim
```

容器起来后，在对应终端里启动 Gazebo 和 MoveIt。脚本说明见 `RoboticArm/docker/noetic/run.sh`。
