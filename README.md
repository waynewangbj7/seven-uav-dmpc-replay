# 七机 DMPC · ROS 2 / RViz 城市回放归档

归档日期为 2026-11-21。默认场景为 Case 1、seed 7。本说明是本阶段的启动入口和交接记录。

## 归档内容

- `replay/` 包含 Python 发布节点、播放控制和浏览器面板、RViz 预设，以及 Mavic 风格机身、旋转桨叶和可选固定翼机模。
- `case-data/` 包含 seed 7–11 的原始状态、误差与间距、事件、求解摘要、对应连接历史及源文件指纹。
- `VERIFICATION.json` 记录从 ZIP 解压后运行的五组数据检查。每个归档文件的 SHA-256 见 `MANIFEST.json`。
- 上游算法、原论文、Python 求解及 MATLAB 验证继续见 `uav_dmpc_learning_baseline_2026-11-09.zip`。本包用于回放该基线的数据。上游 ZIP 的校验摘要收在 manifest 中。

## 启动

在 Ubuntu 终端进入解压后含有本 README 的文件夹。运行环境为此前已跑通的 WSL 2、Ubuntu 24.04 和 ROS 2 Jazzy Desktop。也可在装好 ROS 2 Jazzy 的原生 Ubuntu 上打开浏览器和 RViz。

```bash
source /opt/ros/jazzy/setup.bash
python3 replay/replay_case1.py --archive "$PWD/case-data" --rviz
```

脚本会用包内的城市配置打开 RViz。左侧应出现 `City` 和 `Seven UAVs`。在浏览器打开终端显示的地址，默认是 `http://localhost:8765/`。面板支持暂停、前后单步、拖动进度条和事件跳转。可点 **6.6 s · S6**，再点「上一帧」对照合并前后。RViz 的 `Views` 可选 `Follow formation`、`City flyby` 和 `Full route`。

- 加 `--seed 8` 切换结果，加 `--speed 0.5` 慢放，加 `--panel-port 8766` 更换占用的端口。
- 加 `--mesh replay/fixed_wing.obj` 可换回固定翼。Mavic 机身与桨叶需保留在同一目录。
- 单独检查数据时可运行 `python3 replay/replay_case1.py --archive "$PWD/case-data" --check-data`。这一步只需 Python。

## 核心原理

上游 Python 工程版按每 **0.3 秒**保存七机的平面位置、航向和角色。回放节点用同一个时钟驱动画面与浏览器面板。画面在相邻采样帧之间插值，事件、最近机对和距离仍对应原始采样点。城市静态标记发布在 `/formation/city`，飞机、桨叶和轨迹发布在 `/formation/markers`，TF `formation_view` 跟随编队中心。建筑、放大机模高度和桨速是展示设置。

## 关键结果与验收

- 五组求解器初值使用相同的物理初态。每组 **203 帧 × 7 架**，记录覆盖 **0–60.6 秒**。对应的 21 对机间距及已记录最近机对可从原始位置复算。
- Seed 7 在 **6.6 秒**首次达到有效分配图连通。在 **11.1 秒**，UAV-4 和 UAV-6 的全程最小采样间距约 **118.07 m**。
- 原工作环境中，用户确认城市回放及联动启动已跑通。归档阶段复验解压数据和指纹。GUI 部分沿用既有验证。
- 上游属于工程复现，包含求解器替换与 S6 工程补全。当前包只回放平面结果。连续时间安全性、机型动力学和城市避障须在独立实验中验证。

## 后续 Gazebo 交接

先用一架四旋翼和一栋带碰撞体的楼验证“状态反馈 → ROS 2 指令 → Gazebo 动力学”，记录跟踪误差和离楼距离。再做少量机体的编队协调，最后扩到七机。四旋翼速度、间距和控制约束需与原 Case 1 参数分别定标。
