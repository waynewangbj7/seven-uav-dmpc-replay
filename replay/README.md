# 回放：ROS 2 / RViz 城市轨迹回放

把第一阶段算出来的七机轨迹数据（`case-data/`）在 ROS 2 + rviz2 里重放，带浏览器控制面板。

**这是数据回放，不是物理仿真**——没有动力学、没有碰撞。Gazebo 闭环见 [../gazebo-case1/](../gazebo-case1/)。

---

## 归档内容

- `replay/` 包含 Python 发布节点、播放控制、浏览器面板、RViz 预设，以及 Mavic 风格机身、旋转桨叶和可选固定翼机模
- `../case-data/` 包含 seed 7–11 的原始状态、误差与间距、事件、求解摘要、对应连接历史及源文件指纹
- `../VERIFICATION.json` 记录从 ZIP 解压后运行的五组数据检查；每个归档文件的 SHA-256 见 `../MANIFEST.json`
- 上游算法、Python 求解及 MATLAB 验证见 `uav_dmpc_learning_baseline_2026-11-09.zip`

---

## 启动

**在仓库根目录执行**（不是 `replay/` 里）：

```bash
source /opt/ros/jazzy/setup.bash
python3 replay/replay_case1.py --archive "$PWD/case-data" --rviz
```

脚本会用包内的城市配置打开 RViz。左侧应出现 `City` 和 `Seven UAVs`。在浏览器打开终端显示的地址，默认是 `http://localhost:8765/`。

面板支持暂停、前后单步、拖动进度条和事件跳转。可点 **6.6 s · S6**，再点「上一帧」对照合并前后。

RViz 的 `Views` 可选 `Follow formation`、`City flyby` 和 `Full route`。

| 参数 | 作用 |
|---|---|
| `--seed 8` | 切换种子（7–11） |
| `--speed 0.5` | 慢放 |
| `--panel-port 8766` | 更换面板端口 |
| `--mesh replay/fixed_wing.obj` | 换回固定翼机模 |
| `--check-data` | 只校验数据，不需 ROS |

Mavic 机身与桨叶需保留在同一目录。

---

## 核心原理

上游 Python 工程版按每 **0.3 秒**保存七机的平面位置、航向和角色。回放节点用同一个时钟驱动画面与浏览器面板。

画面在相邻采样帧之间**插值**，事件、最近机对和距离仍对应原始采样点。

| 话题 | 内容 |
|---|---|
| `/formation/city` | 城市静态标记 |
| `/formation/markers` | 飞机、桨叶、轨迹 |
| TF `formation_view` | 跟随编队中心 |

建筑、放大机模高度和桨速是**展示设置**，不参与任何计算。

---

## 关键结果与验收

- 五组求解器初值使用相同的物理初态。每组 **203 帧 × 7 架**，记录覆盖 **0–60.6 秒**
- seed 7 在 **6.6 秒**首次达到有效分配图连通
- 在 **11.1 秒**，UAV-4 和 UAV-6 的全程最小采样间距约 **118.07 m**
- 该数值已[独立复算](../docs/独立复算.md)验证一致

**边界**：本包只回放平面结果。连续时间安全性、机型动力学和城市避障须在独立实验中验证（见 `../gazebo-case1/`）。
