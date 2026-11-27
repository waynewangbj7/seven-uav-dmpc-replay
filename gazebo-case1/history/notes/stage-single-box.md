> 历史实验记录，命令中的路径对应原归档环境。当前入口见[项目首页](../../README.md)。

# 七机单箱避障变体

三个变体由冻结包 `case1_cycle_v1_20261127` 生成，变动量是橙色箱体的世界坐标 y。Gazebo 的箱体碰撞模型和控制器使用的地图坐标同步移动。各轮沿用同一套控制器、七机初始机位、机身模型、园区道路和停机坪。

| 变体 | 箱体 y（m） | 要观察的变化 |
|---|---:|---|
| A_middle_lane | −1.732 | 原直飞航线上的 UAV-2 和 UAV-6 是否转为绕行 |
| B_outer_lane | −3.464 | 最外侧 UAV-7 是否能单独绕行 |
| C_reduced_clearance | +0.160 | 原四架机绕行时，箱体向绕行侧靠近后的净距 |

`comparison.csv` 汇总三轮飞行。各变体目录包含本轮场景、坐标配置、控制台日志和完整轨迹。轨迹图的冻结版对照使用冻结复跑第 4 轮，标注的机间距为绕行段 21 对机心距离的采样最小值。障碍净距是根据机体半径与障碍几何估算的采样值。

在原 Ubuntu 中重看一个变体时可把 `A_middle_lane` 替换成表中的名称。

```bash
cd $HOME/seven_uav_gazebo/obstacle_variants_20261127/A_middle_lane
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  bash case1/run_avoidance.sh --cycle --output "$PWD/demo_$(date +%Y%m%d_%H%M%S)"
```
