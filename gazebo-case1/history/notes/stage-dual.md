> 历史实验记录，命令中的路径对应原归档环境。当前入口见[项目首页](../../README.md)。

# 七机双箱串联最终场景

同一次 Gazebo 飞行从起飞坪转入道路上空，完成 Case 1 编队，随后分别绕过两个箱体、归队并在七个停机标记上着陆。

第一个箱体位于 (58.10, −1.73) m，分配给 UAV-2 和 UAV-6。第二个箱体位于 (66.60, −3.46) m，分配给 UAV-7。两箱纵向相隔 8.50 m，停机坪位于第二箱之后。场景基于已知静态地图生成绕行轨迹。验收用各机相对原航线的横向偏差判断进入绕行和恢复队形，分界值为 0.10 m。

## 在 Ubuntu 重跑

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  bash $HOME/seven_uav_gazebo/final_dual_case1_20261127/case1/run_avoidance.sh --dual \
  --output "$HOME/seven_uav_gazebo/final_dual_case1_20261127/demo_$(date +%Y%m%d_%H%M%S)"
```

同一窗口已在运行时，关闭该窗口后再用默认 ROS 域重跑。冻结包包含控制代码、场景、模型和 SHA-256 清单。当前配置使用原 Ubuntu 的 ROS 2 Jazzy、Gazebo 和 `$HOME/seven_uav_gazebo/.venv-dmpc`。恢复时把压缩包解压到原 WSL 目录，再为最终目录下的 `.venv-dmpc` 建立指向该虚拟环境的符号链接。

`verification.json` 记录分段的绕行机号、复队间隔、采样最小机间距和按机体包络估算的障碍净距。 `formation` 和 `avoidance` 目录保存仿真轨迹，`landing_states.csv` 保存逐机下降和停稳记录。
