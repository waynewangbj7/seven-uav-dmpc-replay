# 早期记录与程序

[开发过程](../docs/process.md)按阶段串起这些记录。保留旧记录便于理解当初为何从定高、定点和逐步增加机数开始，以及每次改变是如何验收的。当前五个场景的入口见[项目首页](../README.md)。

- `x3-demos/` 内有早期单机、两机、三机、五机和七机 SDF、桥配置与启动脚本，以及理想速度和速度滞后两份 DMPC 对比模块。控制器复用项目根目录中的 `x3_altitude_demo.py`、`x3_pair_demo.py` 和 `x3_dmpc.py`。
- [早期试验 CSV](../results/stages/) 保存定高、定点、速度响应、刹停和小规模到七机的轨迹。 [演进数据](../results/history/) 保存整队绕箱、Mavic 园区、连续 Case 1、过渡场景、完整起降和冻结前代表性飞行。
- [冻结阶段记录](notes/stage-freeze.md)、[四场景记录](notes/stage-single-box.md)、[双箱阶段记录](notes/stage-dual.md) 保留各阶段的结果目录和运行口径。

在安装好项目首页所列依赖后，可从项目根目录回看早期程序。早期场景使用官方 X3 Fuel 模型。

```bash
bash history/x3-demos/run_x3_altitude_demo.sh
bash history/x3-demos/run_x3_altitude_demo.sh --pair --dmpc
bash history/x3-demos/run_x3_altitude_demo.sh --trio
```
