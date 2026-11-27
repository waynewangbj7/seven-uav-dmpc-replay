# 复跑与排查

主入口见[项目首页](../README.md)。测试环境为 WSL 2 下 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、Python 3.12、CasADi 3.7.2 和 NumPy 1.26.4。Ubuntu 上先安装 ROS 2 Jazzy 和对应的 `ros_gz` 套件。 [Gazebo 与 ROS 官方配对](https://gazebosim.org/docs/harmonic/ros_installation/)

## 复跑命令与输出

在本目录的上一级运行 `bash run.sh --help` 查看完整参数。固定入口用 `bash run.sh dual`，计时范围从起飞控制到七机接地。批量运行可加 `--exit-on-finish` 让窗口在通过后关闭，桌面渲染故障排查可以加 `--headless` 跑物理验证。每轮消耗约 5 分钟仿真时间，双箱约 5 分半钟。

已有本地虚拟环境时，设置 `CASE1_PYTHON` 指向该环境的 `bin/python`。ROS 2 安装在其他路径时可设置 `ROS_SETUP`。输出目录使用新名称。启动器建立独立的 Gazebo Transport 分区，已存在同域的闭环任务时为新任务指定不同的 `ROS_DOMAIN_ID`。

## 机模与缓存

原机使用 Gazebo Fuel 的 Open Robotics X3 UAV 第 4 版。启动器从 Fuel 缓存读取 `model.sdf`，按 SHA-256 核对版本。缓存路径不标准时可用 `X3_MODEL_SDF` 指定文件。模型下载命令使用项目首页列出的地址。项目资产含可替换的外观网格和园区模型。脚本检查文件后，在临时目录为本轮生成可供 Gazebo 加载的场景和机模。 [X3 UAV 官方资源](https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/4)

## WSLg 渲染

用 `glxinfo -B` 确认渲染设备。  `llvmpipe` 表示由 CPU 渲染。经实测，旧的 Windows 驱动环境已能通过 WSLg 使用 RTX 2050，四轮冻结实验用的也是这一配置。NVIDIA 环境可按次启动：

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA bash run.sh dual
```

遇到 Qt 的 OpenGL 上下文错误时，可以先结束自己已完成的 Gazebo 窗口，再用普通启动或无界面模式确认仿真和桥是否正常。关注实际错误、端点与里程计，常见抗锯齿告警可结合实际画面判断。 [微软 WSL 图形支持](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) · [Gazebo Harmonic 排查](https://gazebosim.org/docs/harmonic/troubleshooting/)

## 归档和 GitHub 上传

将 `gazebo-case1/` 放在现有回放仓库根目录下。旧仓库的 `replay/`、`case-data/` 和自己的清单继续用于 RViz 回放。新目录内以 `results/MANIFEST.json` 提供各 CSV、JSON 和运行数据文件的 SHA-256。代码、模板、过程记录、实验表与图片可一并提交。
