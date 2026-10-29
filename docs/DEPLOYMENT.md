# CPU 部署与仓库上传范围

日期：2026-10-29。适用当前 ROS1 Noetic 源码工作区，首选与验证机一致的 Ubuntu 20.04、系统 Python 3.8、系统 OpenCV 4.2。其他 CPU 架构/系统须重新构建和验证；本次不提供已验证的 ARM/容器发行包。

## 1. 仓库包含与外部依赖

包含源码、消息/服务、启动文件、CPU Python 节点、测试、准备模型及离线诊断脚本、必要配置和技术文档。数据集、结果图片/JSON/缓存、bag/db3、日志、build/devel/install、ONNX 权重及 engine 不上传。

**仍需 Ceres。** 当前导入的 VINS 和 camera_models 仍链接 Ceres；GTSAM 集群后端仅有协议/校验，尚无 iSAM2 优化实现。不能将本版本描述为全工程已完全去除 Ceres。

在已有 ROS Noetic 环境安装编译依赖，系统级操作由部署者按目标机权限执行：

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libeigen3-dev libceres-dev libopencv-dev \
  libboost-filesystem-dev libboost-program-options-dev libboost-system-dev \
  python3-pip python3-catkin-pkg python3-yaml python3-numpy python3-opencv \
  ros-noetic-roscpp ros-noetic-rospy ros-noetic-cv-bridge ros-noetic-tf \
  ros-noetic-image-transport ros-noetic-rosbag ros-noetic-rviz
```

另需提供可被 CMake `find_package(GTSAM 4.2 REQUIRED)` 找到的 GTSAM 头文件与库；本机历史环境为 4.3a1，版本兼容需在目标机验证。如安装在独立前缀，可给构建传 `-DGTSAM_DIR=/实际前缀/lib/cmake/GTSAM`，路径以实际安装为准。不要复制本机编译产物到其他 CPU 架构。

## 2. 获取与构建

```bash
git clone git@github.com:WalkerZhang0/RDSwarm.git
cd RDSwarm
source /opt/ros/noetic/setup.bash
/usr/bin/python3 -m pip install --user -r requirements-global.txt
catkin_make --build build_cpu -j2 -l2 \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel_cpu" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DRDSWARM_WITH_TENSORRT=OFF
source devel_cpu/setup.bash
ctest --test-dir build_cpu --output-on-failure
```

此处是 Bash；zsh 请 source 相应 setup.zsh。不要叠加旧 VINS-Fusion 工作区的同名 vins/camera_models。OpenCV_DIR 按目标机调整，并与 cv_bridge 匹配；ARM 上目录名不会是 x86_64-linux-gnu。Noetic 的 cv_bridge 等使用系统 Python，避免未经适配切换到 Conda Python。ORT 必须有目标 Python/CPU 架构对应的安装包；requirements 版本只代表现有 CPU 验证环境。

## 3. 单独准备模型

权重独立传输/按原来源获取，遵循原许可。不会随 git clone 获得，也没有脚本自动下载。

| 模型 | 仓库运行时位置 | 已验证 SHA256 |
|---|---|---|
| SuperPoint dense | `models/superpoint_dense.onnx` | c0445fe3b891fd5b4d6ac7808175a7c8b2637f61f24ae1aff6527e2cfb254eba |
| MobileNetVLAD | `models/mobilenetvlad.onnx` | f0487199f784d1cc854dff88512fea15c9a13e1a4343fee1a41f149b92dd87bd |

如果已有当前模型，可复制对应 ONNX 和 JSON manifest 到目标工作区 models/，核对 SHA256。模型 identity、SP 采样契约及 VLAD 预处理必须一致，不能仅保证文件名相同。VLAD 为 NHWC 灰度 float32 **0–255、不除以255**，320×240，输出 4096 维归一化描述子。

若只有已有来源模型，使用明确的目标机路径准备，不依赖脚本中的原作者路径默认值：

```bash
/usr/bin/python3 -m pip install --user 'onnx==1.15.0'
/usr/bin/python3 scripts/prepare_superpoint_dense.py \
  /目标机/原始superpoint.onnx models/superpoint_dense.onnx
/usr/bin/python3 scripts/prepare_mobilenetvlad.py \
  --source /目标机/mobilenetvlad_dyn_size.onnx --output models/mobilenetvlad.onnx
sha256sum models/superpoint_dense.onnx models/mobilenetvlad.onnx
```

SP 准备脚本只支持包含所需 detector/descriptor 原始 head 的兼容图；不能将任意 SP ONNX 当作等价输入。可通过 `--detector-head` / `--descriptor-head` 显式设置兼容节点名称；不确定时优先迁移已经验证的 dense ONNX/manifest。`onnx` 仅准备 SP 时需要，主运行不依赖 Torch、TensorFlow 或 CUDA。

## 4. 标定与启动

S3E Alpha/Carol 左目与 VINS 配置已随源码放在：

```text
src/rdswarm_bringup/config/s3e/alpha_cam0.yaml
src/rdswarm_bringup/config/s3e/carol_cam0.yaml
src/rdswarm_bringup/config/s3e/alpha_vins.yaml
src/rdswarm_bringup/config/s3e/carol_vins.yaml
```

S3E 启动入口默认读取包内配置，输出独立写到 results_dir/output_path。`describe_agent.launch` 默认使用相同左目标定。Alpha acc_n=0.1 是当前诊断实验值，其余配置也未经完整噪声一致性认证，不能视为生产标定。自己的机器人必须提供实际左右/IMU 标定、图像尺寸和噪声，不复用 S3E 外参。

- 传感器/VIO 启动：见 [S3E_DUAL_VINS.md](S3E_DUAL_VINS.md)。`config_file` 或 `config_dir` 可覆盖包内默认。
- 回环主链路：见 [GLOBAL_LOOP_BASELINE.md](GLOBAL_LOOP_BASELINE.md)，按终端启动 features、loop_core、global_descriptor、baseline。
- 整体运行/后端边界：见 [loop→gtsam交接.md](loop→gtsam交接.md)。

用于回放的 bag 需独立放在目标机，现有文档中的 `results/...` 为本地实验约定，不是仓库附件。数据转换工具为 `scripts/convert_s3e_ros1.py`，所需源文件和额外依赖见 [S3E_CONVERSION.md](S3E_CONVERSION.md)。

每次实验使用新输出目录和清晰 session。更换模型、标定或 SP 补点参数后清晰划分缓存，不混用旧 frame 数据。全分辨率 SP 较慢；当前未保证整套实时运行。

## 5. 单台机器迁移与多主机部署的区别

本版本支持把源码、配置和外部模型迁移到另一台机器后重新构建，并在同一 ROS master 下运行多个 agent。**不包含跨 master 桥或经过验证的多主机部署方案。**

未来若分到多台 ROS1 机器，可在同一可达 master 下显式配置 ROS_MASTER_URI 及各机可路由的 ROS_IP/ROS_HOSTNAME，并保证 agent 命名空间/ID 唯一、服务名与 `agent_namespaces` 对应。ROS1 动态端口、DNS/路由、防火墙、断线重连和吞吐须独立验证；这不是已有 WiFi 传输实现。与外网隔离的可信局域网部署策略由使用者按实际网络确定。

当前 remote feature packet 不含原图，但真实多主机仍需各机就地处理图像。不能为了共享摘要，把原始图像全部通过远端网络传给同一节点并认为带宽不变。后端所需邻机状态/里程计摘要传输尚待设计。

## 6. 验证层级

1. 不依赖数据集的 5 项 CTest：采样、特征增补、缓存历史、PnP、后端输入契约。
2. 准备好模型和任意有效灰度图后运行真实 VLAD：

```bash
/usr/bin/python3 tests/global_descriptor_test.py --image /目标机/测试灰度图.png
```

3. 有相同关键帧记录时运行 `tests/global_loop_services_test.py`；它使用本地 keyframes.bag，不是 git 自带样例。
4. 全部部署验收完成后才报告目标机可用。现有结果在 git 中只保留技术记录；图片/报告链接指向未上传的本地实验产物时，需要自行运行对应脚本生成。
