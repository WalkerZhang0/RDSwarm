# RDSwarm

单目 VIO 集群的异步观测与调度研究框架。新工程独立维护，不直接修改原 VINS-Fusion/Omni-swarm。

**当前操作入口：[回环工作流程](docs/LOOP_WORKFLOW.md)**。
该文档按终端和服务逐项说明运行、数据流、参数和 debug，不要求一个脚本启动全部模块。

## 已实现

- 导入的单目 VIO、原生关键帧/状态/二维—三维—feature_id 对应。
- 默认 CPU 构建；OpenCV SuperPoint CPU 推理；可选独立 TensorRT 检测器。
- 在既有 VIO 像素位置采样 256-D SuperPoint 描述子，不重新按检测点顺序配深度。
- 显式 describe_keyframe、get_local_features、fetch_peer_features、query_candidates、verify_loop 服务。
- CPU 全局向量 cosine Top-K、互检/ratio 局部匹配、PnP-RANSAC 和几何验收。
- VerifiedMatches 与共视接口预留，不构建机间重投影残差。

## 明确未实现

- 真正的 MobileNetVLAD 推理；索引目前接受外部提供的全局描述子。
- iSAM2 求解和相对协方差服务实现；后端当前仍是 Pose3 协议/校验接口。
- 调度器、UWB、ADMM、跨 ROS master/WiFi 通信桥、VIO 反馈和长期图压缩。

CPU 代码没有 YOLO、Torch、CUDA 或 TensorRT 的强制依赖。
源码保留可选 TensorRT 10 插件，但默认 OFF；CPU 构建不查找或链接它。

## 构建

在新终端只 source 基础 ROS 和本工作区，不叠加原工程同名 vins/camera_models：

~~~bash
source /opt/ros/noetic/setup.bash
catkin_make --build build_cpu -j2 -l2 \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel_cpu" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DRDSWARM_WITH_TENSORRT=OFF
source devel_cpu/setup.bash
~~~

OpenCV_DIR 应匹配该机 cv_bridge。本机使用系统 OpenCV 4.2。
模型/engine 不随仓库分发，显式准备见工作流程。
GTSAM 本机头文件为 4.3a1，当前仅验证后端接口构建，不宣称稳定版部署或完整优化。

## 逐步测试

以下测试可单独运行，不需要启动整套项目：

~~~bash
devel_cpu/lib/rdswarm_features/superpoint_sampling_test
devel_cpu/lib/rdswarm_loop/loop_core_test
devel_cpu/lib/rdswarm_backend/constraint_contract_test
~~~

- [工作流程与参数](docs/LOOP_WORKFLOW.md)
- [当前接口](docs/INTERFACES.md)
- [本轮验证记录](docs/LOOP_VALIDATION.md)
- [本轮修改记录](2026-10-24_Portable_Loop_Core.md)
- [来源与许可证](docs/THIRD_PARTY.md)
- [第一阶段历史记录](docs/PHASE1.md)

合成几何、数值对照和 ROS 接口测试不等于真实多机回环精度/召回率验证。
