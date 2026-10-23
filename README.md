# RDSwarm

单目 VIO 集群的异步观测与调度研究框架。每机独立运动基底和协同后端；
当前第一阶段先导入 VINS-Fusion，打通关键帧与回环/后端接口。

## 当前范围

- 已导入用户工作副本的 VINS estimator、SuperPoint 检测器和 camera_models。
- 已提供 FrameKey / Keyframe / VioState / LoopCandidate / LoopConstraint、
  带噪声有效性标记的运动约束、后端状态及相对协方差查询接口。
- 已提供关键帧缓存与按需读取服务。
- 后端当前是 Pose3 类型与输入校验接口，尚无完整 iSAM2 求解器。
- 回环当前是输入、缓存和接口，尚无描述子导出、检索、匹配或 PnP。
- 不启用 UWB、分布式共识、VINS loop_fusion 或结果反馈。

## 工作区

本仓库本身是 ROS Noetic catkin 工作区。src/vins_fusion 是选择性源码导入，
不嵌套旧 .git，不修改原项目。原生包名 vins 和 camera_models 保留，
构建时请只 source /opt/ros/noetic/setup.bash，避免叠加旧工作区的同名包。

GPU 用于已有 TensorRT 10 SuperPoint；后续 GTSAM 后端和调度使用 CPU。
GTSAM 4.2+ 接口用于基础契约编译；实际稳定发布版本须在部署前固定。
本机若使用 4.3a1，仅能记录为开发版构建验证，不能声称已锁定稳定依赖。

## 构建

~~~bash
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DTENSORRT_ROOT=/usr/local/TensorRT-10.8.0.43 \
  -DCUDA_ROOT=/usr/local/cuda
source devel/setup.bash
~~~

不将模型权重、engine、ROS bag、build/devel 或日志提交到 Git。
详细接口见 docs/INTERFACES.md；来源记录见 docs/THIRD_PARTY.md。
本机示例使用系统 OpenCV 4.2，必须与实际 cv_bridge 的 ABI 一致；其他平台设置对应 OpenCV_DIR。

## 启动

~~~bash
export RDSWARM_SUPERPOINT_ENGINE=/absolute/path/to/superpoint.engine
roslaunch rdswarm_bringup mono_agent.launch
roslaunch rdswarm_bringup dual_agent.launch
~~~

双机的图像/IMU话题由 image_a、imu_a、image_b、imu_b 参数指定。
自采数据通过 config_a、config_b 指定真实标定，默认仅为 EuRoC 示例。
engine 不随仓库分发；该导入代码使用 TensorRT 10 API，Jetson 兼容性需单独验证。

## 第一阶段验证

~~~bash
python3 scripts/smoke_interfaces.py
python3 scripts/smoke_interfaces.py --engine /absolute/path/to/superpoint.engine
~~~

catkin 编译及原生接口 smoke 已通过；第二条命令额外检查双节点启动与正常退出，
不输入图像，不执行 engine 推理。详情见 docs/VALIDATION.md。
已实现范围见 docs/PHASE1.md，修改记录见 2026-10-23_Init_RDSwarm_Framework.md。
