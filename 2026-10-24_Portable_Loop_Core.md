# 2026-10-24_Portable_Loop_Core

## 1. 修改文件清单

- src/vins_fusion/vins_estimator/CMakeLists.txt、package.xml：默认 CPU 构建、可选 TensorRT 目标和测试入口。
- src/vins_fusion/vins_estimator/src/featureTracker/superpoint_extractor*、superpoint_tensorrt.hpp：CPU facade 与独立 GPU detector。
- src/vins_fusion/vins_estimator/src/estimator/parameters.*、featureTracker/feature_tracker.cpp：显式模型/后端参数。
- src/rdswarm_features：CPU 密集 SuperPoint 推理、指定 VIO 像素的描述子采样及全局模型预留接口。
- src/rdswarm_msgs：局部特征包、已验证关联、手动 describe/fetch/query/verify 服务和质量单位。
- src/rdswarm_loop：CPU 索引、局部匹配、PnP、几何验收、显式服务节点及重投影预留接口。
- src/rdswarm_bringup/launch/mono_agent.launch、配置：可见的检测器和模型选择。
- scripts/prepare_superpoint_dense.py：单一用途的模型格式准备。
- scripts/smoke_interfaces.py：支持指定本轮 devel 空间。
- tests/*：独立夹具发布和描述子数值对照；不作为运行整个项目的黑盒入口。
- docs/LOOP_WORKFLOW.md、相关接口和验证文档、README.md：分终端流程、参数、状态和范围。

## 2. 功能实现说明

完成用户选定的前三步：依赖解耦，VIO 特征轨迹的描述子导出，CPU 回环核心。

默认不查找、链接 CUDA/TensorRT。可选 GPU 检测器保持独立；没有引入 YOLO、Torch 依赖。
CPU 路径使用 OpenCV DNN，不要求 ONNX Runtime C++ SDK。模型准备和数值对照的 Python
onnx/onnxruntime 仅是可选开发工具，不是运行节点依赖。

描述子节点保留原始 feature_id、二维和三维对应，仅在相同 VIO 像素位置采样。
默认由用户调用 describe_keyframe；取对端数据和验证也是分开的服务。
MobileNetVLAD 只预留提供器接口；CPU 索引可接收外部全局向量，不伪造模型结果。

重投影仅预留 VerifiedMatches 与 CoVisibilityObservationSink，没有构建残差。
PnP 用本机三维点、对端二维点和两端外参，结果约定 T_Ba_Bb。
默认未知协方差无效；名义噪声只有显式启用时才加入并标注来源。

## 3. 问题与解决办法

- 原模型只返回 Top-K 离散点的描述子，不能匹配任意 KLT 像素。
  截取相同权重的原始检测/描述子头，按原模型的半像素与归一化约定采样。
- 描述子采样容易混淆 H/W、半像素和归一化先后。
  通过解析子像素测试及与原 ONNX 稀疏输出的真实数值对照验证。
- 原路径强制 TensorRT/CUDA 且耦合推理。
  默认 CPU facade，GPU 文件独立成可选目标，CPU 库用 ldd 检查依赖。
- PnP 容易混淆变换方向及像素/归一化门限。
  使用非零机体外参、已知相对旋转/平移、外点和退化数据做几何测试。
- “对端只发描述子”不足以构建几何约束。
  局部包同时包含明确身份/时间、二维坐标及外参，不传原图；远端有效三维点可省略。
- 自动流水线会掩盖调度到底跳过什么工作。
  默认不自动 describe/query/fetch/PnP，工作流程与参数由用户分步控制。
- 测试结果及范围见 docs/LOOP_VALIDATION.md；未验证事项不按已完成报告。

## 4. 注意事项 (Precautions)

- 当前没有接入真正 MobileNetVLAD 推理，也没有 iSAM2 求解或调度器。
- 测试中的已知帧对、检索向量是合成夹具，不证明真实回环召回率。
- 本机/对端描述子模型、预处理和坐标约定必须一致。
- 同 master ROS 服务仅用于当前调试，实际跨 master/WiFi 适配另行实现。
- 当前局部包沿用带三维占位字段的消息，include_3d=false 不等于二进制载荷已压缩到最小。
- 模型/engine 不提交 Git；权重许可不由本代码的 GPL 自动替代。
- 接口和位姿约定变更需要所有节点重新构建；不要混用旧 devel 和新 devel_cpu。
- CPU VIO 模式默认显式使用传统检测器；选择 SuperPoint CPU/GPU 时须提供对应模型。
- GPU engine 推理、Jetson 和真实单目数据精度需要与构建/接口验证分别报告。
