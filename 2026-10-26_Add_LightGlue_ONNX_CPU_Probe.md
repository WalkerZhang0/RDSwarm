# 2026-10-26_Add_LightGlue_ONNX_CPU_Probe

## 1. 修改文件清单
- `src/rdswarm_loop/include/rdswarm_loop/loop_core.hpp`：增加外部匹配输入和匹配/几何耗时诊断字段。
- `src/rdswarm_loop/src/loop_core.cpp`：保留 BF ratio＋互检基线，增加外部 LightGlue 匹配进入同一 PnP 几何核的路径。
- `src/rdswarm_loop/src/loop_node.cpp`：输出 matcher、matching、geometry 和 total 耗时日志。
- `src/rdswarm_loop/src/feature_node.cpp`：输出 SuperPoint 推理、特征增补和总耗时日志。
- `src/rdswarm_loop/test/loop_core_test.cpp`：增加外部匹配路径的几何回归。
- `src/rdswarm_loop/test/pair_probe.cpp`：支持读取外部匹配索引并调用统一几何验证。
- `scripts/lightglue_pair_probe.py`：新增 matcher-only LightGlue ONNX CPU 离线探针、匹配解码、耗时记录和 PnP 调用。
- `docs/LIGHTGLUE_CPU_PROBE.md`：记录模型选择、运行命令、耗时字段和 BF 对照方法。
- `README.md`：增加 LightGlue CPU 探针入口。

## 2. 功能实现说明
使用 `/home/walker/LightGlue-ONNX/weights/512/superpoint_lightglue.onnx` 的 matcher-only 模型。探针复用 RDSwarm 已有的 256-D SuperPoint 描述子和原始像素坐标，只替换局部匹配器；本机三维点、邻机二维点、feature_id、PnP-RANSAC、外参变换和几何验收均保持不变。

日志现在分别记录：SuperPoint `infer_ms/enrich_ms/total_ms`，BF 或外部 matcher 标识，C++ 匹配装配时间，PnP 几何时间和总时间。LightGlue 探针额外记录模型加载、warm-up、稳态 ONNX CPU 推理和分数解码时间。

## 3. 问题与解决办法
系统没有 ONNX Runtime C++ 库，OpenCV DNN 4.2 也不能稳定加载动态 `[1,N0,N1]` LightGlue score 图。因此采用独立 Python ONNX Runtime CPU 探针，并把外部匹配索引交给现有 C++ `loop_pair_probe`，避免把 Python/LightGlue 依赖侵入 ROS C++ 节点。曾尝试把图像尺寸加入 `LocalFeaturePacket`，导致旧序列化 packet 不兼容，已撤销；探针改为显式接收两端图像宽高。

## 4. 注意事项 (Precautions)
- 当前 LightGlue 是离线 A/B 匹配器，不是 `rdswarm_loop_node` 内部的在线 matcher 替换；在线 ROS 路径仍使用 BF ratio＋互检。
- 端到端 SuperPoint+LightGlue 模型没有使用，因为它会重新检测点并破坏现有 VIO 三维点与 `feature_id` 对应关系。
- 在现有 `pair_2` 上，SuperPoint A/B 分别耗时 483.39/479.59 ms；BF 为 24 匹配、15 内点并通过；LightGlue 为 74 匹配、21 内点，但内点率约 0.28，低于当前 0.5 门限，因此被拒绝。匹配数量增加不等于回环验证成功。
- LightGlue 稳态 CPU 推理约 90--104 ms，首次模型加载约 635 ms，warm-up 约 145 ms；这些数字只代表单个帧对和当前 CPU/线程配置。
- 当前结果未证明 LightGlue 在完整数据集上提升召回率或精度，需要后续多帧正负样本和固定预算对照。
