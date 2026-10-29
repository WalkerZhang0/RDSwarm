# LightGlue ONNX CPU 离线匹配

## 接入边界

当前系统使用 `/home/walker/LightGlue-ONNX/weights/512/superpoint_lightglue.onnx` 的 matcher-only 模型。该模型输入两侧的关键点和 256-D SuperPoint 描述子，输出动态分配分数矩阵 `[1, N0, N1]`。

本仓库没有 ONNX Runtime C++ 库；系统 OpenCV DNN 4.2 也不能稳定加载这个动态 LightGlue 图。因此 LightGlue 作为独立 Python CPU 探针运行，不侵入 ROS C++ 节点、VIO、`feature_id` 或 GTSAM 接口。探针只替换局部匹配阶段，结果重新交给现有 `loop_pair_probe` 执行相同的 PnP-RANSAC、正深度、空间退化和重投影验收。

## 模型选择

不要使用端到端 `SuperPoint+LightGlue` 模型做当前 A/B。端到端模型会重新检测特征，无法保留 RDSwarm 的 VIO 三维点和 `feature_id` 对应关系。使用 matcher-only 模型：

```text
/home/walker/LightGlue-ONNX/weights/512/superpoint_lightglue.onnx
```

LightGlue 输入的关键点由 packet 中的原始像素坐标按图像尺寸归一化；描述子保持现有 256-D SuperPoint 描述子。发起端只发送 `position_valid=true` 的本机特征，邻机端可以使用全部有效二维描述子。

## 运行

先构建 CPU 工作区并准备 LightGlue-ONNX 虚拟环境：

```bash
cd "/media/walker/My PSSD/tmp/论文/RDSwarm"
env -i HOME="$HOME" PATH="$PATH" bash --noprofile --norc -c \
  'source /opt/ros/noetic/setup.bash && source devel_cpu/setup.bash && \
   python3 scripts/lightglue_pair_probe.py \
     results/peer_2d_union/pair_2_local.bin \
     results/peer_2d_union/pair_2_peer.bin \
     results/lightglue_pair_2.json \
     --size-a 1224 1024 --size-b 1224 1024 \
     --model /home/walker/LightGlue-ONNX/weights/512/superpoint_lightglue.onnx \
     --threads 8 --keep-matches'
```

`--size-a/--size-b` 按顺序传入两端原始图像宽、高；离线 packet 不保存图像尺寸，因此必须从生成 packet 的关键帧/图片元数据中明确传入。实际执行时 Python 解释器应使用包含 `onnxruntime` 和 ROS Python 消息的环境。若使用 LightGlue-ONNX 的 Python 3.10 虚拟环境，需要额外让其能够导入 `devel_cpu` 生成的 `rdswarm_msgs`；也可以在系统 Python 中安装 `onnxruntime` 后直接执行。

探针输出字段包括：

- `lightglue_model_load_ms`：创建 ORT session 的时间；首次运行成本，不应混入稳态平均值。
- `lightglue_warmup_ms`：第一次执行时间；用于区分图优化/缓存成本。
- `lightglue_inference_ms`：第二次 LightGlue CPU 推理时间。
- `lightglue_decode_ms`：互检和置信度阈值解码时间。
- `matching_ms`：C++ 几何核接收外部匹配后的装配时间。
- `geometry_ms`：PnP-RANSAC、LM refinement、正深度和重投影验收时间。
- `total_ms`：从 C++ 外部匹配入口开始的总时间，不包括 Python 模型加载和 ORT 推理；完整端到端时间需将 `lightglue_total_ms` 与 `total_ms` 相加，并明确是否包含 session 初始化。

## 与 BF 基线对照

同一 packet 先运行原有 BF：

```bash
devel_cpu/lib/rdswarm_loop/loop_pair_probe \
  results/peer_2d_union/pair_2_local.bin \
  results/peer_2d_union/pair_2_peer.bin \
  results/bf_pair_2.json
```

再运行上述 LightGlue 探针。两者必须固定：输入 packet、特征预算、相机归一化、PnP 门限和 RANSAC seed。比较 `matches`、`inliers`、`rms_normalized`、是否通过以及耗时；不能只比较 LightGlue 输出的匹配数量。

注意：旧 packet 格式保持兼容；匹配探针的图像宽高作为单独参数输入，不修改 ROS 消息协议。

## 当前 pair_2 基线

使用同一对 `pair_2` 原始图像和 `models/superpoint_dense.onnx` 的离线结果：SuperPoint A 为 1024 点、483.39 ms，B 为 928 点、479.59 ms。对应 packet 上，BF 为 24 匹配、15 内点并通过；LightGlue matcher-only 为 74 匹配、21 内点，但因内点率低于 0.5 被拒绝。LightGlue 稳态 ONNX CPU 推理约 90--104 ms；首次模型加载约 635 ms，warm-up 约 145 ms。详细结果保存于 `results/lightglue_benchmark/sp_pair2_timing.json`、`results/lightglue_pair_2.json` 和 `results/bf_pair_2_current.json`。
