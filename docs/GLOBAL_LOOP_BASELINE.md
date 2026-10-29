# 4096 维全局检索与 SP＋BF＋PnP 回环基线

日期：2026-10-28。

2026-10-29 后端交接与执行计划见 [loop→GTSAM 交接](loop→gtsam交接.md)；数据问题与单双目结论整理见 [回环问题](回环问题.md)。

**参考数据更新：** 本地 `*_gt.txt` 已追溯至原始 GNSS `/fix`；目标区间 Alpha 自报水平不确定性达数米，Carol 高度存在分段偏移。本文长度差仍可复现，但不作为已确认的算法定位误差；详情见 [S3E_REFERENCE_TIME_AUDIT.md](S3E_REFERENCE_TIME_AUDIT.md)。

## 当前交付

本轮接入 NetVLAD 系列中的 **MobileNetVLAD**，使用已有 D2SLAM 预训练 ONNX 文件，输出 4096 维图像全局描述子。它不是机器人状态，也不是临时随机向量。模型来源的调用约定已经核验，原训练 checkpoint 与训练数据的独立溯源尚未完成；当前结果用于本地模型基线。

数据流为：

```text
VIO /agentX/vins/keyframes
  ├─ features：缓存原图/观测，按需运行 SP，RAM 工作集＋磁盘历史
  └─ global_descriptor：独立 CPU worker，MobileNetVLAD → 4096-D 摘要
                                    ↓ 同 master 摘要话题
                              每台 loop_core 的 Top-K 索引
                                    ↓ baseline worker
                         describe → get/fetch → BF＋PnP
                                    ↓
                  LoopConstraint / VerifiedMatches / 来源与耗时日志
```

各机可各自运行 baseline worker；首轮双机实验可只启用 agent0 发起验证，避免把双向端点约束重复计数。两端采样时间可以不同，不把异时帧约束解释成同一时刻机间距离。

默认只由正式 VIO 关键帧触发全局推理。普通图像不会逐帧进入网络。可选超时搜索同时订阅原始图像及 `VioState`，在二者采样时间完全一致且长时间无关键帧时查询邻机摘要；输出到 `fallback_candidates`，明确为 search-only，不作为正式关键帧入库或发布位姿约束。新关键帧到达时取消尚未处理的过期 fallback。3 秒只是可配置实验起点，没有完成标定。

## 模型与依赖

- 原文件：`/media/walker/My PSSD/program/D2SLAM/models/netvlad_series/mobilenetvlad_dyn_size.onnx`。
- SHA256：`f0487199f784d1cc854dff88512fea15c9a13e1a4343fee1a41f149b92dd87bd`。
- 输入：`image:0`，NHWC `[1,H,W,1]`，灰度 float32，**数值 0–255，不除以 255**。
- 输出：`descriptor:0`，`[1,4096]`，再做 L2 归一化。
- 默认缩放：320×240，OpenCV 线性插值。尺寸、缩放和模型 SHA 写入 `model_id`；改变尺寸不能混用旧索引。
- 推理：Python ONNX Runtime CPU；本机 `/usr/bin/python3` 的 ORT 版本为 1.16.3。运行依赖见 `requirements-global.txt`。
- SP 仍使用已有 C++ OpenCV CPU 模型。全局节点不引入 TensorFlow、Torch 或 CUDA。

准备模型（先完成 CPU 构建；以下为 Bash，zsh 用相应 setup.zsh）：

```bash
cd "/media/walker/My PSSD/tmp/论文/RDSwarm"
source /opt/ros/noetic/setup.bash
catkin_make --build build_cpu -j2 -l2 \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel_cpu" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DRDSWARM_WITH_TENSORRT=OFF
source devel_cpu/setup.bash
/usr/bin/python3 scripts/prepare_mobilenetvlad.py
```

模型及 manifest 存入忽略的 `models/`，模型不提交仓库。脚本仅复制已有模型、验证真实输入输出并记录来源，不下载或重训。

## 分终端运行

每个终端 source 相同 ROS 和 `devel_cpu`。使用全新的实验目录，例如 `results/global_run01`；模型、SP 增补参数或标定变化后换目录，避免旧缓存与新配置混用。

1. 启动 ROS master。实时 VINS 按既有 S3E 流程运行，关闭旧 `loop_inputs` 缓存：`cache_keyframes:=false`。也可以仅重放记录关键帧以验证后续链路。
2. 两个描述节点（各自终端）：

```bash
roslaunch rdswarm_bringup describe_agent.launch agent_id:=0 \
  cache_directory:="$PWD/results/global_run01/agent0/features" memory_keyframes:=8
```

```bash
roslaunch rdswarm_bringup describe_agent.launch agent_id:=1 \
  cache_directory:="$PWD/results/global_run01/agent1/features" memory_keyframes:=8
```

3. 两个 loop core（各自终端）：

```bash
roslaunch rdswarm_bringup loop_agent.launch agent_id:=0 \
  summaries_topic:=/rdswarm/summaries \
  cache_directory:="$PWD/results/global_run01/agent0/loop" \
  memory_keyframes:=8 remote_cache_limit:=8
```

```bash
roslaunch rdswarm_bringup loop_agent.launch agent_id:=1 \
  summaries_topic:=/rdswarm/summaries \
  cache_directory:="$PWD/results/global_run01/agent1/loop" \
  memory_keyframes:=8 remote_cache_limit:=8
```

4. 两个全局描述节点（各自终端）：

```bash
roslaunch rdswarm_bringup global_agent.launch agent_id:=0 \
  cache_directory:="$PWD/results/global_run01/agent0/global"
```

```bash
roslaunch rdswarm_bringup global_agent.launch agent_id:=1 \
  cache_directory:="$PWD/results/global_run01/agent1/global"
```

5. agent0 的自动验证 worker：

```bash
roslaunch rdswarm_bringup loop_baseline.launch agent_id:=0 \
  output_directory:="$PWD/results/global_run01/agent0/baseline" top_k:=3
```

6. 所有节点准备好后再播放数据。记录关键帧测试：

```bash
rosbag play --clock --pause results/s3e_loop_validation/keyframes.bag
```

该 bag 不含连续普通图像/状态，因此不会触发 fallback。完整 VIO 数据回放时，可在第 4 步为 Alpha 添加：

```text
fallback_enabled:=true fallback_period_s:=3.0 image_topic:=/Alpha/left_camera/image_raw
```

Carol 使用对应图像话题。fallback 候选需另行评估端点重验证策略；当前自动 BF/PnP 主线仅处理正式关键帧。

`/rdswarm/summaries` 是同 master 的摘要分发，不代表实现了 WiFi 或跨 master 桥。未通过的候选不发布回环约束；通过者仍默认 `covariance_valid=false`，当前没有 GTSAM 消费节点。

## 日志、历史与共视接口

- `global/global.jsonl`：模型身份、加载时间、采样时间、排队、推理、触发原因、cache hit、队列丢弃和 fallback 候选。
- `baseline/loops.jsonl`：候选 rank/cosine、两端 FrameKey、特征准备/请求耗时、2D 包字节、BF/PnP 验证状态。
- features/loop 终端：SP 推理及增补、BF 匹配、PnP 几何耗时与 RANSAC 内点。
- `baseline/attempts.sqlite`：已完成帧对去重；服务错误不写为已完成，以便后续再次出现时重试。
- `baseline/constraints/`：接受约束、VerifiedMatches、两侧 packet 和 provenance。原图不在对端特征 packet 中。
- `CoVisibilityBundleSink`：预留同时接收两端 packet、已验证关联及观测来源的接口；没有实现跨帧地标跟踪或重投影因子，也不将新增高位 ID 当作跨帧地标。

Raw/描述帧可持久化，RAM 上限只限制工作集，不删除历史关键帧。loop core 缓存会移除原图。摘要索引仍随历史线性增长，约 16 KiB/帧的向量另加元数据；磁盘容量也需纳入长期实验预算。

队列满时明确记录丢弃；全局 raw 输入已存盘，可重新回放恢复，不能把丢弃计作算法拒绝。`rospy.wait_for_service` 有可用性超时，但 ROS1 同步 service call 本身没有可靠的执行超时，本轮未实现跨网络故障恢复。模型推理在独立 worker；SP 服务仍是单节点串行，双向 worker 会竞争该服务。

## 无 ROS master 的记录序列实验

```bash
/usr/bin/python3 scripts/evaluate_global_loop.py \
  --keyframes results/s3e_loop_validation/keyframes.bag \
  --output results/global_loop_new --top-k 3 --max-points 400
```

脚本按 bag 实际读取顺序，仅检索已读到的邻机描述子。每个新帧先查询历史，再加入数据库；不使用 GT 或未来帧生成候选。局部特征按需调用同一个 C++ `enrichKeyframe`，PnP 使用原有默认门限。该实验没有并行服务到达抖动，因此与在线异步结果不要求逐对完全相同。

实验参数：全局 320×240、2 ORT 线程；SP 原始 1224×1024、最多追加 400 点、16×12 网格、4 px 去重；BF ratio 0.8、距离上限 0.9；PnP 归一化阈值 0.004、200 次、seed 17、最少 8 内点、内点率 0.5。

2026-10-28 在既有 40 秒记录上得到：

| 指标 | 实测 |
| --- | ---: |
| 关键帧 | Alpha 45、Carol 67，共 112 |
| Top-3 不重复跨机帧对尝试 | 318 |
| 几何验收通过 | 95 |
| 匹配不足 | 95 |
| PnP 内点数/比例不足 | 128 |
| 处理异常 | 0 |
| 全局计算中位 / P95 | 8.43 / 10.91 ms |
| SP 推理＋增补中位 / P95 | 407.36 / 424.93 ms |
| 顺序离线运行墙钟 | 79.29 s |

这 95 对是几何接受的帧对，不是 95 个独立重访事件，也不是已确认的 95 个正确回环；当前未标定 cosine 阈值，默认 -1 表示 Top-K 全送几何验证，便于先建立基线。

95 对均可在文本参考不超过 1.5 秒的间隔内插值得到位置参考点间距。估计平移模长与其间距的绝对差：中位 0.919 m、P95 1.798 m、最大 2.491 m。这些仅是与未经认证 GNSS 衍生参考的长度差，不是已确认的算法定位误差，也不是误回环率；参考质量、高度处理、物理时间对齐和杆臂均未核实。不能用这些差值直接归因于 VIO 深度/尺度错误或认证后端协方差。

对应结果：`results/global_loop_recorded_40s/summary.json`、`events.jsonl`、`global_descriptors.npz`、`gt_length.json` 以及逐帧对 JSON。每次实验使用新目录；全分辨率 raw/SP 产物占磁盘，当前该次运行约 615 MB。

## 验证与后续边界

CPU 构建与五项 CTest，包括新增磁盘历史/重启/损坏数据测试。真实模型测试验证 4096 维、L2、参考 0–255 NHWC 预处理一致性及重复输入。

隔离 ROS 集成测试：16 个真实记录关键帧、16 个全局摘要，自动验证至少 8 个候选并产生几何接受；RAM 工作集设为 1，验证历史磁盘取回、无对端原图和 fallback 只搜索/限频。

```bash
/usr/bin/python3 tests/global_descriptor_test.py
ctest --test-dir build_cpu --output-on-failure
/usr/bin/python3 tests/global_loop_services_test.py --output logs/global_loop_new
```

下一阶段先审查完整时序的候选质量和几何误差，再接入 GTSAM 增量优化与联合协方差。共视地标融合仍为预留，LightGlue 不进入本轮主链路。
