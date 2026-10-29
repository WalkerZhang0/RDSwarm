# 当前接口：2026-10-29

2026-10-29 交接入口：[loop→GTSAM 交接](loop→gtsam交接.md)。记录现有消息到后端协议的映射、未知协方差的接入边界、邻机端点状态缺口、图初始化与仿真计划；优化和协方差服务仍待实现。

2026-10-28 更新：`global_descriptor_node.py` 独立生成真实 4096-D MobileNetVLAD 摘要，`loop_baseline_node.py` 可执行自动 query→describe→fetch→BF/PnP。同 master 的摘要话题和磁盘缓存配置见 [GLOBAL_LOOP_BASELINE.md](GLOBAL_LOOP_BASELINE.md)。原 feature 节点的 metadata 仍不包含全局描述子；全局摘要由独立节点发布。单独启动 core 服务时仍需显式触发，自动编排由 baseline worker 负责。

## 数据与计算分层

VIO 的 raw 输出为 /agentX/vins/keyframes。姿态在该机局部 VIO 坐标系，
三维点在对应关键帧相机系，时间从原始整数采样时间恢复。
该输出的 descriptor_valid=false，不能直接当作已计算好的匹配描述子。
所有当前关键帧上的有效二维轨迹均可导出；未三角化、失败深度或当前帧新点的
`position_valid=false`，不再因为缺深度而丢弃二维观测。

rdswarm_feature_node 默认只缓存 raw 关键帧：
- 调用 /agentX/describe_keyframe 才对选中帧推理和采样。
- 输入准确像素位置，保持 feature_id、二维和三维对应不变。
- 默认保留全部原点并追加去重 SuperPoint 二维检测；新点不带深度。
- `camera_calib_file` 必须与 VIO 相机一致，用于新点去畸变/归一化；尺寸或既有观测不一致时拒绝。
- `append_image_features=false` 可关闭补点，仅对原始轨迹采样描述子。
- publish_result=true 时发布 /agentX/keyframes 的 enriched 数据。
- 另发 /agentX/keyframe_metadata，但其 global_descriptor_valid=false。
- model_id 来自准备模型的 JSON 或显式参数；不同模型不能直接匹配。

rdswarm_loop_node 缓存 enriched 数据，分别暴露：
- get_local_features：按键返回 LocalFeaturePacket，可清除有效三维数据，不含原图。
- fetch_peer_features：显式请求另一个 ROS 服务并缓存，不自动匹配。
- query_candidates：对已输入 summaries 的全局向量作 CPU Top-K。
- verify_loop：只使用已经缓存的本机/对端特征，进行匹配与 PnP。

上述 core 服务通过 `loop_baseline_node.py` 自动编排；MobileNetVLAD 已由独立节点接入。只启动显式 core 而不启动这些辅助节点时，不会自动检索/推理；缺少全局描述子时查询明确失败。
当前同 master 服务供开发验证，网络/跨 master 桥接未实现。

## 几何约定

body_T_camera 将相机点映射到机体。
用本机相机系三维点和对端归一化二维点进行 PnP，K=I。
本机只选择 `position_valid=true` 的描述子特征；对端候选不要求有深度。
`get_local_features(include_3d=false)` 返回全部有效描述子的二维候选，并清除三维有效性。
因此 PnP-RANSAC 门限是归一化坐标单位，不是像素。

PnP 返回 T_Cb_Ca，约束为：

~~~text
T_Ba_Bb = T_Ba_Ca * inverse(T_Cb_Ca) * inverse(T_Bb_Cb)
~~~

LoopConstraint 两端引用原始 FrameKey 与采样时刻，不转移到 UWB/集群帧。
reprojection_error_unit=normalized_camera。
geometry_verified=true 表示通过当前几何门限，不表示已经获得校准协方差。

默认 covariance_valid=false。显式配置名义噪声时来源标为 configured_nominal_not_calibrated。
协方差是 row-major，GTSAM 切空间顺序 rx ry rz tx ty tz（弧度、米）。

## 共视预留，不构建重投影

VerifiedMatches 给出两端 FrameKey、时间、模型、局部/对端 feature_id、距离和内点标志。
CoVisibilityObservationSink 只定义未来关联接入口，目前没有实现或接入重投影因子。
匹配观测与其紧凑回环是同源数据，后续不能按两份独立信息同时加入。

## 后端边界

BackendInterface 仍只定义 PoseInput/PoseConstraint、插入、更新与相对查询；
QueryRelativeState 仍是协议，没有优化服务节点。
VIO 增量的统计噪声不是两个边缘协方差直接相加得到的。
未知噪声保持无效，不用零矩阵或单位阵伪造精确信息。

## 身份、缓存与版本

FrameKey = agent_id + session_id + frame_id；本适配器 frame_id 使用采样纳秒。
VIO reset 产生新 epoch，避免与旧帧/地标混用。
`feature_id` 最高位为 1 表示当前 FrameKey 内独立检测的二维点；这些点不是跨帧 VIO 地标。
新增点的 ID 与 VIO 轨迹 ID 分离，后续共视重投影不能假定高位 ID 已有三维关联。
缓存对帧键保持不可变；更换描述子模型应重启并使用新 session/重放，而不是静默覆盖。
配置 `cache_directory` 后可使用磁盘历史与有界 RAM 工作集；未配置时保留原 RAM 行为。摘要索引仍增长，尚无长期图/存储压缩。
补点参数和标定也是缓存配置的一部分，修改后重启节点并用新 session/重放，不能覆盖同帧缓存。

描述子计算、特征请求、验证是分开的成本。对端数据缺失或模型不兼容会返回明确状态。
LocalFeaturePacket 仍复用带三维占位字段的 TrackedFeature，不代表最终紧凑网络编码。
