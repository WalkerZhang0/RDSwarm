# 当前接口：2026-10-24

## 数据与计算分层

VIO 的 raw 输出为 /agentX/vins/keyframes。姿态在该机局部 VIO 坐标系，
三维点在对应关键帧相机系，时间从原始整数采样时间恢复。
该输出的 descriptor_valid=false，不能直接当作已计算好的匹配描述子。

rdswarm_feature_node 默认只缓存 raw 关键帧：
- 调用 /agentX/describe_keyframe 才对选中帧推理和采样。
- 输入准确像素位置，保持 feature_id、二维和三维对应不变。
- publish_result=true 时发布 /agentX/keyframes 的 enriched 数据。
- 另发 /agentX/keyframe_metadata，但其 global_descriptor_valid=false。
- model_id 来自准备模型的 JSON 或显式参数；不同模型不能直接匹配。

rdswarm_loop_node 缓存 enriched 数据，分别暴露：
- get_local_features：按键返回 LocalFeaturePacket，可清除有效三维数据，不含原图。
- fetch_peer_features：显式请求另一个 ROS 服务并缓存，不自动匹配。
- query_candidates：对已输入 summaries 的全局向量作 CPU Top-K。
- verify_loop：只使用已经缓存的本机/对端特征，进行匹配与 PnP。

没有自动服务编排。MobileNetVLAD 尚未接入；缺少全局描述子时查询明确失败。
当前同 master 服务供开发验证，网络/跨 master 桥接未实现。

## 几何约定

body_T_camera 将相机点映射到机体。
用本机相机系三维点和对端归一化二维点进行 PnP，K=I。
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
缓存对帧键保持不可变；更换描述子模型应重启并使用新 session/重放，而不是静默覆盖。
当前缓存保留在 RAM，尚无长期存储压缩。

描述子计算、特征请求、验证是分开的成本。对端数据缺失或模型不兼容会返回明确状态。
LocalFeaturePacket 仍复用带三维占位字段的 TrackedFeature，不代表最终紧凑网络编码。
