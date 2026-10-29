# 邻机二维候选：保留原点，再追加 SuperPoint

## 当前数据路径

```text
VINS 原生关键帧
  ├─ 全部有效二维轨迹：保留 feature_id、像素、归一化二维坐标
  └─ 其中成熟且深度有效者：附带 position_valid=true 的相机系三维点
                ↓ describe_keyframe（用户显式调用）
原点采样 SuperPoint 描述子＋追加去重 SP 检测点
                ↓ get_local_features / fetch_peer_features
本机：仅有效三维点参与 PnP
邻机：所有有效描述子的二维候选，发送时不要求深度
```

VIO 估计器本身的跟踪与优化没有改变；改动发生在观测导出及描述子节点。之前导出时排除了未三角化的轨迹，现在保留当前关键帧上的有效二维观测，三维无效者明确标记 `position_valid=false`。

描述子节点先为全部原点采样，再检测 SuperPoint 新点。原点 ID、像素、归一化坐标、已有三维及有效性均保留；只过滤距原点或已追加点不超过去重半径的新点。默认半径为 4 像素，不使用新检测点替换旧点。

新增点使用 `feature_id >= 2^63`，仅在当前 FrameKey 内有效，没有跨帧轨迹身份。它们的 `position_valid/depth_variance_valid` 始终为 false，不继承附近原点深度。未来共视重投影仍需另外建立轨迹/地标关联。

## 启动与调参

CPU 版本已重新编译到 `devel_cpu`。继续按 [S3E 手动流程](S3E_INTER_AGENT_LOOP.md) 独立运行 VINS、描述节点、loop core 和 bag；各步骤不会自动编排。

描述节点入口保持不变：

```bash
roslaunch rdswarm_bringup describe_agent.launch agent_id:=0
```

```bash
roslaunch rdswarm_bringup describe_agent.launch agent_id:=1
```

这两个 S3E 默认入口分别使用 `results/alpha_cam0.yaml` 和 `results/carol_cam0.yaml`。其他相机必须显式传 `camera_calib_file`；归一化使用同一 VINS 相机模型，不通过猜测焦距生成。标定尺寸和既有 VIO 归一化观测不匹配时返回明确错误。

可显式调整：

```bash
roslaunch rdswarm_bringup describe_agent.launch agent_id:=1 \
  camera_calib_file:="$PWD/results/carol_cam0.yaml" \
  append_image_features:=true \
  image_feature_max_points:=1024 \
  image_feature_score_threshold:=0.02 \
  image_feature_min_distance:=8 \
  image_feature_border:=8 \
  dedup_radius_px:=4.0
```

`image_feature_max_points` 是本次检测的上限，原点数量不占该上限，追加量会因去重减少。`auto_describe` 默认仍为 false，需手动调用 `describe_keyframe`。缓存后修改检测参数不会更新同一帧；重启描述及 loop 节点，换 session/重放后再比较。

关闭追加可用 `append_image_features:=false`。这只关闭 SP 新点；新 VIO 原生导出仍会包含未三角化二维轨迹，所以不自动等于旧版仅成熟轨迹的历史基线。

`get_local_features(include_3d=false)` 和 `fetch_peer_features(include_3d=false)` 返回全体二维候选，清除三维有效性，不传原图。`verify_loop` 仍从 A 选择有效三维点，对 B 不要求三维。匹配与 PnP 门限没有修改，未知协方差仍为无效。

## 原关键帧的固定输入对照

用前轮保存的相同原始关键帧，只改变“保留 B 原点并追加去重 SP”这一项，逐字段核对 A 的有效三维集及原始描述子不变。旧 bag 本身没有导出未三角化轨迹，因此下表只验证 SP 追加的效果；未三角化点另由原生导出测试验证。

| 帧对 | B 原点 → 总候选 | 匹配数：原 → 新 | PnP 内点：原 → 新 | 新结果 |
| --- | ---: | ---: | ---: | --- |
| 0 | 101 → 1080 | 14 → 8 | 11 → 7 | 内点不足，拒绝 |
| 1 | 122 → 1103 | 16 → 14 | 11 → 7 | 内点不足，拒绝 |
| 2 | 96 → 992 | 21 → 24 | 5 → 15 | 通过 |

新候选集合包含旧候选不代表匹配集合一定包含旧匹配。ratio 的次近邻和双向互检都会随候选改变，新增近似描述子可能使原匹配不再满足判据。这是本次保留原点后仍有两组退化的原因；结果不支持“增加 B 点数就一定改善质量”。没有为通过测试降低门限或隐藏失败帧对。

逐个核对旧匹配后，帧对 0 的 14 个旧匹配有 10 个被新的 ratio 检查淘汰，帧对 1 的 16 个旧匹配有 8 个被淘汰；旧点坐标和描述子都没有丢失。这把当前退化明确定位到候选扩展后的匹配判据，而非原点被覆盖。

本机三维点数量和远点比例仍是另一项限制。补点只修正 B 二维候选的来源，不证明深度或回环位姿精度已正确。需要进一步比较匹配策略时，应保持当前数据接口和三维来源固定。

## 在线服务及导出验证

真实记录帧的 ROS 服务链已通过：

- Alpha 原有 105 个有效三维点保持不变；追加点不会进入本机三维匹配集。
- Carol 从 96 个原点扩到 992 个二维候选，Alpha 的 `fetch_peer_features` 成功取回同样数量。
- 对端载荷没有原图，有效三维点为 0；随后 PnP 得到 24 个匹配、15 个内点，与离线核心一致。
- `publish_constraint=false` 时未发布后端位姿约束；协方差仍为无效。

ROS 原生导出夹具验证：成熟三维点保持原坐标；未三角化、失败深度、无效深度及当前帧新轨迹均保留二维，无伪造深度；精确纳秒时间、缓存及 reset epoch 正常。

新增单元检查覆盖原点保留、未成熟二维点、原点/新点像素去重、帧内独立 ID、标定归一化、确定性重复计算、关闭追加、无三维对端包、网格覆盖以及错误尺寸/内参拒绝。当前 CPU 构建的四项 CTest 均已通过。
关闭 SP 追加后的既有合成服务回归也通过：两端各 42 个描述子、42 个 PnP 内点，显式调用与无原图/对端深度的约定保持有效。

结果位置：

- `results/peer_2d_union/report.json`：三组固定输入对照、原匹配变化、数据来源核验。
- [新帧对 2 的 PnP 内点图](../results/peer_2d_union/pair_2_pnp_inliers.jpg)。
- `logs/peer_2d_services/summary.json`：真实记录帧的在线服务回归。
- `logs/interface_smoke_devel_cpu/summary.json`：原生 VIO 导出/缓存测试。

可选回归脚本 `tests/peer_2d_services_test.py` 会自行启动隔离 master，仅供开发验证；正式运行仍使用上述独立终端。回归使用记录关键帧，不包含实时 VINS 运行、无线传输或 GT 精度评估。

本次 Carol 的特征包序列化约 1.09 MB。扩大候选会增加载荷，尚未做量化/压缩；这属于后续调度与传输策略需要衡量的成本。
