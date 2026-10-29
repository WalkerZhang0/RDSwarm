# S3E：手动验证 SuperPoint＋PnP 机间约束

匹配点少的后续定位及两张原图独立输入方法见 [两图诊断](TWO_IMAGE_DIAGNOSIS.md)。
后续已改为保留原点并追加邻机二维候选，且完成真实记录帧的在线请求/PnP 回归，见 [当前二维候选方案](PEER_2D_FEATURES.md)。下文五组结果保留为修改前的历史基线。

## 本轮已验证的范围

两路单目 VINS 导出真实关键帧，在各自已有 VIO 特征位置采样 256 维 SuperPoint 描述子，再用 Alpha 的相机系三维点和 Carol 的归一化二维点进行 PnP。VIO 保持 Shi-Tomasi＋KLT，SuperPoint 此处用于描述，不替换 VIO 跟踪。

实际双路节点已在独立本机 ROS master 下启动，回放开头 40 秒，保存 Alpha 45 个、Carol 67 个原生关键帧。两机本轮使用 `acc_n=0.1` 的实验配置，其余三个 IMU 噪声保留原值；Carol 仅做这段几何测试，未声称完成整段精度验证。

在线服务调用随后因自动审批超时、重试返回 503 而未执行。因此，本轮使用已保存的同一批关键帧，调用与在线节点相同的 C++ SuperPoint 推理、描述子采样及 `verifyLoop` 核心完成离线几何验证。**离线结果不能代替真实帧的在线特征请求验收。** 先前合成数据服务测试与本次真实数据几何结果分别记录。

5 组人工挑选的帧对中 2 组通过原有门限，3 组被拒绝：

| 帧对 | Alpha 帧 ID | Carol 帧 ID | 匹配数 | PnP 内点 | 归一化重投影 RMS | 结果 |
| --- | --- | --- | ---: | ---: | ---: | --- |
| 0 | `1660900402165000000` | `1660900402127000000` | 14 | 11 | 0.002118 | 通过 |
| 1 | `1660900402165000000` | `1660900408127000000` | 16 | 11 | 0.002315 | 通过 |
| 2 | `1660900407165000000` | `1660900412927000000` | 21 | 5 | — | 内点不足 |
| 3 | `1660900412165000000` | `1660900412327000000` | 8 | 0 | — | 内点不足 |
| 4 | `1660900417565000000` | `1660900417327000000` | 7 | 0 | — | 匹配不足 |

没有降低默认匹配或几何门限。两组成功结果的协方差均保持无效。5 组人工选择不能用于计算自动检索召回率。

帧对 1 相差约 5.962 秒，恢复的两帧相对平移长度约 1.175 m。对原始 GT 的各自时刻位置插值后，间距约 1.001 m，只用于量级核对；未校准 GT 参考点与 IMU 的偏移，也没有可靠 GT 姿态评价。本轮不宣称相对位姿精度已通过完整评测。

### GT 核对结果（2026-10-25）

已检查原始 `alpha_gt.txt` 的全部 400 行与 `carol_gt.txt` 的全部 377 行：两者姿态列均固定为 `[0,0,0,1]`。这些文件不能提供经过确认的姿态真值，不能直接将其拼成完整 SE(3) 真值来计算旋转和方向误差。

以下只比较回环平移长度与 GT 两点间距，分别在 Alpha 的 `t_a`、Carol 的 `t_b` 插值，无缩放、无轨迹对齐、无外推：

| 帧对 | `t_b-t_a` | 估计平移长度 | GT 位置间距 | 长度绝对差 | 相对差 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | −0.038 s | 6.9379 m | 7.1829 m | 0.2450 m | 3.41% |
| 1 | +5.962 s | 1.1746 m | 1.0007 m | 0.1739 m | 17.37% |

指标为 `abs(norm(t_est) - norm(p_gt_b(t_b)-p_gt_a(t_a)))`。两个匹配所需的 GT 插值区间均约 1 秒，没有跨越文件中的长缺口。精确输入帧号、插值端点、数据文件 SHA256 和指标定义保存于 [gt_position_check.json](../results/s3e_loop_validation/gt_position_check.json)，另以 NumPy 插值独立复核，数值差异小于 1 微米。

这些长度差不能称作完整平移向量误差：方向错误可能在长度指标下被隐藏。GT 位置参考点到 IMU 的杆臂也未确认，因此差异还含插值及参考点不一致的影响。PnP 的小重投影误差只说明对输入地图点与像素的拟合，不能替代真值验证。目前没有足够证据将这两条约束认定为已通过完整精度评测。

严格评价需两端真实姿态及 GT 参考点到 body 的外参，用各自时刻的 `T_W_Ba(t_a)`、`T_W_Bb(t_b)` 构造 `T_Ba_Bb_gt = inverse(T_W_Ba(t_a))*T_W_Bb(t_b)`，再评价相对变换误差的平移和旋转；不能把全为单位四元数的列当作真实姿态，也不能用受评估 VIO 的姿态冒充独立 GT。

匹配可视化：

- [相近时刻帧对](../results/s3e_loop_validation/pair_0_inliers.jpg)
- [相差约 6 秒帧对](../results/s3e_loop_validation/pair_1_inliers.jpg)

这些文件为本地结果，不上传仓库。匹配点主要位于远处树木及地面边缘，当前点数与几何覆盖仍有限。

## 逐个终端启动

下面各节点保持独立，bag 不自动播放。每个终端先加载同一环境；以下是 zsh：

```bash
cd "/media/walker/My PSSD/tmp/论文/RDSwarm"
source /opt/ros/noetic/setup.zsh
source devel_cpu/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
```

模型已在本机 `models/superpoint_dense.onnx`，身份来自同目录 JSON。模型文件不随 Git 分发；其他机器按 [LOOP_WORKFLOW.md](LOOP_WORKFLOW.md) 准备同一模型。

终端 1：端口没有 master 时启动。

```bash
roscore -p 11311
```

终端 2：启用仿真时间并启动 Alpha。关闭旧缓存，关键帧由下面的描述子节点缓存。

```bash
rosparam set /use_sim_time true
rosparam set /agent0/vins/frontend_frame_stride 2
roslaunch rdswarm_bringup s3e_alpha.launch \
  config_file:="$PWD/results/alpha_vins_debug.yaml" \
  session_id:=s3e_loop01 cache_keyframes:=false \
  output_path:="$PWD/results/s3e_loop_manual/alpha"
```

终端 3：启动 Carol。

```bash
rosparam set /agent1/vins/frontend_frame_stride 2
roslaunch rdswarm_bringup s3e_carol.launch \
  config_file:="$PWD/results/carol_vins_debug.yaml" \
  session_id:=s3e_loop01 cache_keyframes:=false \
  output_path:="$PWD/results/s3e_loop_manual/carol"
```

终端 4、5：分别启动两台的描述子节点，默认只缓存、不计算。

```bash
roslaunch rdswarm_bringup describe_agent.launch agent_id:=0
```

```bash
roslaunch rdswarm_bringup describe_agent.launch agent_id:=1
```

终端 6、7：分别启动两台的回环核心。它们不会自动检索、请求或 PnP。

```bash
roslaunch rdswarm_bringup loop_agent.launch agent_id:=0
```

```bash
roslaunch rdswarm_bringup loop_agent.launch agent_id:=1
```

终端 8：两个 bag 用同一个播放器，所有节点准备好后按空格回放。先只跑前 40 秒，避免原图缓存长时间占用内存。

```bash
rosbag play --clock --pause --duration=40 -r 1.0 \
  "$PWD/results/s3e_library_1_ros1/alpha_mono8.bag" \
  "$PWD/results/s3e_library_1_ros1/carol_mono8.bag"
```

可另开 RViz：`roslaunch rdswarm_bringup s3e_rviz.launch agent_id:=0`。两路轨迹尚未对齐，回环验证也没有加入后端，不应期待此时轨迹自动拼接。

## 显式调用三步

先核对两台 `features` 终端的 `stage=raw_cached` 日志，确认目标帧已被缓存。也可在回放时观察 `/agent0/vins/keyframes/key` 和 `/agent1/vins/keyframes/key`。

下面使用本次离线帧对 1 的固定 ID；本次 session 为 `s3e_loop01.e0`。VINS 重跑可能因关键帧选择产生不同输出，必须以实际消息为准，不能假定固定帧 ID 必然再次出现。返回 `raw_keyframe_not_found` 时选取附近已缓存帧，或用下文保存的原生关键帧做确定性复现。

终端 9：A，为两帧分别计算并发布描述子。

```bash
rosservice call /agent0/describe_keyframe \
  "{key: {agent_id: 0, session_id: s3e_loop01.e0, frame_id: 1660900402165000000}, publish_result: true}"
```

```bash
rosservice call /agent1/describe_keyframe \
  "{key: {agent_id: 1, session_id: s3e_loop01.e0, frame_id: 1660900408127000000}, publish_result: true}"
```

预期均返回 `success: true`、相同模型标识和有效描述子数量。此时 loop core 才会收到具备描述子的关键帧。

B，从 Alpha 回环模块请求 Carol 局部特征。

```bash
rosservice call /agent0/fetch_peer_features \
  "{key: {agent_id: 1, session_id: s3e_loop01.e0, frame_id: 1660900408127000000}, service_name: /agent1/get_local_features, include_3d: false}"
```

只有这里实际返回 `success: true`，才算这对真实帧的在线请求通过。请求内容含局部描述子、特征 ID、二维观测和外参，不含原图，对端三维点标记为无效。

C，执行匹配和 PnP。

```bash
rosservice call /agent0/verify_loop \
  "{local_key: {agent_id: 0, session_id: s3e_loop01.e0, frame_id: 1660900402165000000}, remote_key: {agent_id: 1, session_id: s3e_loop01.e0, frame_id: 1660900408127000000}, publish_constraint: false}"
```

预期成功状态为 `verified:ok`，同时检查内点数和 `reprojection_error_unit: normalized_camera`。默认 `covariance_valid: false`。参数仍由 `src/rdswarm_loop/config/loop.yaml` 提供，可用 `rosparam get /agent0/loop_core` 查看。

输出为 `T_Ba_Bb`，绑定两帧自己的采样时间。异时帧约束不能直接解释成同一时刻两机相对位姿。`publish_constraint:false` 不发布位姿约束；成功时仍发布 `verified_matches` 供关联检查，尚无后端优化接收。

## 不重跑 VINS 的确定性复现

原生关键帧已保存到 `results/s3e_loop_validation/keyframes.bag`。仅启动 master、两台描述子节点和 loop core，再手动播放：

```bash
rosbag play --clock --pause -r 1.0 \
  "$PWD/results/s3e_loop_validation/keyframes.bag"
```

不要同时运行 VINS 或传感器 bag 播放器。这份 bag 保留本次关键帧 ID、二维/三维和原始图像，在线节点不需重建深度。随后按上面的 A/B/C 调用；这验证记录关键帧到服务的链路，不包含实时 VINS。

## 离线几何检查入口

`loop_pair_probe` 不启动 ROS 节点，只消费两个序列化 `LocalFeaturePacket`，调用在线服务相同的 C++ 核心。输入是本机 VIO 三维＋描述子和对端二维＋描述子，所有门限保持 `LoopOptions` 默认值。

```bash
devel_cpu/lib/rdswarm_loop/loop_pair_probe \
  "$PWD/results/s3e_loop_validation/offline/0_1660900402165000000_local.bin" \
  "$PWD/results/s3e_loop_validation/offline/1_1660900408127000000_peer.bin" \
  "$PWD/results/s3e_loop_validation/pair_manual.json"
```

输出包含内点、重投影 RMS、相对位姿和特征 ID。退出码 0 为通过、1 为几何拒绝、2 为输入/文件错误。本次还验证了模型不一致和截断载荷会被拒绝。

完整离线结果见 `results/s3e_loop_validation/offline_results.json`。本机输出为离线验证产物，不把它们当作 MobileNetVLAD 检索、通信性能或回环召回率的测试结果。
