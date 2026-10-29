# 2026-10-25_Verify_S3E_InterAgent_Loop

## 1. 修改文件清单

- `src/rdswarm_bringup/launch/describe_agent.launch`：独立启动本机关键帧描述节点，默认仅缓存，服务显式触发推理。
- `src/rdswarm_bringup/launch/loop_agent.launch`：独立启动回环核心并加载可调参数，不自动搜索或请求。
- `src/rdswarm_loop/test/pair_probe.cpp`、`CMakeLists.txt`：新增不需 ROS master 的离线帧对探针，调用在线相同的 `verifyLoop` 核心。
- `results/carol_vins_debug.yaml`、`.gitignore`：保留一份使用实验加速度权重的 Carol 配置，原标定配置不覆盖。
- `docs/S3E_INTER_AGENT_LOOP.md`：记录真实帧几何结果、逐个终端运行及已保存关键帧的确定性复现方式。
- `docs/LOOP_WORKFLOW.md`、`README.md`：添加本轮实际数据验证入口。

## 2. 功能实现说明

两个新 launch 分别只负责描述和回环节点，用户继续单独启动 VINS、RViz 和 bag，显式调用 `describe_keyframe/fetch_peer_features/verify_loop`。服务运行方式与算法核心保持分开。

离线探针读取序列化 `LocalFeaturePacket`，不启动网络服务；使用默认匹配及几何门限，结果以 JSON 输出，包含特征 ID 关联、内点、重投影 RMS、位姿及协方差有效性。对端载荷只保留二维与描述子，不含原图或有效三维点。

本轮独立 ROS master 使用 11478 端口，回放 Alpha/Carol 开头 40 秒，生成 45/67 个真实关键帧并保存原生关键帧 bag。两机使用 `acc_n=0.1` 的实验权重，其余三个噪声值、标定和时偏保持原值。

## 3. 问题与解决办法

实际服务调用遇到自动审批超时，按提示重试后返回 503，调用未执行。为继续验证不依赖网络的工作，使用已经保存的真实原生关键帧，调用与在线节点相同的描述子采样和几何核心。真实数据在线特征请求仍未验收，文档明确区分。

五组人工帧对中两组通过默认门限：14/16 个匹配、各 11 个 PnP 内点，归一化重投影 RMS 约 0.002118/0.002315；另三组因内点或匹配不足被拒绝。时间相差约 5.962 秒的帧对恢复相对平移长度约 1.175 m，其 GT 位置间距约 1.001 m，仅为参考点未校准的量级核对。

CPU 构建、现有描述子采样和 PnP 核心测试通过。离线探针的模型不一致和截断数据负例被拒绝，对端载荷无原图、有效三维点为 0。采样和匹配过程中 `feature_id` 与原生像素/三维对应关系保持不变。匹配图和完整结果保存在 `results/s3e_loop_validation/`。临时节点和 master 已正常停止。

## 4. 注意事项 (Precautions)

- 两组通过不代表自动全局检索已实现，也不能给出真实回环召回率；MobileNetVLAD 本轮未接入。
- PnP 返回绑定各自时间的 `T_Ba_Bb`；异时帧结果不能解释为同一时刻两机位姿。
- `covariance_valid=false`，未发布后端位姿约束，也未添加机间重投影因子。
- CPU 描述子探针每帧约 0.55–0.60 秒，包含单独进程及模型载入等开销，不是在线推理基准。
- CPU 原生关键帧保留图像会占用内存，首次只回放 40 秒。新 launch 不自动限制缓存时长，长期存储仍待设计。
- Carol 的实验噪声配置只用于本段几何测试，不代表全段精度或噪声标定已验证。
- `loop_pair_probe` 在 `CATKIN_ENABLE_TESTING` 下构建，输入二进制为本机 ROS1 消息序列化格式，非通用跨版本文件协议。
- 全工程已有其他未提交变更；本轮未覆盖用户修改的 EuRoC 配置。
