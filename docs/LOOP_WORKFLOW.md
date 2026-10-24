# 回环工作流程：逐项运行与调参

日期：2026-10-24。本文是当前操作入口。PHASE1.md/VALIDATION.md 保留为上一阶段历史记录。

## 1. 当前做到了哪里

本次实现：可选 TensorRT、CPU SuperPoint 检测/描述子采样、关键帧描述子导出、
CPU 全局向量索引、局部匹配、PnP 与几何验收。

本次未实现：MobileNetVLAD 推理、自动调度、iSAM2 求解、机间重投影残差、
跨 ROS master 的通信桥。全局索引能够消费未来 MobileNetVLAD 的描述子，
但不会用空向量或临时直方图假装已经接入该网络。

算法和运行控制分开：
- rdswarm_features：OpenCV DNN CPU 推理与指定像素采样，无 CUDA/TensorRT/YOLO。
- VINS：原来的跟踪/深度估计；TensorRT 检测器现在是可选库。
- rdswarm_feature_node：缓存 VIO 原始关键帧，显式计算对应像素的描述子。
- rdswarm_loop_node：缓存就绪特征，分别提供查询、对端数据请求、几何验证服务。
- CPU 核心仍使用 ROS 生成消息作为数据结构，但不执行订阅、服务或网络副作用。

## 2. 先选择构建和前端模式

默认构建不查找 CUDA/TensorRT，也不依赖 Torch、YOLO 或 ONNX Runtime C++ SDK：

~~~bash
cd "/media/walker/My PSSD/tmp/论文/RDSwarm"
source /opt/ros/noetic/setup.bash
catkin_make --build build_cpu -j2 -l2 \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel_cpu" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DRDSWARM_WITH_TENSORRT=OFF
source devel_cpu/setup.bash
~~~

本机 cv_bridge 使用系统 OpenCV 4.2；其他机器应选择与其 cv_bridge 相容的 OpenCV_DIR。
不要同时 source 旧工程的同名 vins/camera_models 包。

三种前端模式是明确不同的实验设置，不自动互相回退：

| 模式 | 编译选项 | VINS 启动参数 |
|---|---|---|
| KLT＋Shi-Tomasi 检测 | OFF 即可 | use_superpoint:=0；默认，无模型也可启动 |
| KLT＋CPU SuperPoint 检测 | OFF 即可 | use_superpoint:=1 superpoint_backend:=opencv superpoint_model:=绝对路径 |
| KLT＋GPU SuperPoint 检测 | ON，需 TensorRT/CUDA | use_superpoint:=1 superpoint_backend:=tensorrt superpoint_engine:=绝对路径 |

无论 VIO 使用哪种点检测器，描述子节点都在这些**已有 VIO 轨迹的像素位置**
采样 SuperPoint 描述子。它不会将新检测点按数组序号拼给旧三维点。
同一资源对照实验应固定 VIO 模式，不把更换检测器带来的差异混入调度收益。

可选 GPU 构建示例：

~~~bash
catkin_make --build build_gpu -j2 -l2 \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel_gpu" \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DRDSWARM_WITH_TENSORRT=ON \
  -DTENSORRT_ROOT=/usr/local/TensorRT-10.8.0.43 \
  -DCUDA_ROOT=/usr/local/cuda
~~~

TensorRT 插件启用后仍可选择 opencv 后端。插件不包含 YOLO。
当前插件使用 TensorRT 10 API；Jetson 的 SDK 和 engine 兼容性需另行验证。

## 3. 显式准备描述子模型

已有用户 ONNX 输出经过 Top-K/NMS 后的离散点描述子，
不能直接拿它们对应任意 KLT 像素。本项目的小工具只截取两个原始卷积头，
保留权重，由 CPU 端按准确像素采样：

~~~bash
python3 scripts/prepare_superpoint_dense.py \
  /absolute/path/to/superpoint_1024.onnx \
  "$PWD/models/superpoint_dense.onnx"
~~~

这是单独的模型格式准备步骤，不启动 VIO、ROS、回环或后端。
需要 Python onnx 包；正式运行节点只依赖 OpenCV。
输出 onnx 和 JSON 记录来源/模型 SHA256 与 model_id。模型文件不提交 Git。
不同网络头名可显式使用 --detector-head 和 --descriptor-head，不能靠猜测替换。

固定约定：灰度 0–1，右/下零填充到 8 的倍数，原像素不缩放；
256 维密集图逐点归一化，双线性采样采用原模型半像素/align_corners 约定，再 L2 归一化。
双方必须使用相同权重、采样约定和 model_id。

## 4. 各终端分别做什么

每个终端都 source /opt/ros/noetic/setup.bash 和同一 devel_cpu/setup.bash。
开发调试先让两节点使用同一个 ROS master，明确消息接口；这不表示存在集中式优化器。

### 终端 1：ROS master

~~~bash
roscore
~~~

### 每台设备的 VIO 终端

~~~bash
roslaunch rdswarm_bringup mono_agent.launch \
  agent_id:=0 session_id:=run01 cache_keyframes:=false \
  use_superpoint:=0 \
  image_topic:=/drone0/cam0/image_raw imu_topic:=/drone0/imu0
~~~

agent1 使用 agent_id:=1、另一台图像/IMU话题及真实标定 config_file。
默认标定仅为 EuRoC 示例，自采数据必须替换。
cache_keyframes:=false 关闭第一阶段的旧输入缓存节点，避免和新流程重复缓存。

可选 CPU SuperPoint VIO：

~~~bash
roslaunch rdswarm_bringup mono_agent.launch \
  agent_id:=0 session_id:=run01 cache_keyframes:=false \
  use_superpoint:=1 superpoint_backend:=opencv \
  superpoint_model:="$PWD/models/superpoint_dense.onnx"
~~~

### 每台设备的描述子终端

~~~bash
rosrun rdswarm_loop rdswarm_feature_node \
  __ns:=/agent0 __name:=features \
  keyframes_in:=vins/keyframes \
  _model_path:="$PWD/models/superpoint_dense.onnx" \
  _auto_describe:=false
~~~

agent1 将 namespace 换为 /agent1。
该节点默认只缓存原始关键帧，**不会自动运行描述子推理**。
model_id 默认读取模型旁的 JSON；也可显式设置 _model_id。

### 每台设备的回环核心终端

~~~bash
rosparam load src/rdswarm_loop/config/loop.yaml /agent0/loop_core
rosrun rdswarm_loop rdswarm_loop_node __ns:=/agent0 __name:=loop_core
~~~

agent1 换为对应 namespace。这个节点不自动检索、不自动请求对端、不自动 PnP。
终端日志显示 stage、状态、匹配数、内点数、归一化重投影误差和耗时。

## 5. 从一帧到回环的显式动作

先观察帧键：

~~~bash
rostopic echo -n 1 /agent0/vins/keyframes/key
rostopic echo -n 1 /agent1/vins/keyframes/key
~~~

下面使用接口测试的固定键；真实数据必须替换为刚观察到的 agent/session/frame_id。

### A：为两端选中的关键帧计算描述子

~~~bash
rosservice call /agent0/describe_keyframe \
  "{key: {agent_id: 0, session_id: fixture, frame_id: 11}, publish_result: true}"
rosservice call /agent1/describe_keyframe \
  "{key: {agent_id: 1, session_id: fixture, frame_id: 22}, publish_result: true}"
~~~

发生：本机从原始图像为既有 feature_id 采样描述子，发布 /agentX/keyframes。
不发生：全局检索、通信请求、PnP。
服务响应只包含状态、特征数量和模型标识，便于检查。

### B：向对端请求局部特征

~~~bash
rosservice call /agent0/fetch_peer_features \
  "{key: {agent_id: 1, session_id: fixture, frame_id: 22}, \
service_name: /agent1/get_local_features, include_3d: false}"
~~~

发生：只请求并缓存对端数据。不自动执行匹配和 PnP。
若对端尚未 describe/publish，返回 keyframe_not_found，而不是偷偷计算。

### C：验证已知帧对

~~~bash
rosservice call /agent0/verify_loop \
  "{local_key: {agent_id: 0, session_id: fixture, frame_id: 11}, \
remote_key: {agent_id: 1, session_id: fixture, frame_id: 22}, \
publish_constraint: false}"
~~~

使用本机相机系三维点和对端归一化二维观测：
互检＋ratio 匹配 → PnP-RANSAC → 内点、几何分布、正深度、重投影验收。
PnP 的相机变换经两机外参变换为 T_Ba_Bb。
publish_constraint=false 只返回结果；需要明确发送约束时再设置 true。

/agent0/verified_matches 保留已验证特征 ID 关联，供未来共视逻辑接入。
本次没有构建或加入任何机间重投影因子；该话题不是另一份独立视觉信息。
默认 covariance_valid=false，不能把 PnP 成功等同于协方差已经校准。

### D：有真正全局描述子后再查询 Top-K

rdswarm_loop_node 接收 /agentX/summaries 的 KeyframeSummary。
CPU 索引支持归一化向量 cosine Top-K，并检查模型/维数、排除自身和同机近邻。
当前不自动产生 MobileNetVLAD 描述子；metadata 的 global_descriptor_valid=false，
不会被伪装成可检索的全局向量。

实际发布器尚未接入时，可显式注入测试向量检查索引，不能用它评价检索效果：

~~~bash
rostopic pub -1 /agent0/summaries rdswarm_msgs/KeyframeSummary \
  "{header: {stamp: {secs: 1760000001, nsecs: 123456789}}, \
key: {agent_id: 0, session_id: fixture, frame_id: 11}, \
global_descriptor_valid: true, global_descriptor_model: synthetic_index_test_only, \
global_descriptor: [1.0, 0.0]}"
rostopic pub -1 /agent0/summaries rdswarm_msgs/KeyframeSummary \
  "{header: {stamp: {secs: 1760000002, nsecs: 123456789}}, \
key: {agent_id: 1, session_id: fixture, frame_id: 22}, \
global_descriptor_valid: true, global_descriptor_model: synthetic_index_test_only, \
global_descriptor: [0.9, 0.1]}"
~~~

~~~bash
rosservice call /agent0/query_candidates \
  "{query_key: {agent_id: 0, session_id: fixture, frame_id: 11}, \
max_candidates: 5, inter_agent_only: true, min_time_separation: 5.0}"
~~~

未接入全局模型时返回 query_global_descriptor_missing 是正常且明确的状态。
先用已知帧对测试 B/C；MobileNetVLAD 接入是下一阶段，不把测试向量当真实检索效果。

## 6. 对端到底需要传什么

对回环前端，通常不需要传原图，所需内容分两层：

1. 经常/按策略交换：帧键、时间、全局描述子及其模型标识。
2. 候选确定后按需交换：局部描述子、对应 feature_id、归一化二维点、
   帧时间与相机外参。本机已有米制三维点时，对端可以不提供有效三维点。

LocalFeaturePacket 没有 image、整条 VIO 轨迹或对端全局位姿字段。
当前复用 TrackedFeature，因此 include_3d=false 会清除有效三维数据，
但消息仍包含三维占位字段，尚未做最终二进制载荷压缩。

这是回环前端的通信需求。整个协同后端还需要邻机关键帧/里程计链，
不能只靠描述子重建完整多机状态。这部分不在本次三个步骤中实现。
ROS 服务现在用于同 master 的接口验证，跨 master/WiFi 桥、超时和重试另行接入。

## 7. 哪些参数可以当场改

匹配/PnP 参数在每次 verify_loop 时从 /agentX/loop_core 读取：

~~~bash
rosparam get /agent0/loop_core
rosparam set /agent0/loop_core/matching/ratio_threshold 0.75
rosparam set /agent0/loop_core/geometry/pnp_threshold_normalized 0.003
~~~

| 参数 | 含义 |
|---|---|
| matching/ratio_threshold | 最近与次近描述子距离之比，越小越严格 |
| matching/max_descriptor_distance | L2 归一化描述子的绝对距离门限 |
| geometry/min_matches | 启动几何验证的最少匹配数 |
| geometry/min_inliers、min_inlier_ratio | RANSAC 接受条件 |
| geometry/pnp_threshold_normalized | **归一化坐标单位，不是像素** |
| geometry/min_second_singular_ratio | 拒绝近共线/微小分布；不无条件拒绝所有平面 |
| geometry/min_positive_depth_fraction | 内点中正深度比例 |
| covariance/use_nominal | 默认 false；true 才使用明确配置的名义噪声 |
| covariance/rotation_std、translation_std | 弧度／米，名义标准差，不是校准后的真实误差 |

参数优先级：显式 ROS 私有参数／launch 参数覆盖 VIO YAML；修改 YAML 后不要忘记对应 launch 默认值。
model_path、model_id、auto_describe 和 VIO 检测器是启动参数，修改后应重启对应节点。
更换模型/采样约定时使用新 session 并重放输入，避免将同一帧缓存变成混合模型。
当前缓存保留历史、原始图像占用 RAM；首次验证使用短片段，不声称已解决长期存储。

## 8. 每次修改如何测试和 debug

单元和几何测试无需 ROS master：

~~~bash
devel_cpu/lib/rdswarm_features/superpoint_sampling_test
devel_cpu/lib/rdswarm_loop/loop_core_test
devel_cpu/lib/rdswarm_backend/constraint_contract_test
~~~

实际模型对照需可选 Python onnxruntime（仅测试用，不是节点依赖）：

~~~bash
python3 tests/check_descriptor_parity.py \
  --original-model /absolute/path/to/superpoint_1024.onnx \
  --prepared-model "$PWD/models/superpoint_dense.onnx" \
  --probe "$PWD/devel_cpu/lib/rdswarm_features/superpoint_model_probe"
~~~

显式检查 VIO 检测器选择：

~~~bash
devel_cpu/lib/vins/superpoint_detector_probe \
  opencv models/superpoint_dense.onnx logs/descriptor_parity/image.png
~~~

ROS 接口夹具只发布两个合成帧，不启动整个系统，也不做任何服务调用：

~~~bash
python3 tests/publish_loop_fixture.py
~~~

使用夹具时在独立测试 master/namespace 中启动上述节点，避免混入真实数据。
随后亲自执行 A/B/C，看到每一步输出。夹具为已知相同视图，不是实际回环精度证据。

修改后至少执行：
1. 针对问题输入保留最小复现和失败测试；
2. CPU 构建与相关单元测试；
3. 明确涉及的服务链；
4. 原生 VIO 接口回归（可选自动化测试，不是正式运行入口）：

~~~bash
python3 scripts/smoke_interfaces.py --devel-space devel_cpu
~~~

两机服务链也有可选的独立回归测试，供修改后自动检查，不替代上述手动操作入口：

~~~bash
python3 tests/loop_services_test.py --model models/superpoint_dense.onnx
~~~

如果涉及 GPU 插件，额外检查 ON 构建。日志和模型对照结果留在 logs/；
只有检查通过才提交。不要因为已知帧对 PnP 通过，就宣称真实检索、调度或精度已经验证。
