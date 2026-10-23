# 第一阶段接口

## 已实现的数据链路

VINS-Fusion 的滑窗关键帧直接导出为 rdswarm_msgs/Keyframe，同时保留原始 ROS 输出。
由原始图像缓存恢复整数采样时间戳；缓存缺失时不伪造“精确时间”关键帧。
关键帧含原生 feature_id、当前相机系三维点、对应二维像素/归一化坐标、
机体 VIO 位姿、body_T_camera、session 和可选单目图像。

每机 ROS namespace 独立。例如 agent0/vins/keyframes 供本机后端及回环输入缓存消费。
rdswarm_loop_input_node 提供 get_keyframe，可请求去掉图像/描述子的轻量响应。
检索、匹配、PnP 和后端求解尚未在本阶段实现，不产生伪造回环或位姿修正。

## 后端边界

BackendInterface 输入独立的 PoseInput 和 PoseConstraint；Pose3 和约束噪声都是六维。
状态定义 X = T_W_B；相对观测 Z_ab = T_Ba_Bb。
GTSAM 局部协方差顺序为 rx ry rz tx ty tz，旋转为弧度，平移为米。
联合协方差顺序为 pose_a(6), pose_b(6)，消息矩阵按行存储。

VIO 输出不提供已经校准的增量噪声；covariance_valid=false 时禁止当成精确信息加入。
接口验证器拒绝缺失、非有限、非对称或非正定的有效协方差。
真正实现 iSAM2 更新时，还需要显式处理跨机初始化、gauge、去重和迟到因子。

QueryRelativeState 是接口定义，不提供默认零协方差响应。当前没有后端服务节点。

## 描述子边界

导入的 SuperPoint 前端仅替换新点检测，KLT 跟踪和 VIO 深度估计保持原流程。
该导入版本尚不导出局部或全局描述子；所有 descriptor_valid 都明确为 false。
后续需要在关键帧原始图像上为同一 feature_id 的准确像素位置采样描述子，
并提供全局检索描述子。不能把另一遍检测的点按数组顺序拼接到 VIO 三维点上。

## 身份、时间和坐标

FrameKey = agent_id + session_id + frame_id。
本适配器 frame_id 使用原始图像采样时间的纳秒整数，同一 session 内重复时间不重复发布。
VIO reset 创建新的 session epoch，旧帧 ID 不会与重置后的关键帧冲突。
UTC 同步由输入保证；不引入 UWB/LPS 时间转换，不使用接收时刻替代采样时刻。

图像仍在各自相机坐标系，三维点 position_camera 位于当前关键帧相机系。
body_pose_vio 位于该机自己的局部 VIO 坐标系，不是已跨机对齐的世界位姿。
body_T_camera 将相机点变换到机体；后端尚未做任何地图对齐或反馈。
