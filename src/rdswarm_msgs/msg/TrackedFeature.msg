# VIO 轨迹使用低位 ID；最高位为 1 的 ID 仅标识当前 FrameKey 内追加的二维检测点。 (Ref: 2026-10-25_Extend_Peer_2D_Features.md)
uint64 feature_id
geometry_msgs/Point32 pixel
geometry_msgs/Point32 normalized
# 三维点位于该关键帧相机坐标系，不能按世界系解释。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
geometry_msgs/Point position_camera
bool position_valid
float64 depth_variance
bool depth_variance_valid
float32[] descriptor
bool descriptor_valid
