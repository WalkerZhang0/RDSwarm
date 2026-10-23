std_msgs/Header header
uint32 agent_id
string session_id
string vio_frame_id
geometry_msgs/Pose body_pose_vio
geometry_msgs/Vector3 velocity_vio
bool initialized
# 未计算的协方差保持无效，不把默认零矩阵当作精确估计。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
bool covariance_valid
float64[36] covariance
string covariance_source
