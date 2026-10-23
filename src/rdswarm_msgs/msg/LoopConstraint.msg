std_msgs/Header header
string observation_id
FrameKey key_a
FrameKey key_b
time stamp_a
time stamp_b
# relative_pose is T_Ba_Bb; covariance is row-major GTSAM tangent order.
# tangent order: rx ry rz tx ty tz, radians and meters.
geometry_msgs/Pose relative_pose
float64[36] covariance
bool covariance_valid
string covariance_source
uint32 inlier_count
float64 reprojection_error
string[] source_observation_ids
