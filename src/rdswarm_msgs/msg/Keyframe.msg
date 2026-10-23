std_msgs/Header header
FrameKey key
string vio_frame_id
geometry_msgs/Pose body_pose_vio
# body_T_camera maps camera coordinates into body coordinates.
geometry_msgs/Transform body_T_camera
geometry_msgs/Vector3 velocity_vio
bool initialized
bool capture_stamp_exact
TrackedFeature[] features
string local_descriptor_model
bool image_included
sensor_msgs/Image image
