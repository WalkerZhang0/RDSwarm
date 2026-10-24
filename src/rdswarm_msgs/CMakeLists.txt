cmake_minimum_required(VERSION 3.10)
project(rdswarm_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs)
add_message_files(FILES FrameKey.msg TrackedFeature.msg Keyframe.msg VioState.msg
  KeyframeSummary.msg LoopCandidate.msg LoopConstraint.msg OdometryConstraint.msg BackendStatus.msg
  LocalFeaturePacket.msg FeatureMatch.msg VerifiedMatches.msg)
add_service_files(FILES GetKeyframe.srv QueryRelativeState.srv DescribeKeyframe.srv GetLocalFeatures.srv
  FetchPeerFeatures.srv QueryLoopCandidates.srv VerifyLoop.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs)
