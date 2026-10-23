#pragma once
#include <gtsam/geometry/Pose3.h>
#include <rdswarm_core/frame_key.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace rdswarm {
struct PoseInput {
  FrameKey key;
  std::uint64_t capture_time_ns = 0;
  std::string vio_frame_id;
  gtsam::Pose3 body_pose_vio;
};
struct PoseConstraint {
  std::string observation_id;
  FrameKey a, b;
  gtsam::Pose3 a_T_b;
  gtsam::Matrix66 covariance = gtsam::Matrix66::Zero();
  bool covariance_valid = false;
  std::string covariance_source;
};
struct RelativeState {
  bool valid = false;
  std::string status = "not_computed";
  std::uint64_t snapshot_version = 0;
  gtsam::Pose3 a_T_b;
  gtsam::Matrix joint_covariance;
  gtsam::Matrix66 relative_covariance = gtsam::Matrix66::Zero();
};
bool validateConstraint(const PoseConstraint& input, std::string& reason);

// 此处只定义后端边界，不返回虚假的优化结果或有效零协方差。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
class BackendInterface {
 public:
  virtual ~BackendInterface() = default;
  virtual bool insertKeyframe(const PoseInput&, std::string& reason) = 0;
  virtual bool insertOdometry(const PoseConstraint&, std::string& reason) = 0;
  virtual bool insertLoop(const PoseConstraint&, std::string& reason) = 0;
  virtual bool update(std::string& reason) = 0;
  virtual RelativeState relativeState(const FrameKey&, const FrameKey&) const = 0;
};
}  // namespace rdswarm
