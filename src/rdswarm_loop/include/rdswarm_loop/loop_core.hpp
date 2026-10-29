#pragma once
#include <rdswarm_loop/keyframe_store.hpp>
#include <rdswarm_msgs/LocalFeaturePacket.h>
#include <rdswarm_msgs/KeyframeSummary.h>
#include <rdswarm_msgs/LoopCandidate.h>
#include <rdswarm_msgs/LoopConstraint.h>
#include <rdswarm_msgs/VerifiedMatches.h>
#include <map>
#include <string>
#include <vector>
namespace rdswarm {
struct ExternalMatch {
  std::size_t local_feature_index = 0;
  std::size_t remote_feature_index = 0;
  float confidence = 0.0f;
};
struct LoopOptions {
  double ratio_threshold=0.8;
  double max_descriptor_distance=0.9;
  int min_matches=8,min_inliers=8,ransac_iterations=200;
  double min_inlier_ratio=0.5,pnp_threshold_normalized=0.004;
  double min_second_singular_ratio=1e-3,min_positive_depth_fraction=0.9;
  int ransac_seed=17;
  bool use_nominal_covariance=false;
  double nominal_rotation_std=0.03,nominal_translation_std=0.10;
};
struct LoopDiagnostics {
  std::string stage,status;
  std::string matcher = "bf_ratio_mutual";
  std::size_t local_features=0,remote_features=0,matches=0,inliers=0;
  double rms_normalized=0.0,matching_ms=0.0,geometry_ms=0.0,total_ms=0.0;
};
bool validatePacket(const rdswarm_msgs::LocalFeaturePacket&,std::string& reason);
rdswarm_msgs::LocalFeaturePacket localPacket(const rdswarm_msgs::Keyframe&,bool include_3d);
bool verifyLoop(const rdswarm_msgs::LocalFeaturePacket& local,
    const rdswarm_msgs::LocalFeaturePacket& remote,const LoopOptions&,
    rdswarm_msgs::LoopConstraint&,rdswarm_msgs::VerifiedMatches&,LoopDiagnostics&);
bool verifyLoopWithMatches(const rdswarm_msgs::LocalFeaturePacket& local,
    const rdswarm_msgs::LocalFeaturePacket& remote,
    const std::vector<ExternalMatch>& matches, const LoopOptions&,
    rdswarm_msgs::LoopConstraint&,rdswarm_msgs::VerifiedMatches&,LoopDiagnostics&);
class GlobalRetrievalIndex {
 public:
  bool insert(const rdswarm_msgs::KeyframeSummary&,std::string& reason);
  bool query(const FrameKey&,std::size_t count,bool inter_agent_only,double min_dt,
      std::vector<rdswarm_msgs::LoopCandidate>&,std::string& reason) const;
 private:
  std::map<FrameKey,rdswarm_msgs::KeyframeSummary> summaries_;
};
std::string identityText(const FrameKey&);
}  // namespace rdswarm
