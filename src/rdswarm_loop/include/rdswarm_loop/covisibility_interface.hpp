#pragma once
#include <rdswarm_msgs/VerifiedMatches.h>
#include <rdswarm_msgs/LocalFeaturePacket.h>
#include <string>
namespace rdswarm {
// 三维点是 VIO 条件估计；新增二维点没有跨帧地标身份，后续融合需补齐轨迹来源与噪声。 (Ref: 2026-10-28_Add_Global_Retrieval.md)
struct CoVisibilityBundle {
  rdswarm_msgs::LocalFeaturePacket local, remote;
  rdswarm_msgs::VerifiedMatches associations;
  std::string source_observation_id;
  std::string geometry_source = "vio_conditioned_3d_not_ground_truth";
};
// 这里只预留已验证关联的入口，不构建或加入机间重投影残差。 (Ref: 2026-10-24_Portable_Loop_Core.md)
class CoVisibilityObservationSink {
 public:
  virtual ~CoVisibilityObservationSink()=default;
  virtual bool stageVerifiedCorrespondences(const rdswarm_msgs::VerifiedMatches&,
      std::string& reason)=0;
};
class CoVisibilityBundleSink {
 public:
  virtual ~CoVisibilityBundleSink()=default;
  virtual bool stageBundle(const CoVisibilityBundle&,std::string& reason)=0;
};
}
