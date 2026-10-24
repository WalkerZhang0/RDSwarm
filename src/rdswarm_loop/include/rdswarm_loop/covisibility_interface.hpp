#pragma once
#include <rdswarm_msgs/VerifiedMatches.h>
#include <string>
namespace rdswarm {
// 这里只预留已验证关联的入口，不构建或加入机间重投影残差。 (Ref: 2026-10-24_Portable_Loop_Core.md)
class CoVisibilityObservationSink {
 public:
  virtual ~CoVisibilityObservationSink()=default;
  virtual bool stageVerifiedCorrespondences(const rdswarm_msgs::VerifiedMatches&,
      std::string& reason)=0;
};
}
