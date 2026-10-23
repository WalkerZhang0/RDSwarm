#pragma once
#include <rdswarm_core/frame_key.hpp>
#include <rdswarm_msgs/Keyframe.h>
#include <rdswarm_msgs/LoopCandidate.h>
#include <rdswarm_msgs/LoopConstraint.h>
#include <string>
#include <vector>

namespace rdswarm {
class LoopDetectorInterface {
 public:
  virtual ~LoopDetectorInterface() = default;
  virtual bool insert(const rdswarm_msgs::Keyframe&, std::string& reason) = 0;
  virtual std::vector<rdswarm_msgs::LoopCandidate> retrieve(
      const FrameKey& query, std::size_t max_candidates) = 0;
  virtual bool verify(const rdswarm_msgs::LoopCandidate&,
                      rdswarm_msgs::LoopConstraint&, std::string& reason) = 0;
};
}  // namespace rdswarm
