#pragma once
#include <rdswarm_core/frame_key.hpp>
#include <rdswarm_msgs/Keyframe.h>
#include <map>
#include <mutex>
#include <string>
#include <deque>

namespace rdswarm {
FrameKey frameKey(const rdswarm_msgs::FrameKey& key);
bool validateKeyframe(const rdswarm_msgs::Keyframe&, std::string& reason);
class KeyframeStore {
 public:
  void configureArchive(const std::string& directory, std::size_t memory_frames);
  bool insert(const rdswarm_msgs::Keyframe&, std::string& reason);
  bool get(const FrameKey&, rdswarm_msgs::Keyframe&) const;
  std::size_t size() const;
 private:
  mutable std::mutex mutex_;
  std::map<FrameKey, rdswarm_msgs::Keyframe> frames_;
  std::deque<FrameKey> insertion_order_;
  std::string archive_directory_;
  std::size_t memory_frames_ = 0;
};
}  // namespace rdswarm
