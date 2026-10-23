#pragma once
#include <cstdint>
#include <string>
#include <tuple>

namespace rdswarm {
struct FrameKey {
  std::uint32_t agent_id = 0;
  std::string session_id;
  std::uint64_t frame_id = 0;
  bool operator<(const FrameKey& other) const {
    return std::tie(agent_id, session_id, frame_id) <
           std::tie(other.agent_id, other.session_id, other.frame_id);
  }
  bool operator==(const FrameKey& other) const {
    return agent_id == other.agent_id && session_id == other.session_id &&
           frame_id == other.frame_id;
  }
  bool valid() const { return !session_id.empty(); }
};
}  // namespace rdswarm
