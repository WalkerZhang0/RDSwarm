#include <rdswarm_loop/keyframe_store.hpp>
#include <cmath>
#include <set>

namespace rdswarm {
FrameKey frameKey(const rdswarm_msgs::FrameKey& key) {
  FrameKey result;
  result.agent_id = key.agent_id;
  result.session_id = key.session_id;
  result.frame_id = key.frame_id;
  return result;
}
bool validateKeyframe(const rdswarm_msgs::Keyframe& frame, std::string& reason) {
  if (!frameKey(frame.key).valid() || frame.header.stamp.isZero() ||
      frame.vio_frame_id.empty() || !frame.initialized || !frame.capture_stamp_exact) {
    reason = "invalid_frame_metadata"; return false;
  }
  const auto& p = frame.body_pose_vio.position;
  const auto& q = frame.body_pose_vio.orientation;
  const auto& t = frame.body_T_camera.translation;
  const auto& r = frame.body_T_camera.rotation;
  const double nq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  const double nr = r.x*r.x + r.y*r.y + r.z*r.z + r.w*r.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
      !std::isfinite(nq) || !std::isfinite(nr) ||
      std::abs(nq-1.0) > 1e-5 || std::abs(nr-1.0) > 1e-5) {
    reason = "invalid_pose_or_extrinsic"; return false;
  }
  std::set<std::uint64_t> ids;
  for (const auto& f : frame.features) {
    if (!ids.insert(f.feature_id).second ||
        !std::isfinite(f.pixel.x) || !std::isfinite(f.pixel.y) ||
        !std::isfinite(f.normalized.x) || !std::isfinite(f.normalized.y)) {
      reason = "invalid_feature_identity_or_pixel"; return false;
    }
    if (f.position_valid && (!std::isfinite(f.position_camera.x) ||
        !std::isfinite(f.position_camera.y) || !std::isfinite(f.position_camera.z) ||
        f.position_camera.z <= 0.0)) {
      reason = "invalid_camera_point"; return false;
    }
    if (f.depth_variance_valid && (!std::isfinite(f.depth_variance) || f.depth_variance <= 0.0)) {
      reason = "invalid_depth_variance"; return false;
    }
    if (f.descriptor_valid) {
      if (f.descriptor.empty() || frame.local_descriptor_model.empty()) {
        reason = "missing_descriptor_payload"; return false;
      }
      for (float value : f.descriptor) {
        if (!std::isfinite(value)) { reason = "invalid_descriptor"; return false; }
      }
    }
  }
  if (frame.image_included &&
      (frame.image.header.stamp != frame.header.stamp || frame.image.data.empty() ||
       frame.image.height == 0 || frame.image.width == 0 ||
       frame.image.data.size() != static_cast<std::size_t>(frame.image.step)*frame.image.height)) {
    reason = "invalid_image_or_capture_time"; return false;
  }
  reason = "ok"; return true;
}
bool KeyframeStore::insert(const rdswarm_msgs::Keyframe& frame, std::string& reason) {
  if (!validateKeyframe(frame, reason)) return false;
  std::lock_guard<std::mutex> guard(mutex_);
  // 键重复不覆盖历史观测；重发与新观测必须严格区分。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
  if (!frames_.emplace(frameKey(frame.key), frame).second) {
    reason = "duplicate_key"; return false;
  }
  reason = "ok"; return true;
}
bool KeyframeStore::get(const FrameKey& key, rdswarm_msgs::Keyframe& frame) const {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto it = frames_.find(key);
  if (it == frames_.end()) return false;
  frame = it->second; return true;
}
std::size_t KeyframeStore::size() const {
  std::lock_guard<std::mutex> guard(mutex_); return frames_.size();
}
}  // namespace rdswarm
