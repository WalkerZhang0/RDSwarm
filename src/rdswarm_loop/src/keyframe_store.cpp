#include <rdswarm_loop/keyframe_store.hpp>
#include <cmath>
#include <set>
#include <filesystem>
#include <fstream>
#include <ros/serialization.h>
#include <stdexcept>

namespace rdswarm {
namespace {
std::filesystem::path archivePath(const std::string& directory,const FrameKey& key) {
  static const char* hex="0123456789abcdef";
  std::string session;
  for(unsigned char c:key.session_id){session+=hex[c>>4];session+=hex[c&15];}
  return std::filesystem::path(directory)/std::to_string(key.agent_id)/session/(std::to_string(key.frame_id)+".bin");
}
}
void KeyframeStore::configureArchive(const std::string& directory,std::size_t memory_frames) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(!frames_.empty())throw std::runtime_error("configure_archive_before_insertion");
  if(memory_frames>0 && directory.empty())throw std::runtime_error("bounded_cache_requires_archive");
  archive_directory_=directory;memory_frames_=memory_frames;
  if(!directory.empty())std::filesystem::create_directories(directory);
}
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
  const auto key=frameKey(frame.key);
  if(frames_.count(key)){reason="duplicate_key";return false;}
  if(!archive_directory_.empty()) {
    try {
      const auto path=archivePath(archive_directory_,key);
      if(std::filesystem::exists(path)){reason="duplicate_key";return false;}
      std::filesystem::create_directories(path.parent_path());
      std::vector<std::uint8_t> bytes(ros::serialization::serializationLength(frame));
      ros::serialization::OStream stream(bytes.data(),bytes.size());
      ros::serialization::serialize(stream,frame);
      const auto temporary=path.string()+".tmp";
      std::ofstream output(temporary,std::ios::binary|std::ios::trunc);
      output.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());output.close();
      if(!output){reason="archive_write_failed";return false;}
      std::filesystem::rename(temporary,path);
    } catch(const std::exception& e){reason=std::string("archive_write_failed:")+e.what();return false;}
  }
  // 键重复不覆盖历史观测；重发与新观测必须严格区分。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
  if (!frames_.emplace(frameKey(frame.key), frame).second) {
    reason = "duplicate_key"; return false;
  }
  insertion_order_.push_back(key);
  // RAM 只保留工作集，历史关键帧仍从磁盘按原身份读取。 (Ref: 2026-10-28_Add_Global_Retrieval.md)
  while(memory_frames_>0 && frames_.size()>memory_frames_){frames_.erase(insertion_order_.front());insertion_order_.pop_front();}
  reason = "ok"; return true;
}
bool KeyframeStore::get(const FrameKey& key, rdswarm_msgs::Keyframe& frame) const {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto it = frames_.find(key);
  if (it == frames_.end()) {
    if(archive_directory_.empty())return false;
    try {
      const auto path=archivePath(archive_directory_,key);
      std::ifstream input(path,std::ios::binary|std::ios::ate);
      if(!input)return false;
      const auto size=input.tellg();if(size<=0 || size>64*1024*1024)return false;
      std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));input.seekg(0);
      if(!input.read(reinterpret_cast<char*>(bytes.data()),bytes.size()))return false;
      ros::serialization::IStream stream(bytes.data(),bytes.size());
      ros::serialization::deserialize(stream,frame);std::string reason;
      return stream.getLength()==0 && frameKey(frame.key)==key && validateKeyframe(frame,reason);
    }catch(const std::exception&){return false;}
  }
  frame = it->second; return true;
}
std::size_t KeyframeStore::size() const {
  std::lock_guard<std::mutex> guard(mutex_); return frames_.size();
}
}  // namespace rdswarm
