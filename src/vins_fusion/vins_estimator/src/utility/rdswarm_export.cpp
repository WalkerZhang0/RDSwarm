#include "rdswarm_export.h"
#include "../estimator/estimator.h"
#include <rdswarm_msgs/Keyframe.h>
#include <rdswarm_msgs/VioState.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <map>
#include <mutex>
#include <sstream>
#include <cmath>
#include <stdexcept>

namespace {
struct SourceImage {
  std_msgs::Header header;
  cv::Mat image;
};
struct ExportState {
  std::mutex mutex;
  ros::Publisher keyframes, states;
  bool ready = false;
  bool include_image = true;
  int cache_limit = 120;
  std::uint32_t agent_id = 0;
  std::string base_session;
  std::uint64_t epoch = 0;
  std::uint64_t last_received_ns = 0;
  std::uint64_t last_keyframe_ns = 0;
  std::map<double, SourceImage> images;
  std::string session() const {
    return base_session + ".e" + std::to_string(epoch);
  }
  std::string frame() const {
    return "vio/agent_" + std::to_string(agent_id) + "/" + session();
  }
};
ExportState& exports() {
  static ExportState state;
  return state;
}
void fillPose(const Eigen::Vector3d& p, const Eigen::Matrix3d& r,
              geometry_msgs::Pose& out) {
  out.position.x=p.x(); out.position.y=p.y(); out.position.z=p.z();
  const Eigen::Quaterniond q(r);
  out.orientation.x=q.x(); out.orientation.y=q.y();
  out.orientation.z=q.z(); out.orientation.w=q.w();
}
}  // namespace

void initializeRdswarmExport(ros::NodeHandle& nh) {
  auto& state = exports();
  std::lock_guard<std::mutex> guard(state.mutex);
  int agent = 0;
  nh.param("agent_id", agent, 0);
  if (agent < 0) throw std::runtime_error("agent_id must be nonnegative");
  state.agent_id = static_cast<std::uint32_t>(agent);
  nh.param<std::string>("session_id", state.base_session, "");
  if (state.base_session.empty())
    state.base_session = "run_" + std::to_string(ros::WallTime::now().toNSec());
  nh.param("include_keyframe_image", state.include_image, true);
  nh.param("source_image_cache_size", state.cache_limit, 120);
  if (state.cache_limit < WINDOW_SIZE+1)
    throw std::runtime_error("source_image_cache_size must exceed VIO window size");
  state.keyframes = nh.advertise<rdswarm_msgs::Keyframe>("keyframes", 20);
  state.states = nh.advertise<rdswarm_msgs::VioState>("states", 100);
  state.ready = true;
}

bool cacheRdswarmImage(const std_msgs::Header& header, const cv::Mat& image) {
  auto& state = exports();
  std::lock_guard<std::mutex> guard(state.mutex);
  if (!state.ready) return true;
  const auto ns = header.stamp.toNSec();
  if (header.stamp.isZero() || image.empty() || ns <= state.last_received_ns) {
    ROS_ERROR_THROTTLE(1.0, "RDSwarm rejected empty/nonmonotonic source image; reset VIO before replaying time.");
    return false;
  }
  // VIO 内部沿用 double 时间；对外导出时从同一输入的缓存恢复原始整数采样时间。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
  state.images.emplace(header.stamp.toSec(), SourceImage{header, image.clone()});
  state.last_received_ns = ns;
  while (state.images.size() > static_cast<std::size_t>(state.cache_limit))
    state.images.erase(state.images.begin());
  return true;
}

void resetRdswarmExport() {
  auto& state = exports();
  std::lock_guard<std::mutex> guard(state.mutex);
  if (!state.ready) return;
  // VIO 重置产生新 epoch，避免轨迹重启后 ID 与旧地标、旧回环混用。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
  ++state.epoch;
  state.images.clear();
  state.last_received_ns = state.last_keyframe_ns = 0;
  ROS_WARN_STREAM("RDSwarm VIO session reset: " << state.session());
}

std::size_t rdswarmKeyframeSubscriberCount() {
  auto& state = exports();
  std::lock_guard<std::mutex> guard(state.mutex);
  return state.ready ? state.keyframes.getNumSubscribers() : 0;
}

void publishRdswarmState(const Estimator& estimator, const std_msgs::Header& header) {
  auto& state = exports();
  std::lock_guard<std::mutex> guard(state.mutex);
  if (!state.ready || estimator.solver_flag != Estimator::NON_LINEAR) return;
  const auto source = state.images.find(header.stamp.toSec());
  if (source == state.images.end()) return;
  rdswarm_msgs::VioState out;
  out.header = source->second.header;
  out.header.frame_id = state.frame();
  out.agent_id = state.agent_id;
  out.session_id = state.session();
  out.vio_frame_id = state.frame();
  out.initialized = true;
  fillPose(estimator.Ps[WINDOW_SIZE], estimator.Rs[WINDOW_SIZE], out.body_pose_vio);
  out.velocity_vio.x=estimator.Vs[WINDOW_SIZE].x();
  out.velocity_vio.y=estimator.Vs[WINDOW_SIZE].y();
  out.velocity_vio.z=estimator.Vs[WINDOW_SIZE].z();
  out.covariance_valid=false;
  out.covariance_source="not_estimated_by_frontend";
  state.states.publish(out);
}

void publishRdswarmKeyframe(const Estimator& estimator) {
  auto& state = exports();
  std::lock_guard<std::mutex> guard(state.mutex);
  if (!state.ready || estimator.solver_flag != Estimator::NON_LINEAR ||
      estimator.marginalization_flag != 0) return;
  const int k = WINDOW_SIZE-2;
  const auto source = state.images.find(estimator.Headers[k]);
  if (source == state.images.end()) {
    ROS_ERROR_THROTTLE(1.0, "RDSwarm cannot export keyframe: original capture image expired.");
    return;
  }
  const auto capture_ns = source->second.header.stamp.toNSec();
  if (capture_ns <= state.last_keyframe_ns) return;
  rdswarm_msgs::Keyframe out;
  out.header = source->second.header;
  out.header.frame_id = state.frame();
  out.key.agent_id=state.agent_id;
  out.key.session_id=state.session();
  out.key.frame_id=capture_ns;
  out.vio_frame_id=state.frame();
  out.initialized=true;
  out.capture_stamp_exact=true;
  fillPose(estimator.Ps[k], estimator.Rs[k], out.body_pose_vio);
  const Eigen::Quaterniond qbc(estimator.ric[0]);
  out.body_T_camera.translation.x=estimator.tic[0].x();
  out.body_T_camera.translation.y=estimator.tic[0].y();
  out.body_T_camera.translation.z=estimator.tic[0].z();
  out.body_T_camera.rotation.x=qbc.x(); out.body_T_camera.rotation.y=qbc.y();
  out.body_T_camera.rotation.z=qbc.z(); out.body_T_camera.rotation.w=qbc.w();
  out.velocity_vio.x=estimator.Vs[k].x();
  out.velocity_vio.y=estimator.Vs[k].y();
  out.velocity_vio.z=estimator.Vs[k].z();
  for (const auto& track : estimator.f_manager.feature) {
    const int observation = k-track.start_frame;
    if (track.feature_id < 0 || track.start_frame >= k ||
        observation < 0 || observation >= static_cast<int>(track.feature_per_frame.size()) ||
        track.solve_flag != 1 || !std::isfinite(track.estimated_depth) ||
        track.estimated_depth <= 0.0) continue;
    const int host = track.start_frame;
    const Eigen::Vector3d ph = track.feature_per_frame[0].point * track.estimated_depth;
    const Eigen::Vector3d pw = estimator.Rs[host] *
        (estimator.ric[0]*ph + estimator.tic[0]) + estimator.Ps[host];
    // 三维点先还原到当前关键帧相机系，保持与该帧像素及 feature_id 一一对应。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
    const Eigen::Vector3d pc = estimator.ric[0].transpose() *
        (estimator.Rs[k].transpose()*(pw-estimator.Ps[k])-estimator.tic[0]);
    if (!pc.allFinite() || pc.z() <= 0.0) continue;
    const auto& obs = track.feature_per_frame[observation];
    if (!obs.point.allFinite() || !obs.uv.allFinite() || std::abs(obs.point.z()) < 1e-12) continue;
    rdswarm_msgs::TrackedFeature feature;
    feature.feature_id=static_cast<std::uint64_t>(track.feature_id);
    feature.pixel.x=obs.uv.x(); feature.pixel.y=obs.uv.y();
    feature.normalized.x=obs.point.x()/obs.point.z();
    feature.normalized.y=obs.point.y()/obs.point.z();
    feature.normalized.z=1.0;
    feature.position_camera.x=pc.x();
    feature.position_camera.y=pc.y();
    feature.position_camera.z=pc.z();
    feature.position_valid=true;
    feature.depth_variance_valid=false;
    feature.descriptor_valid=false;
    out.features.push_back(feature);
  }
  if (state.include_image) {
    out.image=*cv_bridge::CvImage(source->second.header,
        sensor_msgs::image_encodings::MONO8, source->second.image).toImageMsg();
    out.image_included=true;
  }
  state.keyframes.publish(out);
  state.last_keyframe_ns=capture_ns;
}
