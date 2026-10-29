#pragma once
#include <rdswarm_features/superpoint_cpu.hpp>
#include <rdswarm_msgs/Keyframe.h>
#include <camodocal/camera_models/Camera.h>
#include <cstdint>

namespace rdswarm {
// 高位 ID 为当前帧独立图像检测点；不表示跨帧 VIO 轨迹或三维地标。 (Ref: 2026-10-25_Extend_Peer_2D_Features.md)
constexpr std::uint64_t kImageOnlyFeatureIdBase = std::uint64_t{1} << 63;
struct FeatureEnrichmentOptions {
  bool append_image_features = true;
  DetectorOptions detector;
  double dedup_radius_px = 4.0;
  int grid_rows = 16, grid_cols = 12;
};
struct FeatureEnrichmentDiagnostics {
  std::size_t original_features = 0, original_valid_3d = 0;
  std::size_t detected_features = 0, duplicate_pixels = 0, appended_features = 0;
};
bool enrichKeyframe(const rdswarm_msgs::Keyframe& input,
    const DenseSuperPoint& dense, const SuperPointCpu& model,
    const camodocal::CameraPtr& camera, const std::string& model_id,
    const FeatureEnrichmentOptions&, rdswarm_msgs::Keyframe& output,
    FeatureEnrichmentDiagnostics&, std::string& reason);
}
