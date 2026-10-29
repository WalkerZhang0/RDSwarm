#include <rdswarm_loop/feature_enrichment.hpp>
#include <rdswarm_loop/keyframe_store.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace rdswarm {
bool enrichKeyframe(const rdswarm_msgs::Keyframe& input,
    const DenseSuperPoint& dense, const SuperPointCpu& model,
    const camodocal::CameraPtr& camera, const std::string& model_id,
    const FeatureEnrichmentOptions& options, rdswarm_msgs::Keyframe& output,
    FeatureEnrichmentDiagnostics& diagnostics, std::string& reason) {
  output = rdswarm_msgs::Keyframe();
  diagnostics = FeatureEnrichmentDiagnostics();
  if (!validateKeyframe(input, reason)) return false;
  if (model_id.empty() || !std::isfinite(options.dedup_radius_px) || options.dedup_radius_px < 0.0) {
    reason = "invalid_enrichment_options"; return false;
  }
  if (options.append_image_features && (!camera ||
      camera->imageWidth() != dense.image_size.width || camera->imageHeight() != dense.image_size.height)) {
    reason = "camera_calibration_missing_or_image_size_mismatch"; return false;
  }
  if (input.image_included && (static_cast<int>(input.image.width) != dense.image_size.width ||
      static_cast<int>(input.image.height) != dense.image_size.height)) {
    reason = "keyframe_image_size_mismatch"; return false;
  }
  std::vector<cv::Point2f> pixels;
  for (const auto& feature : input.features) {
    if (feature.feature_id >= kImageOnlyFeatureIdBase) {
      reason = "raw_keyframe_uses_reserved_image_feature_id"; return false;
    }
    pixels.emplace_back(feature.pixel.x, feature.pixel.y);
    diagnostics.original_valid_3d += feature.position_valid;
    if (options.append_image_features) {
      Eigen::Vector3d ray;
      camera->liftProjective(Eigen::Vector2d(feature.pixel.x, feature.pixel.y), ray);
      if (!ray.allFinite() || std::abs(ray.z()) < 1e-12 ||
          (ray.head<2>() / ray.z() - Eigen::Vector2d(feature.normalized.x, feature.normalized.y)).norm() > 1e-4) {
        reason = "camera_calibration_inconsistent_with_vio_observations"; return false;
      }
    }
  }
  diagnostics.original_features = pixels.size();
  std::vector<std::vector<float>> descriptors;
  std::vector<unsigned char> valid;
  if (!sampleSuperPointDescriptors(dense.descriptors, dense.image_size, pixels, descriptors, valid, reason))
    return false;
  auto enriched = input;
  enriched.local_descriptor_model = model_id;
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    enriched.features[i].descriptor = std::move(descriptors[i]);
    enriched.features[i].descriptor_valid = valid[i] != 0;
  }
  if (options.append_image_features) {
    if (options.grid_rows<1 || options.grid_cols<1) {
      reason = "invalid_feature_grid"; return false;
    }
    auto detector_options=options.detector;
    detector_options.grid_rows=options.grid_rows;
    detector_options.grid_cols=options.grid_cols;
    cv::Mat available(dense.image_size,CV_8U,cv::Scalar(255));
    const int dedup_radius=static_cast<int>(std::ceil(options.dedup_radius_px));
    for (const auto& pixel : pixels)
      cv::circle(available,cv::Point(cvRound(pixel.x),cvRound(pixel.y)),dedup_radius,cv::Scalar(0),-1);
    std::vector<cv::Point2f> detected;
    if (!model.detect(dense, available, detector_options, detected, reason)) return false;
    diagnostics.detected_features = detected.size();
    std::vector<cv::Point2f> additions;
    std::vector<std::size_t> ranks;
    const double radius_squared = options.dedup_radius_px * options.dedup_radius_px;
    for (std::size_t rank = 0; rank < detected.size(); ++rank) {
      const auto candidate = detected[rank];
      bool duplicate = false;
      // 原二维点优先保留；只删除靠近原点或已追加点的新检测，不替换旧坐标或描述子。 (Ref: 2026-10-25_Extend_Peer_2D_Features.md)
      for (const auto& pixel : pixels) {
        const auto delta = candidate - pixel;
        if (delta.dot(delta) <= radius_squared) { duplicate = true; break; }
      }
      if (duplicate) { ++diagnostics.duplicate_pixels; continue; }
      additions.push_back(candidate); ranks.push_back(rank); pixels.push_back(candidate);
    }
    if (!sampleSuperPointDescriptors(dense.descriptors, dense.image_size, additions, descriptors, valid, reason))
      return false;
    for (std::size_t i = 0; i < additions.size(); ++i) {
      if (!valid[i]) continue;
      Eigen::Vector3d ray;
      camera->liftProjective(Eigen::Vector2d(additions[i].x, additions[i].y), ray);
      if (!ray.allFinite() || std::abs(ray.z()) < 1e-12) {
        reason = "invalid_camera_unprojection"; return false;
      }
      rdswarm_msgs::TrackedFeature feature;
      feature.feature_id = kImageOnlyFeatureIdBase + ranks[i];
      feature.pixel.x = additions[i].x; feature.pixel.y = additions[i].y;
      feature.normalized.x = ray.x() / ray.z(); feature.normalized.y = ray.y() / ray.z();
      feature.normalized.z = 1.0;
      feature.descriptor = std::move(descriptors[i]); feature.descriptor_valid = true;
      // 新检测仅提供二维观测；不能继承邻近 VIO 点的深度或协方差。 (Ref: 2026-10-25_Extend_Peer_2D_Features.md)
      feature.position_valid = false; feature.depth_variance_valid = false;
      enriched.features.push_back(std::move(feature)); ++diagnostics.appended_features;
    }
  }
  if (!validateKeyframe(enriched, reason)) return false;
  output = std::move(enriched); reason = "ok"; return true;
}
}
