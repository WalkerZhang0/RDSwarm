#include <rdswarm_loop/feature_enrichment.hpp>
#include <rdswarm_loop/loop_core.hpp>
#include <camodocal/camera_models/PinholeCamera.h>
#include <iostream>
#include <cmath>

int main() {
  rdswarm_msgs::Keyframe raw;
  raw.key.agent_id=1; raw.key.session_id="enrichment_test"; raw.key.frame_id=23;
  raw.header.stamp=ros::Time(100,123); raw.vio_frame_id="vio/test";
  raw.initialized=true; raw.capture_stamp_exact=true;
  raw.image.width=16; raw.image.height=16;
  raw.body_pose_vio.orientation.w=1; raw.body_T_camera.rotation.w=1;
  rdswarm_msgs::TrackedFeature a,b;
  a.feature_id=7; a.pixel.x=3.5; a.pixel.y=3.5;
  a.normalized.x=-.375; a.normalized.y=-.375; a.normalized.z=1;
  a.position_valid=true; a.position_camera.x=-1.5; a.position_camera.y=-1.5; a.position_camera.z=4;
  b.feature_id=8; b.pixel.x=7; b.pixel.y=12;
  b.normalized.x=-1.0/12; b.normalized.y=1.0/3; b.normalized.z=1;
  raw.features={a,b};
  rdswarm::DenseSuperPoint dense; dense.image_size={16,16};
  dense.scores=cv::Mat::zeros(16,16,CV_32F);
  dense.scores.at<float>(3,3)=.9; dense.scores.at<float>(4,12)=.8; dense.scores.at<float>(5,14)=.7;
  int sizes[]={1,256,2,2}; dense.descriptors=cv::Mat(4,sizes,CV_32F,cv::Scalar(0));
  for(int i=0;i<4;++i)dense.descriptors.ptr<float>()[i*4+i]=1;
  camodocal::CameraPtr camera(new camodocal::PinholeCamera("test",16,16,0,0,0,0,12,12,8,8));
  rdswarm::FeatureEnrichmentOptions options;
  options.detector.min_distance=0; options.detector.max_points=2; options.detector.border=1; options.dedup_radius_px=2;
  options.grid_rows=2; options.grid_cols=2;
  rdswarm::SuperPointCpu model; rdswarm_msgs::Keyframe result;
  rdswarm::FeatureEnrichmentDiagnostics diagnostics; std::string reason;
  rdswarm::DenseSuperPoint grid_dense;
  grid_dense.image_size={16,16};
  grid_dense.scores=cv::Mat::zeros(16,16,CV_32F);
  grid_dense.scores.at<float>(2,2)=.95f;
  grid_dense.scores.at<float>(3,3)=.94f;
  grid_dense.scores.at<float>(12,12)=.50f;
  auto grid_options=options.detector;
  grid_options.score_threshold=.1f; grid_options.max_points=2;
  grid_options.min_distance=0; grid_options.border=1;
  grid_options.grid_rows=2; grid_options.grid_cols=2;
  std::vector<cv::Point2f> grid_points;
  if(!model.detect(grid_dense,cv::Mat(),grid_options,grid_points,reason) ||
     grid_points.size()!=2 || grid_points[0]!=cv::Point2f(2,2) ||
     grid_points[1]!=cv::Point2f(12,12)) return 12;
  if(!rdswarm::enrichKeyframe(raw,dense,model,camera,"model",options,result,diagnostics,reason)) {
    std::cerr<<reason<<"\n";return 1;
  }
  if(result.features.size()!=4 || diagnostics.duplicate_pixels!=0 || diagnostics.detected_features!=2 || diagnostics.original_valid_3d!=1 ||
     diagnostics.appended_features!=2 || result.key!=raw.key || result.header.stamp!=raw.header.stamp) {
    std::cerr<<"counts total="<<result.features.size()<<" detected="<<diagnostics.detected_features
             <<" duplicate="<<diagnostics.duplicate_pixels<<" appended="<<diagnostics.appended_features<<"\n";
    return 2;
  }
  for(std::size_t i=0;i<raw.features.size();++i) {
    auto actual=result.features[i]; actual.descriptor.clear();actual.descriptor_valid=false;
    if(actual!=raw.features[i] || !result.features[i].descriptor_valid) return 3;
  }
  for(std::size_t i=raw.features.size();i<result.features.size();++i) {
    const auto& added=result.features[i];
    if(added.feature_id<rdswarm::kImageOnlyFeatureIdBase || added.position_valid || added.depth_variance_valid ||
       added.position_camera.z!=0 || added.normalized.z!=1) return 4;
  }
  const auto peer=rdswarm::localPacket(result,false);
  if(peer.features.size()!=4 || !rdswarm::validatePacket(peer,reason)) return 5;
  for(const auto& feature:peer.features)if(feature.position_valid || feature.position_camera.z!=0) return 6;
  const auto first=result;
  if(!rdswarm::enrichKeyframe(raw,dense,model,camera,"model",options,result,diagnostics,reason) || result!=first) return 7;
  camodocal::CameraPtr wrong(new camodocal::PinholeCamera("wrong",32,16,0,0,0,0,12,12,8,8));
  if(rdswarm::enrichKeyframe(raw,dense,model,wrong,"model",options,result,diagnostics,reason)) return 8;
  camodocal::CameraPtr wrong_intrinsics(new camodocal::PinholeCamera("wrong_intrinsics",16,16,0,0,0,0,20,20,8,8));
  if(rdswarm::enrichKeyframe(raw,dense,model,wrong_intrinsics,"model",options,result,diagnostics,reason) ||
     reason!="camera_calibration_inconsistent_with_vio_observations") return 11;
  options.append_image_features=false;
  if(!rdswarm::enrichKeyframe(raw,dense,model,camodocal::CameraPtr(),"model",options,result,diagnostics,reason) ||
     result.features.size()!=2 || diagnostics.appended_features!=0) return 9;
  options.append_image_features=true; options.dedup_radius_px=-1;
  if(rdswarm::enrichKeyframe(raw,dense,model,camera,"model",options,result,diagnostics,reason)) return 10;
  std::cout<<"PASS: preserves VIO geometry and depthless tracks, pixel deduplication, frame-local IDs, calibrated rays, deterministic output, no peer depth and invalid calibration rejection.\n";
}
