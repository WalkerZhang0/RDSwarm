#include <rdswarm_loop/loop_core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <sstream>
#include <chrono>
namespace rdswarm {
namespace {
bool finitePose(const geometry_msgs::Transform& t) {
  const auto& q=t.rotation;
  const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(n) && std::abs(n-1)<1e-5 &&
      std::isfinite(t.translation.x) && std::isfinite(t.translation.y) &&
      std::isfinite(t.translation.z);
}
Eigen::Matrix4d matrix(const geometry_msgs::Transform& t) {
  Eigen::Matrix4d m=Eigen::Matrix4d::Identity();
  m.block<3,3>(0,0)=Eigen::Quaterniond(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z).toRotationMatrix();
  m.block<3,1>(0,3)=Eigen::Vector3d(t.translation.x,t.translation.y,t.translation.z);
  return m;
}
void fillPose(const Eigen::Matrix4d& m,geometry_msgs::Pose& p) {
  p.position.x=m(0,3);p.position.y=m(1,3);p.position.z=m(2,3);
  const Eigen::Quaterniond q(m.block<3,3>(0,0));
  p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
}
bool row(const rdswarm_msgs::TrackedFeature& f,std::vector<float>& output) {
  if(!f.descriptor_valid || f.descriptor.empty()) return false;
  double n=0;for(float v:f.descriptor) { if(!std::isfinite(v)) return false;n+=double(v)*v; }
  if(n<1e-20) return false;
  output=f.descriptor;for(float& v:output)v/=std::sqrt(n);return true;
}
bool geometricSpread(const std::vector<cv::Point3f>& points,double ratio) {
  if(points.size()<3) return false;
  Eigen::MatrixXd x(points.size(),3);
  for(std::size_t i=0;i<points.size();++i)x.row(i)<<points[i].x,points[i].y,points[i].z;
  const Eigen::RowVector3d mean=x.colwise().mean();
  x.rowwise()-=mean;
  const auto sv=Eigen::JacobiSVD<Eigen::MatrixXd>(x).singularValues();
  return sv.size()==3 && sv[0]>1e-6 && sv[1]/sv[0]>=ratio;
}
bool optionsValid(const LoopOptions& o) {
  return std::isfinite(o.ratio_threshold) && o.ratio_threshold>0 && o.ratio_threshold<1 &&
      std::isfinite(o.max_descriptor_distance) && o.max_descriptor_distance>0 &&
      o.min_matches>=6 && o.min_inliers>=6 && o.ransac_iterations>0 &&
      std::isfinite(o.min_inlier_ratio) && o.min_inlier_ratio>0 && o.min_inlier_ratio<=1 &&
      std::isfinite(o.pnp_threshold_normalized) && o.pnp_threshold_normalized>0 &&
      std::isfinite(o.min_second_singular_ratio) && o.min_second_singular_ratio>0 &&
      std::isfinite(o.min_positive_depth_fraction) && o.min_positive_depth_fraction>0 &&
      o.min_positive_depth_fraction<=1 &&
      (!o.use_nominal_covariance || (std::isfinite(o.nominal_rotation_std) &&
       std::isfinite(o.nominal_translation_std) && o.nominal_rotation_std>0 && o.nominal_translation_std>0));
}
}
std::string identityText(const FrameKey& k) {
  return std::to_string(k.agent_id)+"#"+std::to_string(k.session_id.size())+"#"+
      k.session_id+"#"+std::to_string(k.frame_id);
}
bool validatePacket(const rdswarm_msgs::LocalFeaturePacket& p,std::string& reason) {
  if(!frameKey(p.key).valid() || p.header.stamp.isZero() ||
     p.local_descriptor_model.empty() || p.descriptor_dimension==0 || !finitePose(p.body_T_camera)) {
    reason="invalid_packet_metadata";return false;
  }
  std::set<std::uint64_t> ids;
  for(const auto& f:p.features) {
    if(!ids.insert(f.feature_id).second || !std::isfinite(f.normalized.x) ||
       !std::isfinite(f.normalized.y) || !std::isfinite(f.normalized.z) ||
       !std::isfinite(f.pixel.x) || !std::isfinite(f.pixel.y) ||
       std::abs(f.normalized.z-1)>1e-4) {reason="invalid_feature_geometry";return false;}
    if(f.position_valid && (!std::isfinite(f.position_camera.x) ||
       !std::isfinite(f.position_camera.y) || !std::isfinite(f.position_camera.z) ||
       f.position_camera.z<=0)) {reason="invalid_3d_point";return false;}
    if(f.descriptor_valid) {
      std::vector<float> d;
      if(f.descriptor.size()!=p.descriptor_dimension || !row(f,d)) {
        reason="invalid_descriptor";return false;
      }
    }
  }
  reason="ok";return true;
}
rdswarm_msgs::LocalFeaturePacket localPacket(const rdswarm_msgs::Keyframe& f,bool xyz) {
  rdswarm_msgs::LocalFeaturePacket p;
  p.header=f.header;p.key=f.key;p.body_T_camera=f.body_T_camera;
  p.local_descriptor_model=f.local_descriptor_model;
  for(const auto& feature:f.features)if(feature.descriptor_valid) {
    p.features.push_back(feature);
    if(!xyz) {
      auto& dst=p.features.back();dst.position_valid=false;
      dst.position_camera=geometry_msgs::Point();dst.depth_variance_valid=false;
    }
    if(p.descriptor_dimension==0)p.descriptor_dimension=feature.descriptor.size();
  }
  return p;
}
namespace {
bool verifyLoopInternal(const rdswarm_msgs::LocalFeaturePacket& local,
    const rdswarm_msgs::LocalFeaturePacket& remote,const LoopOptions& opts,
    const std::vector<ExternalMatch>* supplied_matches, const std::string& matcher_name,
    rdswarm_msgs::LoopConstraint& constraint,rdswarm_msgs::VerifiedMatches& match_set,
    LoopDiagnostics& diag) {
  constraint=rdswarm_msgs::LoopConstraint();match_set=rdswarm_msgs::VerifiedMatches();diag=LoopDiagnostics();
  diag.matcher=matcher_name;
  const auto total_start=std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point geometry_start;
  bool geometry_started=false;
  auto fail=[&](const std::string& stage,const std::string& status) {
    diag.stage=stage;diag.status=status;
    if (geometry_started) {
      diag.geometry_ms=std::chrono::duration<double,std::milli>(
          std::chrono::steady_clock::now()-geometry_start).count();
    }
    diag.total_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-total_start).count();
    return false;
  };
  std::string reason;
  if(!optionsValid(opts))return fail("input","invalid_options");
  if(!validatePacket(local,reason) || !validatePacket(remote,reason))return fail("input",reason);
  const auto ka=frameKey(local.key),kb=frameKey(remote.key);
  if(ka==kb)return fail("input","same_frame");
  if(local.local_descriptor_model!=remote.local_descriptor_model ||
      local.descriptor_dimension!=remote.descriptor_dimension)return fail("input","descriptor_model_mismatch");
  std::vector<int> local_ids,remote_ids;
  cv::Mat ld,rd;
  for(std::size_t i=0;i<local.features.size();++i) {
    std::vector<float> d;
    if(local.features[i].position_valid && row(local.features[i],d)) {
      ld.push_back(cv::Mat(1,d.size(),CV_32F,d.data()).clone());local_ids.push_back(i);
    }
  }
  for(std::size_t i=0;i<remote.features.size();++i) {
    std::vector<float> d;
    if(row(remote.features[i],d)) {
      rd.push_back(cv::Mat(1,d.size(),CV_32F,d.data()).clone());remote_ids.push_back(i);
    }
  }
  diag.local_features=local_ids.size();diag.remote_features=remote_ids.size();
  std::vector<cv::Point3f> object;
  std::vector<cv::Point2f> image;
  const auto matching_start=std::chrono::steady_clock::now();
  if (supplied_matches) {
    std::set<std::size_t> used_local,used_remote;
    for (const auto& candidate : *supplied_matches) {
      if (candidate.local_feature_index>=local.features.size() ||
          candidate.remote_feature_index>=remote.features.size() ||
          !used_local.insert(candidate.local_feature_index).second ||
          !used_remote.insert(candidate.remote_feature_index).second ||
          !std::isfinite(candidate.confidence) || candidate.confidence<=0.0f ||
          candidate.confidence>1.0f) {
        return fail("match","invalid_external_match");
      }
      const auto& a=local.features[candidate.local_feature_index];
      const auto& b=remote.features[candidate.remote_feature_index];
      if (!a.position_valid || !a.descriptor_valid || !b.descriptor_valid)
        return fail("match","external_match_feature_invalid");
      object.emplace_back(a.position_camera.x,a.position_camera.y,a.position_camera.z);
      image.emplace_back(b.normalized.x,b.normalized.y);
      rdswarm_msgs::FeatureMatch match;
      match.local_feature_id=a.feature_id;match.remote_feature_id=b.feature_id;
      match.descriptor_distance=1.0f-candidate.confidence;match_set.matches.push_back(match);
    }
  } else {
    if(ld.rows<opts.min_matches || rd.rows<2)return fail("match","insufficient_features");
    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<std::vector<cv::DMatch>> nearest;
    std::vector<cv::DMatch> reverse;
    matcher.knnMatch(ld,rd,nearest,2);matcher.match(rd,ld,reverse);
    for(const auto& nn:nearest) {
      if(nn.size()!=2 || nn[0].distance>=opts.ratio_threshold*nn[1].distance ||
         nn[0].distance>opts.max_descriptor_distance ||
         reverse[nn[0].trainIdx].trainIdx!=nn[0].queryIdx)continue;
      const auto& a=local.features[local_ids[nn[0].queryIdx]];
      const auto& b=remote.features[remote_ids[nn[0].trainIdx]];
      object.emplace_back(a.position_camera.x,a.position_camera.y,a.position_camera.z);
      image.emplace_back(b.normalized.x,b.normalized.y);
      rdswarm_msgs::FeatureMatch match;
      match.local_feature_id=a.feature_id;match.remote_feature_id=b.feature_id;
      match.descriptor_distance=nn[0].distance;match_set.matches.push_back(match);
    }
  }
  diag.matching_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-matching_start).count();
  diag.matches=object.size();
  if(object.size()<static_cast<std::size_t>(opts.min_matches))return fail("match","insufficient_matches");
  if(!geometricSpread(object,opts.min_second_singular_ratio))return fail("geometry","collinear_or_tiny_3d_geometry");
  geometry_start=std::chrono::steady_clock::now(); geometry_started=true;
  cv::Mat r,t,inliers;
  try {
    // 使用归一化坐标时 K=I，RANSAC 门限也必须采用归一化单位。 (Ref: 2026-10-24_Portable_Loop_Core.md)
    cv::setRNGSeed(opts.ransac_seed);
    const bool ok=cv::solvePnPRansac(object,image,cv::Mat::eye(3,3,CV_64F),cv::Mat(),
        r,t,false,opts.ransac_iterations,opts.pnp_threshold_normalized,0.999,inliers,cv::SOLVEPNP_EPNP);
    diag.inliers=inliers.rows;
    if(!ok || inliers.rows<opts.min_inliers ||
       double(inliers.rows)/object.size()<opts.min_inlier_ratio)return fail("pnp","insufficient_inliers");
    std::vector<cv::Point3f> inlier_object;
    std::vector<cv::Point2f> inlier_image;
    for(int i=0;i<inliers.rows;++i) {
      const int k=inliers.at<int>(i);inlier_object.push_back(object[k]);inlier_image.push_back(image[k]);
    }
    if(!geometricSpread(inlier_object,opts.min_second_singular_ratio))return fail("geometry","degenerate_inlier_geometry");
    cv::solvePnPRefineLM(inlier_object,inlier_image,cv::Mat::eye(3,3,CV_64F),cv::Mat(),r,t);
    if(!cv::checkRange(r) || !cv::checkRange(t))return fail("pnp","nonfinite_pose");
    cv::Mat rotation;cv::Rodrigues(r,rotation);
    Eigen::Matrix4d cb_T_ca=Eigen::Matrix4d::Identity();
    for(int y=0;y<3;++y){for(int x=0;x<3;++x)cb_T_ca(y,x)=rotation.at<double>(y,x);cb_T_ca(y,3)=t.at<double>(y);}
    double squared=0;std::size_t positive=0;
    for(std::size_t i=0;i<inlier_object.size();++i) {
      const auto p=inlier_object[i];
      const Eigen::Vector4d q=cb_T_ca*Eigen::Vector4d(p.x,p.y,p.z,1);
      if(q.z()>1e-6)++positive;
      if(std::abs(q.z())<1e-12)return fail("geometry","zero_projected_depth");
      const double dx=q.x()/q.z()-inlier_image[i].x,dy=q.y()/q.z()-inlier_image[i].y;
      squared+=dx*dx+dy*dy;
    }
    diag.rms_normalized=std::sqrt(squared/inlier_object.size());
    if(double(positive)/inlier_object.size()<opts.min_positive_depth_fraction)
      return fail("geometry","failed_cheirality");
    if(!std::isfinite(diag.rms_normalized) || diag.rms_normalized>opts.pnp_threshold_normalized)
      return fail("geometry","reprojection_error_too_large");
    // PnP 返回 T_Cb_Ca；反向并施加两机外参后得到约定的 T_Ba_Bb。 (Ref: 2026-10-24_Portable_Loop_Core.md)
    const Eigen::Matrix4d a_T_b=matrix(local.body_T_camera)*cb_T_ca.inverse()*matrix(remote.body_T_camera).inverse();
    fillPose(a_T_b,constraint.relative_pose);
    constraint.key_a=local.key;constraint.key_b=remote.key;
    constraint.stamp_a=local.header.stamp;constraint.stamp_b=remote.header.stamp;
    constraint.header.stamp=std::max(local.header.stamp,remote.header.stamp);
    const auto sa=identityText(ka),sb=identityText(kb);
    constraint.observation_id="loop:"+(ka<kb?sa+"|"+sb:sb+"|"+sa);
    constraint.inlier_count=inliers.rows;constraint.reprojection_error=diag.rms_normalized;
    constraint.reprojection_error_unit="normalized_camera";constraint.geometry_verified=true;
    if(opts.use_nominal_covariance) {
      for(int i=0;i<6;++i)constraint.covariance[i*6+i]=
          std::pow(i<3?opts.nominal_rotation_std:opts.nominal_translation_std,2);
      constraint.covariance_valid=true;constraint.covariance_source="configured_nominal_not_calibrated";
    } else {constraint.covariance_valid=false;constraint.covariance_source="not_estimated";}
    match_set.header=constraint.header;match_set.local_key=local.key;match_set.remote_key=remote.key;
    match_set.local_stamp=local.header.stamp;match_set.remote_stamp=remote.header.stamp;
    match_set.local_descriptor_model=local.local_descriptor_model;match_set.geometry_verified=true;
    for(int i=0;i<inliers.rows;++i) {
      auto& match=match_set.matches[inliers.at<int>(i)];match.inlier=true;
      constraint.source_observation_ids.push_back(sa+":"+std::to_string(match.local_feature_id));
      constraint.source_observation_ids.push_back(sb+":"+std::to_string(match.remote_feature_id));
    }
    diag.geometry_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-geometry_start).count();
    diag.stage="verified";diag.status="ok";
    diag.total_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-total_start).count();
    return true;
  } catch(const cv::Exception& e) {return fail("pnp",e.what());}
}
}
bool verifyLoop(const rdswarm_msgs::LocalFeaturePacket& local,
    const rdswarm_msgs::LocalFeaturePacket& remote,const LoopOptions& opts,
    rdswarm_msgs::LoopConstraint& constraint,rdswarm_msgs::VerifiedMatches& match_set,
    LoopDiagnostics& diag) {
  return verifyLoopInternal(local,remote,opts,nullptr,"bf_ratio_mutual",constraint,match_set,diag);
}
bool verifyLoopWithMatches(const rdswarm_msgs::LocalFeaturePacket& local,
    const rdswarm_msgs::LocalFeaturePacket& remote,const std::vector<ExternalMatch>& matches,
    const LoopOptions& opts,rdswarm_msgs::LoopConstraint& constraint,
    rdswarm_msgs::VerifiedMatches& match_set,LoopDiagnostics& diag) {
  return verifyLoopInternal(local,remote,opts,&matches,"lightglue_onnx_cpu",constraint,match_set,diag);
}
bool GlobalRetrievalIndex::insert(const rdswarm_msgs::KeyframeSummary& input,std::string& reason) {
  if(!frameKey(input.key).valid() || input.header.stamp.isZero() ||
     !input.global_descriptor_valid || input.global_descriptor_model.empty() ||
     input.global_descriptor.empty()){reason="global_descriptor_unavailable";return false;}
  auto s=input;double n=0;
  for(float x:s.global_descriptor){if(!std::isfinite(x)){reason="nonfinite_global_descriptor";return false;}n+=double(x)*x;}
  if(n<1e-20){reason="zero_global_descriptor";return false;}
  for(float& x:s.global_descriptor)x/=std::sqrt(n);
  const auto key=frameKey(s.key);
  const auto old=summaries_.find(key);
  if(old!=summaries_.end()) {
    if(old->second.global_descriptor_model==s.global_descriptor_model &&
       old->second.global_descriptor==s.global_descriptor){reason="already_present";return true;}
    reason="conflicting_summary";return false;
  }
  summaries_.emplace(key,std::move(s));reason="ok";return true;
}
bool GlobalRetrievalIndex::query(const FrameKey& key,std::size_t count,bool inter,
    double dt,std::vector<rdswarm_msgs::LoopCandidate>& out,std::string& reason) const {
  out.clear();const auto query=summaries_.find(key);
  if(query==summaries_.end()){reason="query_global_descriptor_missing";return false;}
  if(count==0 || !std::isfinite(dt) || dt<0){reason="invalid_query_options";return false;}
  const auto& q=query->second;
  for(const auto& item:summaries_) {
    const auto& k=item.first;const auto& s=item.second;
    if(k==key || (inter && k.agent_id==key.agent_id) ||
       s.global_descriptor_model!=q.global_descriptor_model ||
       s.global_descriptor.size()!=q.global_descriptor.size())continue;
    if(k.agent_id==key.agent_id && k.session_id==key.session_id &&
       std::abs((q.header.stamp-s.header.stamp).toSec())<dt)continue;
    rdswarm_msgs::LoopCandidate c;
    c.header=q.header;c.query_key=q.key;c.reference_key=s.key;
    c.similarity=std::inner_product(q.global_descriptor.begin(),q.global_descriptor.end(),s.global_descriptor.begin(),0.0f);
    c.retrieval_model=q.global_descriptor_model;
    c.candidate_id=identityText(key)+"->"+identityText(k);
    c.data_cutoff=std::max(q.header.stamp,s.header.stamp);out.push_back(c);
  }
  std::sort(out.begin(),out.end(),[](const auto& a,const auto& b){
    if(a.similarity!=b.similarity)return a.similarity>b.similarity;
    return a.candidate_id<b.candidate_id;
  });
  if(out.size()>count)out.resize(count);
  reason="ok";return true;
}
}  // namespace rdswarm
