#include <rdswarm_loop/loop_core.hpp>
#include <rdswarm_msgs/GetLocalFeatures.h>
#include <rdswarm_msgs/FetchPeerFeatures.h>
#include <rdswarm_msgs/QueryLoopCandidates.h>
#include <rdswarm_msgs/VerifyLoop.h>
#include <ros/ros.h>
#include <ros/names.h>
#include <chrono>

class LoopNode {
 public:
  LoopNode():private_("~") {
    frames_sub_=nh_.subscribe("keyframes",30,&LoopNode::frame,this);
    summaries_sub_=nh_.subscribe("summaries",100,&LoopNode::summary,this);
    get_=nh_.advertiseService("get_local_features",&LoopNode::get,this);
    fetch_=nh_.advertiseService("fetch_peer_features",&LoopNode::fetch,this);
    query_=nh_.advertiseService("query_candidates",&LoopNode::query,this);
    verify_=nh_.advertiseService("verify_loop",&LoopNode::verify,this);
    loops_=nh_.advertise<rdswarm_msgs::LoopConstraint>("loop_constraints",10);
    matches_=nh_.advertise<rdswarm_msgs::VerifiedMatches>("verified_matches",10);
    ROS_INFO("Loop core ready. Query, peer fetch and PnP are separate explicit services; no automatic search.");
  }
 private:
  void frame(const rdswarm_msgs::KeyframeConstPtr& f) {
    std::string reason;
    if(!frames_.insert(*f,reason) && reason!="duplicate_key")
      ROS_WARN_STREAM("stage=enriched_input status="<<reason);
  }
  void summary(const rdswarm_msgs::KeyframeSummaryConstPtr& s) {
    std::string reason;
    if(!index_.insert(*s,reason))
      ROS_WARN_STREAM("stage=summary status="<<reason<<" frame="<<s->key.frame_id);
  }
  bool get(rdswarm_msgs::GetLocalFeatures::Request& req,
           rdswarm_msgs::GetLocalFeatures::Response& res) {
    rdswarm_msgs::Keyframe f;
    if(!frames_.get(rdswarm::frameKey(req.key),f)){res.status="keyframe_not_found";return true;}
    res.packet=rdswarm::localPacket(f,req.include_3d);
    res.success=rdswarm::validatePacket(res.packet,res.status);
    ROS_INFO_STREAM("stage=serve_features frame="<<req.key.frame_id
                    <<" count="<<res.packet.features.size()<<" include_3d="<<req.include_3d);
    return true;
  }
  bool fetch(rdswarm_msgs::FetchPeerFeatures::Request& req,
             rdswarm_msgs::FetchPeerFeatures::Response& res) {
    if(req.service_name.empty() || req.service_name.front()!='/') {
      res.status="absolute_peer_service_name_required";return true;
    }
    std::string name_error;
    if(!ros::names::validate(req.service_name,name_error)) {
      res.status="invalid_peer_service_name";return true;
    }
    // 此服务只请求并缓存对端特征，不隐式启动匹配和 PnP。 (Ref: 2026-10-24_Portable_Loop_Core.md)
    auto client=nh_.serviceClient<rdswarm_msgs::GetLocalFeatures>(req.service_name);
    if(!client.exists()){res.status="peer_service_unavailable";return true;}
    rdswarm_msgs::GetLocalFeatures call;
    call.request.key=req.key;call.request.include_3d=req.include_3d;
    if(!client.call(call)){res.status="peer_call_failed";return true;}
    if(!call.response.success){res.status=call.response.status;return true;}
    if(!(rdswarm::frameKey(call.response.packet.key)==rdswarm::frameKey(req.key)) ||
       !rdswarm::validatePacket(call.response.packet,res.status)) {
      res.status="peer_identity_or_payload_invalid";return true;
    }
    remote_[rdswarm::frameKey(req.key)]=call.response.packet;
    res.success=true;res.status="ok";res.feature_count=call.response.packet.features.size();
    res.local_descriptor_model=call.response.packet.local_descriptor_model;
    ROS_INFO_STREAM("stage=peer_cached frame="<<req.key.frame_id<<" count="<<res.feature_count);
    return true;
  }
  bool query(rdswarm_msgs::QueryLoopCandidates::Request& req,
             rdswarm_msgs::QueryLoopCandidates::Response& res) {
    res.success=index_.query(rdswarm::frameKey(req.query_key),req.max_candidates,
        req.inter_agent_only,req.min_time_separation,res.candidates,res.status);
    const ros::Time decision_time=ros::Time::now();
    for(auto& candidate:res.candidates) {
      candidate.header.stamp=decision_time;
      candidate.data_cutoff=decision_time;
    }
    ROS_INFO_STREAM("stage=query status="<<res.status<<" candidates="<<res.candidates.size());
    return true;
  }
  rdswarm::LoopOptions options() {
    rdswarm::LoopOptions o;
    private_.param("matching/ratio_threshold",o.ratio_threshold,o.ratio_threshold);
    private_.param("matching/max_descriptor_distance",o.max_descriptor_distance,o.max_descriptor_distance);
    private_.param("geometry/min_matches",o.min_matches,o.min_matches);
    private_.param("geometry/min_inliers",o.min_inliers,o.min_inliers);
    private_.param("geometry/min_inlier_ratio",o.min_inlier_ratio,o.min_inlier_ratio);
    private_.param("geometry/ransac_iterations",o.ransac_iterations,o.ransac_iterations);
    private_.param("geometry/pnp_threshold_normalized",o.pnp_threshold_normalized,o.pnp_threshold_normalized);
    private_.param("geometry/min_second_singular_ratio",o.min_second_singular_ratio,o.min_second_singular_ratio);
    private_.param("geometry/min_positive_depth_fraction",o.min_positive_depth_fraction,o.min_positive_depth_fraction);
    private_.param("geometry/ransac_seed",o.ransac_seed,o.ransac_seed);
    private_.param("covariance/use_nominal",o.use_nominal_covariance,o.use_nominal_covariance);
    private_.param("covariance/rotation_std",o.nominal_rotation_std,o.nominal_rotation_std);
    private_.param("covariance/translation_std",o.nominal_translation_std,o.nominal_translation_std);
    return o;
  }
  bool verify(rdswarm_msgs::VerifyLoop::Request& req,
              rdswarm_msgs::VerifyLoop::Response& res) {
    rdswarm_msgs::Keyframe local;
    if(!frames_.get(rdswarm::frameKey(req.local_key),local)){res.status="local_keyframe_missing";return true;}
    const auto peer=remote_.find(rdswarm::frameKey(req.remote_key));
    if(peer==remote_.end()){res.status="peer_features_missing_call_fetch_first";return true;}
    const auto start=std::chrono::steady_clock::now();
    rdswarm::LoopDiagnostics diagnostics;
    res.success=rdswarm::verifyLoop(rdswarm::localPacket(local,true),peer->second,options(),
        res.constraint,res.correspondences,diagnostics);
    res.status=diagnostics.stage+":"+diagnostics.status;
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    if(res.success) {
      matches_.publish(res.correspondences);
      if(req.publish_constraint)loops_.publish(res.constraint);
    }
    ROS_INFO_STREAM("stage="<<diagnostics.stage<<" status="<<diagnostics.status
      <<" matches="<<diagnostics.matches<<" inliers="<<diagnostics.inliers
      <<" rms_normalized="<<diagnostics.rms_normalized<<" elapsed_ms="<<ms
      <<" publish_constraint="<<(res.success && req.publish_constraint));
    return true;
  }
  ros::NodeHandle nh_,private_;
  ros::Subscriber frames_sub_,summaries_sub_;
  ros::ServiceServer get_,fetch_,query_,verify_;
  ros::Publisher loops_,matches_;
  rdswarm::KeyframeStore frames_;
  std::map<rdswarm::FrameKey,rdswarm_msgs::LocalFeaturePacket> remote_;
  rdswarm::GlobalRetrievalIndex index_;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"rdswarm_loop");
  LoopNode node;ros::spin();return 0;
}
