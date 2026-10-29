#include <rdswarm_features/superpoint_cpu.hpp>
#include <rdswarm_loop/loop_core.hpp>
#include <rdswarm_loop/feature_enrichment.hpp>
#include <camodocal/camera_models/CameraFactory.h>
#include <rdswarm_msgs/DescribeKeyframe.h>
#include <rdswarm_msgs/KeyframeSummary.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <filesystem>
#include <map>
#include <chrono>

class FeatureNode {
 public:
  FeatureNode():private_("~") {
    private_.param<std::string>("model_path",model_path_,"");
    private_.param<std::string>("model_id",model_id_,"");
    private_.param("auto_describe",automatic_,false);
    std::string cache_directory;int memory_frames=0;
    private_.param<std::string>("cache_directory",cache_directory,"");
    private_.param("memory_keyframes",memory_frames,0);
    if(memory_frames<0)throw std::runtime_error("invalid_memory_keyframes");
    raw_.configureArchive(cache_directory.empty()?"":cache_directory+"/raw",memory_frames);
    ready_.configureArchive(cache_directory.empty()?"":cache_directory+"/described",memory_frames);
    private_.param<std::string>("camera_calib_file",camera_calib_file_,"");
    private_.param("append_image_features",enrichment_.append_image_features,true);
    private_.param("dedup_radius_px",enrichment_.dedup_radius_px,4.0);
    private_.param("image_feature_max_points",enrichment_.detector.max_points,400);
    private_.param("image_feature_min_distance",enrichment_.detector.min_distance,8);
    private_.param("image_feature_border",enrichment_.detector.border,8);
    private_.param("image_feature_grid_rows",enrichment_.grid_rows,16);
    private_.param("image_feature_grid_cols",enrichment_.grid_cols,12);
    double score_threshold = 0.02;
    private_.param("image_feature_score_threshold",score_threshold,score_threshold);
    enrichment_.detector.score_threshold = static_cast<float>(score_threshold);
    publisher_=nh_.advertise<rdswarm_msgs::Keyframe>("keyframes",10);
    metadata_=nh_.advertise<rdswarm_msgs::KeyframeSummary>("keyframe_metadata",10);
    subscriber_=nh_.subscribe("keyframes_in",30,&FeatureNode::receive,this);
    service_=nh_.advertiseService("describe_keyframe",&FeatureNode::describe,this);
    ROS_INFO("Feature node ready: auto_describe=%s append_image_features=%s max_points=%d grid=%dx%d dedup_radius_px=%.2f; descriptor work is explicit when false.",
             automatic_?"true":"false",enrichment_.append_image_features?"true":"false",
             enrichment_.detector.max_points,enrichment_.grid_rows,enrichment_.grid_cols,enrichment_.dedup_radius_px);
  }
 private:
  bool load(std::string& reason) {
    if (enrichment_.append_image_features && !camera_) {
      if (camera_calib_file_.empty()) { reason="camera_calib_file_required_for_image_features"; return false; }
      try { camera_=camodocal::CameraFactory::instance()->generateCameraFromYamlFile(camera_calib_file_); }
      catch(const cv::Exception& e) { reason=e.what(); return false; }
      if (!camera_) { reason="cannot_load_camera_calibration"; return false; }
    }
    if(model_.ready())return true;
    if(model_id_.empty() && !model_path_.empty()) {
      auto manifest=std::filesystem::path(model_path_);manifest.replace_extension(".json");
      if(std::filesystem::is_regular_file(manifest)) {
        try {cv::FileStorage file(manifest.string(),cv::FileStorage::READ);file["model_id"]>>model_id_;}
        catch(const cv::Exception& e){reason=e.what();return false;}
      }
    }
    if(model_id_.empty()){reason="model_id_missing_use_preparation_manifest_or_parameter";return false;}
    return model_.load(model_path_,reason);
  }
  void receive(const rdswarm_msgs::KeyframeConstPtr& frame) {
    std::string reason;
    if(!raw_.insert(*frame,reason)) {
      if(reason!="duplicate_key")ROS_WARN_STREAM("stage=raw_input status="<<reason);
      return;
    }
    ROS_INFO_STREAM("stage=raw_cached frame="<<rdswarm::identityText(rdswarm::frameKey(frame->key))
                    <<" features="<<frame->features.size());
    if(automatic_) {
      rdswarm_msgs::DescribeKeyframe::Request req;
      rdswarm_msgs::DescribeKeyframe::Response res;
      req.key=frame->key;req.publish_result=true;describe(req,res);
      if(!res.success)ROS_WARN_STREAM("stage=describe status="<<res.status);
    }
  }
  bool describe(rdswarm_msgs::DescribeKeyframe::Request& req,
                rdswarm_msgs::DescribeKeyframe::Response& res) {
    const auto key=rdswarm::frameKey(req.key);
    rdswarm_msgs::Keyframe described;
    if(!ready_.get(key,described)) {
      rdswarm_msgs::Keyframe frame;
      if(!raw_.get(key,frame)){res.status="raw_keyframe_not_found";return true;}
      if(!frame.image_included){res.status="original_image_not_cached";return true;}
      std::string reason;
      if(!load(reason)){res.status=reason;return true;}
      try {
        const cv::Mat image=cv_bridge::toCvCopy(frame.image,"mono8")->image;
        rdswarm::DenseSuperPoint dense;
        const auto sp_start=std::chrono::steady_clock::now();
        if(!model_.infer(image,dense,reason)){res.status=reason;return true;}
        const double sp_infer_ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-sp_start).count();
        rdswarm_msgs::Keyframe enriched;
        rdswarm::FeatureEnrichmentDiagnostics diagnostics;
        const auto enrich_start=std::chrono::steady_clock::now();
        if(!rdswarm::enrichKeyframe(frame,dense,model_,camera_,model_id_,enrichment_,enriched,diagnostics,reason)) {
          res.status=reason;return true;
        }
        const double enrich_ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-enrich_start).count();
        std::size_t count=0;
        for(const auto& feature:enriched.features) count+=feature.descriptor_valid;
        if(count==0){res.status="no_valid_descriptors";return true;}
        ROS_INFO_STREAM("stage=augmented original="<<diagnostics.original_features
            <<" local_3d="<<diagnostics.original_valid_3d<<" detected="<<diagnostics.detected_features
            <<" duplicate_pixels="<<diagnostics.duplicate_pixels<<" appended_2d="<<diagnostics.appended_features);
        ROS_INFO_STREAM("stage=sp_baseline frame="<<rdswarm::identityText(key)
            <<" infer_ms="<<sp_infer_ms<<" enrich_ms="<<enrich_ms
            <<" total_ms="<<(sp_infer_ms+enrich_ms)
            <<" descriptors="<<count<<" appended_2d="<<diagnostics.appended_features);
        if(!ready_.insert(enriched,reason)){res.status=reason;return true;}
        described=std::move(enriched);
      } catch(const cv_bridge::Exception& e){res.status=e.what();return true;}
    }
    const auto& frame=described;
    res.success=true;res.status="ok";res.feature_count=frame.features.size();
    res.local_descriptor_model=frame.local_descriptor_model;
    for(const auto& f:frame.features)res.valid_descriptor_count+=f.descriptor_valid;
    if(req.publish_result) {
      publisher_.publish(frame);
      rdswarm_msgs::KeyframeSummary meta;
      meta.header=frame.header;meta.key=frame.key;
      meta.image_width=frame.image.width;meta.image_height=frame.image.height;
      meta.feature_count=frame.features.size();
      for(const auto& f:frame.features)meta.geometry_available=meta.geometry_available || f.position_valid;
      meta.global_descriptor_valid=false;metadata_.publish(meta);
    }
    ROS_INFO_STREAM("stage=described frame="<<rdswarm::identityText(key)
                    <<" descriptors="<<res.valid_descriptor_count
                    <<" publish="<<static_cast<bool>(req.publish_result));
    return true;
  }
  ros::NodeHandle nh_,private_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_,metadata_;
  ros::ServiceServer service_;
  rdswarm::KeyframeStore raw_;
  rdswarm::KeyframeStore ready_;
  rdswarm::SuperPointCpu model_;
  rdswarm::FeatureEnrichmentOptions enrichment_;
  camodocal::CameraPtr camera_;
  std::string camera_calib_file_;
  std::string model_path_,model_id_;
  bool automatic_=false;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"rdswarm_features");
  FeatureNode node;ros::spin();return 0;
}
