#include <rdswarm_features/superpoint_cpu.hpp>
#include <rdswarm_loop/loop_core.hpp>
#include <rdswarm_msgs/DescribeKeyframe.h>
#include <rdswarm_msgs/KeyframeSummary.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <filesystem>
#include <map>

class FeatureNode {
 public:
  FeatureNode():private_("~") {
    private_.param<std::string>("model_path",model_path_,"");
    private_.param<std::string>("model_id",model_id_,"");
    private_.param("auto_describe",automatic_,false);
    publisher_=nh_.advertise<rdswarm_msgs::Keyframe>("keyframes",10);
    metadata_=nh_.advertise<rdswarm_msgs::KeyframeSummary>("keyframe_metadata",10);
    subscriber_=nh_.subscribe("keyframes_in",30,&FeatureNode::receive,this);
    service_=nh_.advertiseService("describe_keyframe",&FeatureNode::describe,this);
    ROS_INFO("Feature node ready: auto_describe=%s; descriptor work is explicit when false.",automatic_?"true":"false");
  }
 private:
  bool load(std::string& reason) {
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
    auto ready=ready_.find(key);
    if(ready==ready_.end()) {
      rdswarm_msgs::Keyframe frame;
      if(!raw_.get(key,frame)){res.status="raw_keyframe_not_found";return true;}
      if(!frame.image_included){res.status="original_image_not_cached";return true;}
      std::string reason;
      if(!load(reason)){res.status=reason;return true;}
      try {
        const cv::Mat image=cv_bridge::toCvCopy(frame.image,"mono8")->image;
        rdswarm::DenseSuperPoint dense;
        if(!model_.infer(image,dense,reason)){res.status=reason;return true;}
        std::vector<cv::Point2f> points;
        for(const auto& f:frame.features)points.emplace_back(f.pixel.x,f.pixel.y);
        std::vector<std::vector<float>> desc;
        std::vector<unsigned char> valid;
        if(!rdswarm::sampleSuperPointDescriptors(dense.descriptors,image.size(),points,desc,valid,reason)) {
          res.status=reason;return true;
        }
        // 原 feature_id、二维及三维位置保持不变，只补充对应像素的描述子。 (Ref: 2026-10-24_Portable_Loop_Core.md)
        frame.local_descriptor_model=model_id_;
        std::size_t count=0;
        for(std::size_t i=0;i<frame.features.size();++i) {
          frame.features[i].descriptor=std::move(desc[i]);
          frame.features[i].descriptor_valid=valid[i]!=0;
          count+=valid[i]!=0;
        }
        if(count==0){res.status="no_valid_descriptors";return true;}
        ready=ready_.emplace(key,std::move(frame)).first;
      } catch(const cv_bridge::Exception& e){res.status=e.what();return true;}
    }
    const auto& frame=ready->second;
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
                    <<" publish="<<req.publish_result);
    return true;
  }
  ros::NodeHandle nh_,private_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_,metadata_;
  ros::ServiceServer service_;
  rdswarm::KeyframeStore raw_;
  std::map<rdswarm::FrameKey,rdswarm_msgs::Keyframe> ready_;
  rdswarm::SuperPointCpu model_;
  std::string model_path_,model_id_;
  bool automatic_=false;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"rdswarm_features");
  FeatureNode node;ros::spin();return 0;
}
