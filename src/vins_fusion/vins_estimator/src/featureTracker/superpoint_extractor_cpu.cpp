#include "superpoint_extractor.h"
#include <rdswarm_features/superpoint_cpu.hpp>
#include <ros/ros.h>
#ifdef RDSWARM_WITH_TENSORRT
#include "superpoint_tensorrt.hpp"
#endif
class SuperPointExtractor::Impl {
 public:
  bool init(const SuperPointConfig& config) {
    config_=config;
    if(!config.enable) return false;
    if(config.backend=="tensorrt" || config.backend=="trt") {
#ifdef RDSWARM_WITH_TENSORRT
      trt_.reset(new TensorRtDetector);
      initialized_=trt_->init(config);
      return initialized_;
#else
      ROS_ERROR("TensorRT backend is disabled in this CPU build.");
      return false;
#endif
    }
    if(config.backend!="opencv") {
      ROS_ERROR("CPU build requires superpoint_backend=opencv; TensorRT requires RDSWARM_WITH_TENSORRT=ON.");
      return false;
    }
    std::string reason;
    initialized_=model_.load(config.model_path,reason);
    if(!initialized_) ROS_ERROR_STREAM("SuperPoint CPU initialization: "<<reason);
    return initialized_;
  }
  bool ready() const { return initialized_; }
  bool detect(const cv::Mat& image,const cv::Mat& mask,int count,int distance,
      std::vector<cv::Point2f>& points) {
#ifdef RDSWARM_WITH_TENSORRT
    if(trt_) return trt_->detect(image,mask,count,distance,points);
#endif
    rdswarm::DenseSuperPoint result;
    std::string reason;
    rdswarm::DetectorOptions options;
    options.score_threshold=config_.score_threshold;
    options.max_points=std::min(count,config_.max_keypoints);
    options.min_distance=distance; options.border=config_.border;
    const bool ok=initialized_ && model_.infer(image,result,reason) &&
        model_.detect(result,mask,options,points,reason);
    if(!ok) ROS_ERROR_STREAM("SuperPoint CPU detection: "<<reason);
    return ok;
  }
 private:
  SuperPointConfig config_;
  bool initialized_=false;
  rdswarm::SuperPointCpu model_;
#ifdef RDSWARM_WITH_TENSORRT
  std::unique_ptr<TensorRtDetector> trt_;
#endif
};
SuperPointExtractor::SuperPointExtractor():impl_(new Impl) {}
SuperPointExtractor::~SuperPointExtractor()=default;
bool SuperPointExtractor::init(const SuperPointConfig& c) { return impl_->init(c); }
bool SuperPointExtractor::isInitialized() const { return impl_->ready(); }
bool SuperPointExtractor::detect(const cv::Mat& i,const cv::Mat& m,int n,int d,
    std::vector<cv::Point2f>& p) { return impl_->detect(i,m,n,d,p); }
