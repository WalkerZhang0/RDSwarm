#pragma once
#include "superpoint_extractor.h"
class TensorRtDetector {
 public:
  TensorRtDetector();
  ~TensorRtDetector();
  bool init(const SuperPointConfig&);
  bool isInitialized() const;
  bool detect(const cv::Mat&,const cv::Mat&,int,int,std::vector<cv::Point2f>&);
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
