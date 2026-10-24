#pragma once
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
namespace rdswarm {
struct DenseSuperPoint {
  cv::Mat scores;
  cv::Mat descriptors;  // [1,256,H/8,W/8], channel-wise normalized.
  cv::Size image_size;
};
struct DetectorOptions {
  float score_threshold=0.02f;
  int max_points=1024,min_distance=8,border=8;
};
// 对指定 VIO 像素采样，不通过另一组检测点序号关联三维点。 (Ref: 2026-10-24_Portable_Loop_Core.md)
bool sampleSuperPointDescriptors(const cv::Mat& dense,cv::Size original_size,
    const std::vector<cv::Point2f>& pixels,std::vector<std::vector<float>>& output,
    std::vector<unsigned char>& valid,std::string& reason);
class SuperPointCpu {
 public:
  bool load(const std::string& model_path,std::string& reason);
  bool ready() const { return !net_.empty(); }
  bool infer(const cv::Mat& image,DenseSuperPoint& result,std::string& reason);
  bool detect(const DenseSuperPoint&,const cv::Mat& mask,const DetectorOptions&,
      std::vector<cv::Point2f>& points,std::string& reason) const;
 private:
  cv::dnn::Net net_;
};
}
