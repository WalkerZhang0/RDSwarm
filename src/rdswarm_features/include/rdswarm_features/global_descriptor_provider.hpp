#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
namespace rdswarm {
// 预留 MobileNetVLAD 等全局模型的边界，不要求核心链接推理加速库。 (Ref: 2026-10-24_Portable_Loop_Core.md)
class GlobalDescriptorProvider {
 public:
  virtual ~GlobalDescriptorProvider()=default;
  virtual bool compute(const cv::Mat&,std::vector<float>& descriptor,
      std::string& model_id,std::string& reason)=0;
};
}
