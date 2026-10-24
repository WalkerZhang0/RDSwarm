#include <rdswarm_features/superpoint_cpu.hpp>
#include <cmath>
#include <iostream>
#include <limits>
int main() {
  int sizes[]={1,256,2,2};
  cv::Mat map(4,sizes,CV_32F,cv::Scalar(0));
  for(int i=0;i<4;++i)map.ptr<float>()[i*4+i]=1;
  std::vector<cv::Point2f> pts{{3.5f,3.5f},{9.25f,9.25f},{15,15},{16,2},
      {std::numeric_limits<float>::quiet_NaN(),1}};
  std::vector<std::vector<float>> desc;
  std::vector<unsigned char> valid;std::string reason;
  if(!rdswarm::sampleSuperPointDescriptors(map,{16,16},pts,desc,valid,reason))return 1;
  if(valid.size()!=5 || !valid[0] || !valid[1] || !valid[2] || valid[3] || valid[4])return 2;
  if(std::abs(desc[0][0]-1)>1e-6 || std::abs(desc[2][3]-1)>1e-6)return 3;
  for(int i=0;i<4;++i)if(std::abs(desc[1][i]-0.5)>1e-6)return 4;
  map.setTo(0);
  if(!rdswarm::sampleSuperPointDescriptors(map,{16,16},pts,desc,valid,reason) || valid[0])return 5;
  std::cout<<"PASS: subpixel bilinear geometry, normalization, invalid pixels and zero maps.\n";
}
