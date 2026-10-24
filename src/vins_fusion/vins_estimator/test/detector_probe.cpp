#include "../src/featureTracker/superpoint_extractor.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=4){std::cerr<<"Usage: detector_probe opencv|tensorrt model_or_engine image\n";return 1;}
  SuperPointConfig c;c.enable=1;c.backend=argv[1];
  if(c.backend=="opencv")c.model_path=argv[2];else c.engine_path=argv[2];
  SuperPointExtractor detector;
  if(!detector.init(c))return 2;
  std::vector<cv::Point2f> points;
  const cv::Mat image=cv::imread(argv[3],cv::IMREAD_GRAYSCALE);
  if(!detector.detect(image,cv::Mat(),128,8,points))return 3;
  std::cout<<"PASS: selected backend "<<c.backend<<", detected "<<points.size()<<" points.\n";
}
