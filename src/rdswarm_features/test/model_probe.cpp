#include <rdswarm_features/superpoint_cpu.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=5){std::cerr<<"Usage: model_probe dense.onnx image output.yml points.yml\n";return 1;}
  rdswarm::SuperPointCpu model;rdswarm::DenseSuperPoint dense;std::string reason;
  cv::FileStorage source(argv[4],cv::FileStorage::READ);cv::Mat input;source["points"]>>input;
  if(input.type()!=CV_32F || input.cols!=2)return 2;
  std::vector<cv::Point2f> points;
  for(int i=0;i<input.rows;++i)points.emplace_back(input.at<float>(i,0),input.at<float>(i,1));
  if(!model.load(argv[1],reason) || !model.infer(cv::imread(argv[2],cv::IMREAD_GRAYSCALE),dense,reason)) {
    std::cerr<<reason<<"\n";return 3;
  }
  std::vector<std::vector<float>> descriptors;std::vector<unsigned char> valid;
  if(!rdswarm::sampleSuperPointDescriptors(dense.descriptors,dense.image_size,points,descriptors,valid,reason))return 4;
  cv::Mat output(points.size(),256,CV_32F,cv::Scalar(0)),mask(points.size(),1,CV_8U);
  for(std::size_t i=0;i<points.size();++i) {
    mask.at<unsigned char>(i)=valid[i];
    if(valid[i])std::copy(descriptors[i].begin(),descriptors[i].end(),output.ptr<float>(i));
  }
  cv::FileStorage sink(argv[3],cv::FileStorage::WRITE);
  sink<<"points"<<input<<"descriptors"<<output<<"valid"<<mask;
  std::vector<cv::Point2f> detected;
  rdswarm::DetectorOptions opts;
  if(!model.detect(dense,cv::Mat(),opts,detected,reason))return 5;
  std::cout<<"PASS: OpenCV CPU inference, "<<points.size()<<" requested samples, "
           <<detected.size()<<" detected points.\n";
}
