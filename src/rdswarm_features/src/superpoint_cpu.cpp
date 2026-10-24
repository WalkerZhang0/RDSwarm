#include <rdswarm_features/superpoint_cpu.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <tuple>
namespace rdswarm {
bool SuperPointCpu::load(const std::string& path,std::string& reason) {
  net_=cv::dnn::Net();
  try {
    if(path.empty()) { reason="empty_model_path"; return false; }
    net_=cv::dnn::readNetFromONNX(path);
    // CPU 路径不探测或隐式启用 CUDA。 (Ref: 2026-10-24_Portable_Loop_Core.md)
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    if(net_.empty()) { reason="empty_network"; return false; }
    reason="ok"; return true;
  } catch(const cv::Exception& e) { net_=cv::dnn::Net(); reason=e.what(); return false; }
}
bool SuperPointCpu::infer(const cv::Mat& image,DenseSuperPoint& out,std::string& reason) {
  out=DenseSuperPoint();
  if(!ready() || image.empty()) { reason="model_not_ready_or_empty_image"; return false; }
  try {
    cv::Mat gray;
    if(image.channels()==3) cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
    else if(image.channels()==4) cv::cvtColor(image,gray,cv::COLOR_BGRA2GRAY);
    else if(image.channels()==1) gray=image;
    else { reason="unsupported_image_channels"; return false; }
    cv::Mat input;
    if(gray.depth()==CV_8U) gray.convertTo(input,CV_32F,1.0/255.0);
    else if(gray.depth()==CV_32F && cv::checkRange(gray,true,nullptr,0.0,1.000001)) input=gray;
    else { reason="expected_uint8_or_unit_float_image"; return false; }
    const int hp=(input.rows+7)/8*8,wp=(input.cols+7)/8*8;
    cv::copyMakeBorder(input,input,0,hp-input.rows,0,wp-input.cols,cv::BORDER_CONSTANT,0);
    net_.setInput(cv::dnn::blobFromImage(input));
    std::vector<cv::Mat> heads;
    net_.forward(heads,std::vector<std::string>{"detector_logits","descriptor_raw"});
    if(heads.size()!=2 || heads[0].dims!=4 || heads[1].dims!=4 ||
       heads[0].size[0]!=1 || heads[0].size[1]!=65 ||
       heads[1].size[0]!=1 || heads[1].size[1]!=256 ||
       heads[0].size[2]!=hp/8 || heads[0].size[3]!=wp/8 ||
       heads[1].size[2]!=hp/8 || heads[1].size[3]!=wp/8 ||
       heads[0].type()!=CV_32F || heads[1].type()!=CV_32F) {
      reason="incompatible_dense_superpoint_outputs"; return false;
    }
    out.image_size=image.size();
    out.scores=cv::Mat::zeros(image.rows,image.cols,CV_32F);
    const int hc=hp/8,wc=wp/8,area=hc*wc;
    const float* logits=heads[0].ptr<float>();
    for(int y=0;y<hc;++y) for(int x=0;x<wc;++x) {
      const int index=y*wc+x;
      float maxv=logits[index];
      for(int c=1;c<65;++c) maxv=std::max(maxv,logits[c*area+index]);
      double denominator=0;
      for(int c=0;c<65;++c) denominator+=std::exp(logits[c*area+index]-maxv);
      for(int c=0;c<64;++c) {
        const int py=y*8+c/8,px=x*8+c%8;
        if(py<image.rows && px<image.cols)
          out.scores.at<float>(py,px)=std::exp(logits[c*area+index]-maxv)/denominator;
      }
    }
    out.descriptors=heads[1].clone();
    float* desc=out.descriptors.ptr<float>();
    for(int i=0;i<area;++i) {
      double norm=0;
      for(int c=0;c<256;++c) norm+=double(desc[c*area+i])*desc[c*area+i];
      norm=std::max(std::sqrt(norm),1e-12);
      for(int c=0;c<256;++c) desc[c*area+i]/=norm;
    }
    if(!cv::checkRange(out.descriptors) || !cv::checkRange(out.scores)) {
      out=DenseSuperPoint(); reason="nonfinite_network_output"; return false;
    }
    reason="ok"; return true;
  } catch(const cv::Exception& e) { reason=e.what(); return false; }
}
bool sampleSuperPointDescriptors(const cv::Mat& dense,cv::Size size,
    const std::vector<cv::Point2f>& points,std::vector<std::vector<float>>& output,
    std::vector<unsigned char>& valid,std::string& reason) {
  output.assign(points.size(),{}); valid.assign(points.size(),0);
  if(dense.dims!=4 || dense.size[0]!=1 || dense.size[1]!=256 ||
      dense.type()!=CV_32F || !dense.isContinuous() || size.width<=0 || size.height<=0 ||
      dense.size[2]<=0 || dense.size[3]<=0 ||
      dense.size[2]*8<size.height || dense.size[3]*8<size.width) {
    reason="invalid_dense_descriptor_map"; return false;
  }
  const int h=dense.size[2],w=dense.size[3],area=h*w;
  const float* data=dense.ptr<float>();
  for(std::size_t i=0;i<points.size();++i) {
    const auto p=points[i];
    if(!std::isfinite(p.x) || !std::isfinite(p.y) ||
       p.x<0 || p.y<0 || p.x>=size.width || p.y>=size.height) continue;
    // 与原模型 grid_sample(align_corners=True,padding=zeros) 对齐。 (Ref: 2026-10-24_Portable_Loop_Core.md)
    const double x=(p.x-3.5)/(w*8.0-4.5)*(w-1);
    const double y=(p.y-3.5)/(h*8.0-4.5)*(h-1);
    const int x0=static_cast<int>(std::floor(x)),y0=static_cast<int>(std::floor(y));
    const double ax=x-x0,ay=y-y0;
    std::vector<float> d(256,0.0f);
    double norm=0;
    for(int c=0;c<256;++c) {
      double v=0;
      for(int dy=0;dy<2;++dy) for(int dx=0;dx<2;++dx) {
        const int xx=x0+dx,yy=y0+dy;
        if(xx>=0 && xx<w && yy>=0 && yy<h)
          v+=(dx?ax:1-ax)*(dy?ay:1-ay)*data[c*area+yy*w+xx];
      }
      d[c]=static_cast<float>(v); norm+=v*v;
    }
    if(!std::isfinite(norm) || norm<1e-20) continue;
    norm=std::sqrt(norm);
    for(auto& v:d) v/=norm;
    output[i]=std::move(d); valid[i]=1;
  }
  reason="ok"; return true;
}
bool SuperPointCpu::detect(const DenseSuperPoint& result,const cv::Mat& mask,
    const DetectorOptions& opts,std::vector<cv::Point2f>& points,std::string& reason) const {
  points.clear();
  if(result.scores.empty() || result.scores.type()!=CV_32F ||
     result.scores.size()!=result.image_size || !std::isfinite(opts.score_threshold) ||
     opts.score_threshold<0 || opts.score_threshold>1 ||
     opts.max_points<0 || opts.min_distance<0 || opts.border<0 ||
     (!mask.empty() && (mask.type()!=CV_8UC1 || mask.size()!=result.image_size))) {
    reason="invalid_detector_input"; return false;
  }
  struct Candidate { float score; int x,y; };
  std::vector<Candidate> candidates;
  cv::Mat available=mask.empty()?cv::Mat(result.image_size,CV_8U,cv::Scalar(255)):mask.clone();
  for(int y=opts.border;y<result.scores.rows-opts.border;++y)
    for(int x=opts.border;x<result.scores.cols-opts.border;++x) {
      const float s=result.scores.at<float>(y,x);
      if(s>=opts.score_threshold && available.at<unsigned char>(y,x))
        candidates.push_back({s,x,y});
    }
  std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b) {
    if(a.score!=b.score) return a.score>b.score;
    return std::tie(a.y,a.x)<std::tie(b.y,b.x);
  });
  for(const auto& c:candidates) {
    if(points.size()>=static_cast<std::size_t>(opts.max_points)) break;
    if(available.at<unsigned char>(c.y,c.x)) {
      points.emplace_back(c.x,c.y);
      cv::circle(available,cv::Point(c.x,c.y),opts.min_distance,cv::Scalar(0),-1);
    }
  }
  reason="ok"; return true;
}
}
