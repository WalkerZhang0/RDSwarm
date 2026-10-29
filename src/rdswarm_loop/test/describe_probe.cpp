#include <rdswarm_loop/feature_enrichment.hpp>
#include <camodocal/camera_models/CameraFactory.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/serialization.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <chrono>

int main(int argc, char** argv) {
  if (argc!=7 && argc!=8) {
    std::cerr<<"Usage: keyframe_describe_probe raw.bin model.onnx camera.yaml enriched.bin report.json append_2d(0|1) [max_points]\n";
    return 2;
  }
  try {
    std::ifstream input(argv[1],std::ios::binary);
    if(!input)throw std::runtime_error("cannot_open_raw_keyframe");
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)),std::istreambuf_iterator<char>());
    if(bytes.empty() || bytes.size()>64*1024*1024)throw std::runtime_error("empty_or_oversized_raw_keyframe");
    ros::serialization::IStream stream(bytes.data(),bytes.size());
    rdswarm_msgs::Keyframe raw;
    ros::serialization::deserialize(stream,raw);
    if(stream.getLength()!=0 || !raw.image_included)throw std::runtime_error("invalid_raw_keyframe_image_or_trailing_bytes");
    auto manifest=std::filesystem::path(argv[2]);manifest.replace_extension(".json");
    cv::FileStorage identity(manifest.string(),cv::FileStorage::READ);
    std::string model_id;identity["model_id"]>>model_id;
    rdswarm::FeatureEnrichmentOptions options;
    if(argc==8)options.detector.max_points=std::stoi(argv[7]);
    const std::string append=argv[6];
    if(append!="0" && append!="1")throw std::runtime_error("append_must_be_zero_or_one");
    options.append_image_features=append=="1";
    camodocal::CameraPtr camera;
    if(options.append_image_features)camera=camodocal::CameraFactory::instance()->generateCameraFromYamlFile(argv[3]);
    const cv::Mat image=cv_bridge::toCvCopy(raw.image,"mono8")->image;
    rdswarm::SuperPointCpu model;rdswarm::DenseSuperPoint dense;std::string reason;
    const auto load_start=std::chrono::steady_clock::now();
    if(!model.load(argv[2],reason))throw std::runtime_error(reason);
    const auto infer_start=std::chrono::steady_clock::now();
    if(!model.infer(image,dense,reason))throw std::runtime_error(reason);
    const auto enrich_start=std::chrono::steady_clock::now();
    rdswarm_msgs::Keyframe result;rdswarm::FeatureEnrichmentDiagnostics diagnostics;
    // 使用与 describe_keyframe 服务相同的增补函数，离线回归保留完整原始观测。 (Ref: 2026-10-25_Extend_Peer_2D_Features.md)
    if(!rdswarm::enrichKeyframe(raw,dense,model,camera,model_id,options,result,diagnostics,reason))throw std::runtime_error(reason);
    const auto end=std::chrono::steady_clock::now();
    bytes.resize(ros::serialization::serializationLength(result));
    ros::serialization::OStream sink(bytes.data(),bytes.size());ros::serialization::serialize(sink,result);
    std::ofstream output(argv[4],std::ios::binary);
    if(!output)throw std::runtime_error("cannot_write_enriched_keyframe");
    output.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
    if(!output)throw std::runtime_error("enriched_keyframe_write_failed");
    cv::FileStorage report(argv[5],cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    if(!report.isOpened())throw std::runtime_error("cannot_write_report");
    report<<"sp_model_load_ms"<<std::chrono::duration<double,std::milli>(infer_start-load_start).count()
          <<"sp_infer_ms"<<std::chrono::duration<double,std::milli>(enrich_start-infer_start).count()
          <<"sp_enrich_ms"<<std::chrono::duration<double,std::milli>(end-enrich_start).count()
          <<"max_points"<<options.detector.max_points<<"grid_rows"<<options.grid_rows<<"grid_cols"<<options.grid_cols
          <<"original_features"<<static_cast<int>(diagnostics.original_features)
          <<"original_valid_3d"<<static_cast<int>(diagnostics.original_valid_3d)
          <<"detected_features"<<static_cast<int>(diagnostics.detected_features)
          <<"duplicate_pixels"<<static_cast<int>(diagnostics.duplicate_pixels)
          <<"appended_features"<<static_cast<int>(diagnostics.appended_features)
          <<"total_features"<<static_cast<int>(result.features.size());
    std::cout<<"original="<<diagnostics.original_features<<" 3d="<<diagnostics.original_valid_3d
             <<" appended_2d="<<diagnostics.appended_features<<" total="<<result.features.size()<<"\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<"\n";return 2;
  }
}
