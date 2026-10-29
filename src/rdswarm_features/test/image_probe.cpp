#include <rdswarm_features/superpoint_cpu.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 8) {
    std::cerr << "Usage: superpoint_image_probe model.onnx image output.yml score_threshold max_points min_distance border\n";
    return 2;
  }
  try {
    const cv::Mat image = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    if (image.empty()) throw std::runtime_error("cannot_read_image");
    rdswarm::DetectorOptions options;
    options.score_threshold = std::stof(argv[4]);
    options.max_points = std::stoi(argv[5]);
    options.min_distance = std::stoi(argv[6]);
    options.border = std::stoi(argv[7]);
    if (options.max_points <= 0) throw std::runtime_error("max_points_must_be_positive");
    rdswarm::SuperPointCpu model;
    rdswarm::DenseSuperPoint dense;
    std::string reason;
    if (!model.load(argv[1], reason)) throw std::runtime_error(reason);
    const auto start = std::chrono::steady_clock::now();
    if (!model.infer(image, dense, reason)) throw std::runtime_error(reason);
    std::vector<cv::Point2f> points;
    // 检测点只用于图像匹配；不把它们按序号拼接到 VIO 三维点。 (Ref: 2026-10-25_Diagnose_Two_Image_Matching.md)
    if (!model.detect(dense, cv::Mat(), options, points, reason)) throw std::runtime_error(reason);
    std::vector<std::vector<float>> descriptors;
    std::vector<unsigned char> valid;
    if (!rdswarm::sampleSuperPointDescriptors(dense.descriptors, image.size(), points,
                                            descriptors, valid, reason)) throw std::runtime_error(reason);
    cv::Mat pixels(static_cast<int>(points.size()), 2, CV_32F);
    cv::Mat scores(static_cast<int>(points.size()), 1, CV_32F);
    cv::Mat rows(static_cast<int>(points.size()), 256, CV_32F, cv::Scalar(0));
    cv::Mat mask(static_cast<int>(points.size()), 1, CV_8U);
    for (std::size_t i = 0; i < points.size(); ++i) {
      pixels.at<float>(i, 0) = points[i].x;
      pixels.at<float>(i, 1) = points[i].y;
      scores.at<float>(i) = dense.scores.at<float>(static_cast<int>(points[i].y), static_cast<int>(points[i].x));
      mask.at<unsigned char>(i) = valid[i];
      if (valid[i]) std::copy(descriptors[i].begin(), descriptors[i].end(), rows.ptr<float>(i));
    }
    const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    cv::FileStorage out(argv[3], cv::FileStorage::WRITE);
    if (!out.isOpened()) throw std::runtime_error("cannot_write_output");
    out << "model_path" << std::filesystem::absolute(argv[1]).string()
        << "image_path" << std::filesystem::absolute(argv[2]).string()
        << "image_width" << image.cols << "image_height" << image.rows
        << "score_threshold" << options.score_threshold << "max_points" << options.max_points
        << "min_distance" << options.min_distance << "border" << options.border
        << "points" << pixels << "scores" << scores << "descriptors" << rows
        << "valid" << mask << "inference_detection_sampling_ms" << elapsed;
    std::cout << "detected=" << points.size() << " elapsed_ms=" << elapsed << "\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 2;
  }
}
