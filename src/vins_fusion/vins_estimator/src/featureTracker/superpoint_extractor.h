#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

struct SuperPointConfig
{
    int enable = 0;
    std::string backend = "tensorrt";
    std::string engine_path;
    double score_threshold = 0.02;
    int max_keypoints = 1024;
    int border = 10;
};

class SuperPointExtractor
{
public:
    SuperPointExtractor();
    ~SuperPointExtractor();

    bool init(const SuperPointConfig& config);
    bool isInitialized() const;

    bool detect(const cv::Mat& image,
                const cv::Mat& mask,
                int max_new_points,
                int min_dist,
                std::vector<cv::Point2f>& points);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
