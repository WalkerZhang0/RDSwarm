#pragma once
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <opencv2/core.hpp>

class Estimator;
void initializeRdswarmExport(ros::NodeHandle& private_node);
bool cacheRdswarmImage(const std_msgs::Header&, const cv::Mat& mono_image);
void resetRdswarmExport();
void publishRdswarmKeyframe(const Estimator&);
void publishRdswarmState(const Estimator&, const std_msgs::Header&);
std::size_t rdswarmKeyframeSubscriberCount();
