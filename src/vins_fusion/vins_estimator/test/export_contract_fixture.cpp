#include "../src/utility/rdswarm_export.h"
#include "../src/estimator/estimator.h"
#include <rdswarm_msgs/Keyframe.h>
#include <rdswarm_msgs/GetKeyframe.h>
#include <ros/ros.h>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "rdswarm_export_contract_fixture");
  ros::NodeHandle nh("/smoke/agent0/vins");
  nh.setParam("agent_id", 0);
  nh.setParam("session_id", "contract_test");
  nh.setParam("include_keyframe_image", true);
  Estimator estimator;
  initializeRdswarmExport(nh);
  rdswarm_msgs::KeyframeConstPtr received;
  auto subscriber = nh.subscribe<rdswarm_msgs::Keyframe>("keyframes", 10,
      [&](const rdswarm_msgs::KeyframeConstPtr& value) { received=value; });
  auto client = nh.serviceClient<rdswarm_msgs::GetKeyframe>("/smoke/agent0/get_keyframe");
  if (!client.waitForExistence(ros::Duration(5.0))) return 10;
  const auto deadline = ros::WallTime::now()+ros::WallDuration(5.0);
  // 等待测试订阅和缓存订阅均完成 TCP 握手，防止首帧在缓存接通前丢失。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
  while ((subscriber.getNumPublishers()==0 || rdswarmKeyframeSubscriberCount()<2) && ros::WallTime::now()<deadline) {
    ros::spinOnce(); ros::WallDuration(0.02).sleep();
  }
  if (rdswarmKeyframeSubscriberCount()<2) return 17;
  const int k=WINDOW_SIZE-2;
  const std::uint64_t capture_ns=1760000000123456789ULL;
  std_msgs::Header header;
  header.stamp.fromNSec(capture_ns);
  header.frame_id="camera0";
  for (int i=0;i<=WINDOW_SIZE;++i) {
    estimator.Rs[i]=Eigen::Matrix3d::Identity();
    estimator.Ps[i]=Eigen::Vector3d::Zero();
    estimator.Vs[i]=Eigen::Vector3d::Zero();
    estimator.Headers[i]=header.stamp.toSec();
  }
  estimator.Ps[k]=Eigen::Vector3d(0.1,0,0);
  estimator.ric[0]=Eigen::Matrix3d::Identity();
  estimator.tic[0]=Eigen::Vector3d(0.05,0,0);
  estimator.solver_flag=Estimator::NON_LINEAR;
  estimator.marginalization_flag=Estimator::MARGIN_OLD;
  FeaturePerId track(17,0);
  track.estimated_depth=2.0;
  track.solve_flag=1;
  for(int i=0;i<=k;++i) {
    Eigen::Matrix<double,7,1> observation;
    observation << (i==0?0.1:0.05),0.05,1.0,400.0,250.0,0.0,0.0;
    track.feature_per_frame.emplace_back(observation,0.0);
  }
  estimator.f_manager.feature.push_back(track);
  cv::Mat image(4,6,CV_8UC1,cv::Scalar(42));
  if(!cacheRdswarmImage(header,image)) return 11;
  publishRdswarmKeyframe(estimator);
  const auto publish_deadline=ros::WallTime::now()+ros::WallDuration(5.0);
  while(!received && ros::WallTime::now()<publish_deadline) {
    ros::spinOnce(); ros::WallDuration(0.02).sleep();
  }
  if(!received || received->header.stamp.toNSec()!=capture_ns ||
      received->key.frame_id!=capture_ns || !received->capture_stamp_exact ||
      received->features.size()!=1 || !received->image_included) return 12;
  const auto& f=received->features.front();
  // 该夹具只验证接口几何与时间戳，不代表真实 VIO 精度测试。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
  if(f.feature_id!=17 || !f.position_valid || f.descriptor_valid ||
      f.depth_variance_valid || std::abs(f.position_camera.x-0.1)>1e-10 ||
      std::abs(f.position_camera.y-0.1)>1e-10 ||
      std::abs(f.position_camera.z-2.0)>1e-10 ||
      received->image.header.stamp.toNSec()!=capture_ns) return 13;
  rdswarm_msgs::GetKeyframe service;
  service.request.key=received->key;
  service.request.include_image=false;
  service.request.include_descriptors=false;
  bool cached=false;
  const auto service_deadline=ros::WallTime::now()+ros::WallDuration(5.0);
  while(ros::WallTime::now()<service_deadline) {
    if(client.call(service) && service.response.success) { cached=true; break; }
    ros::WallDuration(0.02).sleep();
  }
  if(!cached || service.response.keyframe.image_included ||
      !service.response.keyframe.image.data.empty() ||
      service.response.keyframe.features.size()!=1) return 14;
  resetRdswarmExport();
  if(!cacheRdswarmImage(header,image)) return 15;
  received.reset();
  publishRdswarmKeyframe(estimator);
  const auto reset_deadline=ros::WallTime::now()+ros::WallDuration(5.0);
  while(!received && ros::WallTime::now()<reset_deadline) {
    ros::spinOnce(); ros::WallDuration(0.02).sleep();
  }
  if(!received || received->key.session_id!="contract_test.e1") return 16;
  ROS_INFO("PASS: exact capture ns, camera-coordinate 3D, native IDs, invalid unknown noise/descriptor, cache service, reset epoch.");
  return 0;
}
