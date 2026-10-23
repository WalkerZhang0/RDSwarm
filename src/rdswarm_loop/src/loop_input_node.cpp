#include <rdswarm_loop/keyframe_store.hpp>
#include <rdswarm_msgs/GetKeyframe.h>
#include <ros/ros.h>

class LoopInputNode {
 public:
  explicit LoopInputNode(ros::NodeHandle& nh) {
    sub_ = nh.subscribe("keyframes", 100, &LoopInputNode::receive, this);
    service_ = nh.advertiseService("get_keyframe", &LoopInputNode::get, this);
    ROS_INFO("RDSwarm keyframe cache ready; retrieval and PnP are not implemented in this step.");
  }
 private:
  void receive(const rdswarm_msgs::KeyframeConstPtr& frame) {
    std::string reason;
    if (!store_.insert(*frame, reason))
      ROS_WARN_STREAM("Keyframe rejected: " << reason);
  }
  bool get(rdswarm_msgs::GetKeyframe::Request& req,
           rdswarm_msgs::GetKeyframe::Response& res) {
    res.success = store_.get(rdswarm::frameKey(req.key), res.keyframe);
    res.status = res.success ? "ok" : "not_found";
    if (res.success && !req.include_image) {
      res.keyframe.image = sensor_msgs::Image();
      res.keyframe.image_included = false;
    }
    if (res.success && !req.include_descriptors) {
      for (auto& f : res.keyframe.features) {
        f.descriptor.clear(); f.descriptor_valid = false;
      }
    }
    return true;
  }
  rdswarm::KeyframeStore store_;
  ros::Subscriber sub_;
  ros::ServiceServer service_;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "rdswarm_loop_inputs");
  ros::NodeHandle nh;
  LoopInputNode node(nh);
  ros::spin();
  return 0;
}
