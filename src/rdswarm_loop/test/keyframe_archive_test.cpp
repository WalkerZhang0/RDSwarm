#include <rdswarm_loop/keyframe_store.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <unistd.h>

int main() {
  const auto directory=std::filesystem::temp_directory_path()/("rdswarm_archive_"+std::to_string(getpid()));
  std::filesystem::remove_all(directory);
  rdswarm_msgs::Keyframe a;
  a.key.session_id="test/../session";a.key.frame_id=11;a.header.stamp=ros::Time(100,123);
  a.vio_frame_id="vio/test";a.initialized=a.capture_stamp_exact=true;
  a.body_pose_vio.orientation.w=a.body_T_camera.rotation.w=1;
  auto b=a;b.key.frame_id=12;b.header.stamp=ros::Time(101,456);
  rdswarm::KeyframeStore store;store.configureArchive(directory.string(),1);
  std::string reason;
  if(!store.insert(a,reason)||!store.insert(b,reason)||store.size()!=1)return 1;
  rdswarm_msgs::Keyframe read;
  if(!store.get(rdswarm::frameKey(a.key),read)||read!=a)return 2;
  rdswarm::KeyframeStore reopened;reopened.configureArchive(directory.string(),1);
  if(!reopened.get(rdswarm::frameKey(b.key),read)||read!=b)return 3;
  if(reopened.insert(a,reason)||reason!="duplicate_key")return 4;
  for(const auto& file:std::filesystem::recursive_directory_iterator(directory))
    if(file.is_regular_file()){std::ofstream corrupt(file.path(),std::ios::trunc);corrupt<<"bad";}
  if(reopened.get(rdswarm::frameKey(a.key),read))return 5;
  std::filesystem::remove_all(directory);
  std::cout<<"PASS: bounded RAM, exact archive reload/restart, duplicate rejection, corrupt archive rejection.\n";
}
