#include <rdswarm_loop/loop_core.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <random>
namespace {
geometry_msgs::Transform transform(const Eigen::Matrix4d& m) {
  geometry_msgs::Transform t;
  t.translation.x=m(0,3);t.translation.y=m(1,3);t.translation.z=m(2,3);
  const Eigen::Quaterniond q(m.block<3,3>(0,0));
  t.rotation.x=q.x();t.rotation.y=q.y();t.rotation.z=q.z();t.rotation.w=q.w();return t;
}
rdswarm_msgs::KeyframeSummary summary(int id,int agent,float x,float y) {
  rdswarm_msgs::KeyframeSummary s;
  s.key.agent_id=agent;s.key.session_id="run";s.key.frame_id=id;
  s.header.stamp=ros::Time(100+id);s.global_descriptor_valid=true;
  s.global_descriptor_model="synthetic_retrieval_test";s.global_descriptor={x,y};return s;
}
}
int main() {
  rdswarm_msgs::LocalFeaturePacket a,b;
  a.key.agent_id=0;a.key.session_id="a";a.key.frame_id=10;
  b.key.agent_id=1;b.key.session_id="b";b.key.frame_id=20;
  a.header.stamp=ros::Time(100.03);b.header.stamp=ros::Time(80.01);
  a.local_descriptor_model=b.local_descriptor_model="synthetic_geometry_test";
  a.descriptor_dimension=b.descriptor_dimension=64;
  Eigen::Matrix4d truth=Eigen::Matrix4d::Identity(),ta=truth,tb=truth;
  truth.block<3,3>(0,0)=(Eigen::AngleAxisd(0.2,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(-0.1,Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(0.08,Eigen::Vector3d::UnitX())).toRotationMatrix();
  truth.block<3,1>(0,3)=Eigen::Vector3d(0.4,-0.2,0.1);
  ta.block<3,3>(0,0)=Eigen::AngleAxisd(0.12,Eigen::Vector3d::UnitY()).toRotationMatrix();
  ta.block<3,1>(0,3)=Eigen::Vector3d(0.05,0.01,0);
  tb.block<3,3>(0,0)=Eigen::AngleAxisd(-0.05,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  tb.block<3,1>(0,3)=Eigen::Vector3d(-0.02,0.03,0.01);
  a.body_T_camera=transform(ta);b.body_T_camera=transform(tb);
  std::mt19937 rng(11);std::uniform_real_distribution<double> xy(-1,1),z(4,8);
  for(int i=0;i<40;++i) {
    rdswarm_msgs::TrackedFeature f,g;
    f.feature_id=i;g.feature_id=1000+i;
    f.position_valid=true;f.position_camera.x=xy(rng);f.position_camera.y=xy(rng);f.position_camera.z=z(rng);
    const Eigen::Vector4d p(f.position_camera.x,f.position_camera.y,f.position_camera.z,1);
    const Eigen::Vector4d q=tb.inverse()*truth.inverse()*ta*p;
    f.normalized.x=p.x()/p.z();f.normalized.y=p.y()/p.z();f.normalized.z=1;
    g.normalized.x=q.x()/q.z();g.normalized.y=q.y()/q.z();g.normalized.z=1;
    if(i<8){g.normalized.x=xy(rng);g.normalized.y=xy(rng);}
    f.descriptor.assign(64,0);f.descriptor[i]=1;f.descriptor_valid=true;
    g.descriptor=f.descriptor;g.descriptor_valid=true;
    a.features.push_back(f);b.features.push_back(g);
  }
  rdswarm::LoopOptions opts;rdswarm::LoopDiagnostics diag;
  rdswarm_msgs::LoopConstraint c;rdswarm_msgs::VerifiedMatches matches;
  if(!rdswarm::verifyLoop(a,b,opts,c,matches,diag)){std::cerr<<diag.stage<<":"<<diag.status<<"\n";return 1;}
  const Eigen::Quaterniond estimated(c.relative_pose.orientation.w,c.relative_pose.orientation.x,
      c.relative_pose.orientation.y,c.relative_pose.orientation.z);
  const Eigen::Vector3d t(c.relative_pose.position.x,c.relative_pose.position.y,c.relative_pose.position.z);
  if((t-truth.block<3,1>(0,3)).norm()>0.002 ||
     Eigen::AngleAxisd(estimated.toRotationMatrix()*truth.block<3,3>(0,0).transpose()).angle()>0.002 ||
     c.inlier_count<30 || c.covariance_valid || !c.geometry_verified || !matches.geometry_verified)return 2;
  auto wrong=b;wrong.local_descriptor_model="other_weights";
  if(rdswarm::verifyLoop(a,wrong,opts,c,matches,diag) || diag.status!="descriptor_model_mismatch")return 3;
  auto line=a;
  for(std::size_t i=0;i<line.features.size();++i) {
    line.features[i].position_camera.x=i*0.1;
    line.features[i].position_camera.y=0;line.features[i].position_camera.z=5;
  }
  if(rdswarm::verifyLoop(line,b,opts,c,matches,diag) || diag.status!="collinear_or_tiny_3d_geometry")return 4;
  auto missing=b;
  for(auto& f:missing.features)f.descriptor_valid=false;
  if(rdswarm::verifyLoop(a,missing,opts,c,matches,diag))return 5;
  opts.use_nominal_covariance=true;
  if(!rdswarm::verifyLoop(a,b,opts,c,matches,diag) ||
     !c.covariance_valid || c.covariance_source!="configured_nominal_not_calibrated")return 6;
  rdswarm::GlobalRetrievalIndex index;std::string reason;
  const auto query=summary(0,0,1,0),best=summary(1,1,0.9,0.1),bad=summary(2,2,-1,0);
  if(!index.insert(query,reason) || !index.insert(best,reason) || !index.insert(bad,reason))return 7;
  std::vector<rdswarm_msgs::LoopCandidate> candidates;
  if(!index.query(rdswarm::frameKey(query.key),1,true,5,candidates,reason) ||
     candidates.size()!=1 || candidates[0].reference_key.agent_id!=1)return 8;
  auto empty=summary(3,3,0,0);
  if(index.insert(empty,reason))return 9;
  std::cout<<"PASS: normalized retrieval, reciprocal descriptor matching, outlier RANSAC, "
      "nontrivial extrinsics and T_Ba_Bb direction, invalid model/geometry rejection, covariance provenance.\n";
}
