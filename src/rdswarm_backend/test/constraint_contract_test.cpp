#include <rdswarm_backend/backend_interface.hpp>
#include <iostream>
#include <limits>

int main() {
  rdswarm::PoseConstraint c;
  c.observation_id="loop_001";
  c.a.session_id="a"; c.b.session_id="b";
  std::string reason;
  if(rdswarm::validateConstraint(c,reason) || reason!="invalid_covariance") return 1;
  c.covariance_valid=true;
  c.covariance_source="synthetic_test_only";
  c.covariance=gtsam::Matrix66::Identity()*0.01;
  if(!rdswarm::validateConstraint(c,reason)) return 2;
  c.covariance(0,0)=-1;
  if(rdswarm::validateConstraint(c,reason)) return 3;
  c.covariance=gtsam::Matrix66::Identity();
  c.covariance(0,1)=0.5;
  if(rdswarm::validateConstraint(c,reason)) return 4;
  c.covariance=gtsam::Matrix66::Identity();
  c.covariance(0,0)=std::numeric_limits<double>::quiet_NaN();
  if(rdswarm::validateConstraint(c,reason)) return 5;
  c.covariance=gtsam::Matrix66::Identity(); c.b=c.a;
  if(rdswarm::validateConstraint(c,reason)) return 6;
  std::cout<<"PASS: unknown/negative/asymmetric/nonfinite covariance and duplicate endpoints rejected.\n";
  return 0;
}
