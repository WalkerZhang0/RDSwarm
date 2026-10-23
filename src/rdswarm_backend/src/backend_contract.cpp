#include <rdswarm_backend/backend_interface.hpp>
#include <Eigen/Cholesky>

namespace rdswarm {
bool validateConstraint(const PoseConstraint& input, std::string& reason) {
  if (!input.a.valid() || !input.b.valid() || input.a == input.b ||
      input.observation_id.empty()) {
    reason = "invalid_identity"; return false;
  }
  if (!input.a_T_b.matrix().allFinite()) {
    reason = "nonfinite_transform"; return false;
  }
  // 噪声必须由调用者明确给出，不能把未知噪声替换成单位阵或零阵。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
  if (!input.covariance_valid || input.covariance_source.empty() ||
      !input.covariance.allFinite() ||
      !input.covariance.isApprox(input.covariance.transpose(), 1e-9)) {
    reason = "invalid_covariance"; return false;
  }
  Eigen::LLT<gtsam::Matrix66> llt(input.covariance);
  if (llt.info() != Eigen::Success) {
    reason = "covariance_not_positive_definite"; return false;
  }
  reason = "ok"; return true;
}
}  // namespace rdswarm
