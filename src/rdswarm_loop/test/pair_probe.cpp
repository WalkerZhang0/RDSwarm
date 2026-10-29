#include <rdswarm_loop/loop_core.hpp>
#include <ros/serialization.h>
#include <opencv2/core.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <sstream>
#include <vector>

namespace {
rdswarm_msgs::LocalFeaturePacket readPacket(const char* path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error(std::string("cannot_open_packet:") + path);
  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)),
                                std::istreambuf_iterator<char>());
  if (bytes.empty() || bytes.size() > 64 * 1024 * 1024)
    throw std::runtime_error("empty_or_oversized_packet");
  ros::serialization::IStream stream(bytes.data(), bytes.size());
  rdswarm_msgs::LocalFeaturePacket packet;
  ros::serialization::deserialize(stream, packet);
  if (stream.getLength() != 0) throw std::runtime_error("trailing_packet_bytes");
  return packet;
}

std::vector<rdswarm::ExternalMatch> readMatches(const char* path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error(std::string("cannot_open_matches:") + path);
  std::vector<rdswarm::ExternalMatch> matches;
  std::string line;
  while (std::getline(input,line)) {
    if (line.empty() || line.front()=='#') continue;
    std::istringstream row(line);
    rdswarm::ExternalMatch match;
    if (!(row>>match.local_feature_index>>match.remote_feature_index>>match.confidence))
      throw std::runtime_error("invalid_match_row");
    matches.push_back(match);
  }
  return matches;
}
}

int main(int argc, char** argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "Usage: loop_pair_probe local.bin peer.bin result.json [matches.txt]\n";
    return 2;
  }
  try {
    const auto local = readPacket(argv[1]);
    const auto remote = readPacket(argv[2]);
    rdswarm_msgs::LoopConstraint constraint;
    rdswarm_msgs::VerifiedMatches matches;
    rdswarm::LoopDiagnostics diagnostics;
    // 直接调用在线服务使用的同一核心，不启动 ROS master，也不改变默认几何门限。 (Ref: 2026-10-25_Verify_S3E_InterAgent_Loop.md)
    const auto external = argc==5 ? readMatches(argv[4]) : std::vector<rdswarm::ExternalMatch>();
    const bool success = argc==5
        ? rdswarm::verifyLoopWithMatches(local, remote, external, rdswarm::LoopOptions(),
                                         constraint, matches, diagnostics)
        : rdswarm::verifyLoop(local, remote, rdswarm::LoopOptions(),
                              constraint, matches, diagnostics);
    cv::FileStorage out(argv[3], cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if (!out.isOpened()) throw std::runtime_error("cannot_open_result_file");
    out << "success" << static_cast<int>(success)
        << "stage" << diagnostics.stage << "status" << diagnostics.status
        << "local_frame_id" << std::to_string(local.key.frame_id)
        << "remote_frame_id" << std::to_string(remote.key.frame_id)
        << "local_features" << static_cast<int>(diagnostics.local_features)
        << "remote_features" << static_cast<int>(diagnostics.remote_features)
        << "matches" << static_cast<int>(diagnostics.matches)
        << "inliers" << static_cast<int>(diagnostics.inliers)
        << "matcher" << diagnostics.matcher
        << "matching_ms" << diagnostics.matching_ms
        << "geometry_ms" << diagnostics.geometry_ms
        << "total_ms" << diagnostics.total_ms
        << "rms_normalized" << diagnostics.rms_normalized
        << "covariance_valid" << static_cast<int>(constraint.covariance_valid);
    if (success) {
      const auto& p = constraint.relative_pose.position;
      const auto& q = constraint.relative_pose.orientation;
      out << "translation" << "[" << p.x << p.y << p.z << "]"
          << "quaternion_xyzw" << "[" << q.x << q.y << q.z << q.w << "]";
    }
    out << "correspondences" << "[";
    for (const auto& match : matches.matches)
      out << "{" << "local_feature_id" << std::to_string(match.local_feature_id)
          << "remote_feature_id" << std::to_string(match.remote_feature_id)
          << "descriptor_distance" << match.descriptor_distance
          << "inlier" << static_cast<int>(match.inlier) << "}";
    out << "]";
    std::cout << diagnostics.stage << ":" << diagnostics.status
              << " matcher=" << diagnostics.matcher
              << " matches=" << diagnostics.matches << " inliers=" << diagnostics.inliers
              << " matching_ms=" << diagnostics.matching_ms
              << " geometry_ms=" << diagnostics.geometry_ms
              << " total_ms=" << diagnostics.total_ms
              << " rms_normalized=" << diagnostics.rms_normalized << "\n";
    return success ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 2;
  }
}
