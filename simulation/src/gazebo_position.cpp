#include "simulation/gazebo_position.hpp"

#include "control/frames.hpp"

#include <cmath>
#include <cstddef>

namespace simulation {
namespace {

bool validTimestamp(double timestamp_s) {
  return std::isfinite(timestamp_s) && timestamp_s >= 0.0;
}

}  // namespace

GazeboPoseSelectionResult selectGazeboModelPose(const GazeboPoseVectorSample &sample,
                                                std::string_view world_frame,
                                                std::string_view model_name) {
  GazeboPoseSelectionResult result{};
  if (world_frame.empty() || model_name.empty()) {
    result.reason = "invalid Gazebo identity configuration";
    return result;
  }
  if (!validTimestamp(sample.timestamp_s)) {
    result.reason = "invalid Gazebo simulation timestamp";
    return result;
  }

  const GazeboNamedPose *match = nullptr;
  std::size_t match_index = 0;
  std::size_t match_count = 0;
  for (std::size_t index = 0; index < sample.poses.size(); ++index) {
    const auto &pose = sample.poses[index];
    if (pose.name == model_name) {
      match = &pose;
      match_index = index;
      ++match_count;
    }
  }

  if (match_count == 0) {
    result.reason = "Gazebo model pose not found";
    return result;
  }
  if (match_count != 1) {
    result.reason = "Gazebo model pose is ambiguous";
    return result;
  }
  if (match == nullptr || !match->position_enu_m.finite()) {
    result.reason = "invalid Gazebo model position";
    return result;
  }

  result.source_pose_index = match_index;
  result.pose.frame_id = std::string(world_frame);
  result.pose.child_frame_id = std::string(model_name);
  result.pose.position_enu_m = match->position_enu_m;
  result.pose.timestamp_s = sample.timestamp_s;
  result.ok = true;
  return result;
}

GazeboPositionResult makeCanonicalGazeboPosition(const GazeboDirectPose &pose,
                                                 std::string_view expected_world_frame,
                                                 std::string_view expected_model_name) {
  GazeboPositionResult result{};
  if (expected_world_frame.empty() || expected_model_name.empty()) {
    result.reason = "invalid Gazebo identity configuration";
    return result;
  }
  if (pose.frame_id != expected_world_frame || pose.child_frame_id != expected_model_name) {
    result.reason = "unexpected Gazebo pose identity";
    return result;
  }
  if (!validTimestamp(pose.timestamp_s)) {
    result.reason = "invalid Gazebo simulation timestamp";
    return result;
  }
  if (!pose.position_enu_m.finite()) {
    result.reason = "invalid Gazebo model position";
    return result;
  }

  result.position_ned_m.value = control::enuToNed(pose.position_enu_m);
  result.position_ned_m.timestamp_s = pose.timestamp_s;
  result.ok = true;
  return result;
}

}  // namespace simulation
