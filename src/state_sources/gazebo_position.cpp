#include "px4_offboard_controllers/state_sources/gazebo_position.hpp"

#include "px4_offboard_controllers/core/frames.hpp"

#include <cmath>

namespace px4_offboard {
namespace {

bool validTimestamp(double timestamp_s) {
  return std::isfinite(timestamp_s) && timestamp_s >= 0.0;
}

}  // namespace

GazeboPoseSelectionResult selectGazeboModelPose(const GazeboPoseVectorSample &sample,
                                                 std::string_view world,
                                                 std::string_view model) {
  GazeboPoseSelectionResult result{};
  if (world.empty() || model.empty()) {
    result.reason = "invalid Gazebo identity configuration";
    return result;
  }
  if (!validTimestamp(sample.timestamp_s)) {
    result.reason = "invalid Gazebo simulation timestamp";
    return result;
  }

  const GazeboNamedPose *match = nullptr;
  std::size_t source_pose_index = 0;
  std::size_t match_count = 0;
  for (std::size_t i = 0; i < sample.poses.size(); ++i) {
    if (sample.poses[i].name == model) {
      match = &sample.poses[i];
      source_pose_index = i;
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
  if (!match || !match->position_enu_m.finite()) {
    result.reason = "invalid Gazebo model position";
    return result;
  }

  result.source_pose_index = source_pose_index;
  result.pose = {std::string(world), std::string(model), match->position_enu_m,
                 sample.timestamp_s};
  result.ok = true;
  return result;
}

GazeboPositionResult makeCanonicalGazeboPosition(const GazeboDirectPose &pose,
                                                  std::string_view world,
                                                  std::string_view model) {
  GazeboPositionResult result{};
  if (world.empty() || model.empty()) {
    result.reason = "invalid Gazebo identity configuration";
    return result;
  }
  if (pose.frame_id != world || pose.child_frame_id != model) {
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

  // Gazebo supplies ENU; the canonical controller state owns the one ENU -> NED conversion.
  result.position_ned_m = {enuToNed(pose.position_enu_m), pose.timestamp_s};
  result.ok = true;
  return result;
}

}  // namespace px4_offboard
