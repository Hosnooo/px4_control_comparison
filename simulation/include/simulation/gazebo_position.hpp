#pragma once

#include "control/state.hpp"

#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace simulation {

struct GazeboNamedPose {
  std::string name;
  control::Vec3 position_enu_m{};
};

struct GazeboPoseVectorSample {
  double timestamp_s{std::numeric_limits<double>::quiet_NaN()};
  std::vector<GazeboNamedPose> poses;
};

struct GazeboDirectPose {
  std::string frame_id;
  std::string child_frame_id;
  control::Vec3 position_enu_m{};
  double timestamp_s{std::numeric_limits<double>::quiet_NaN()};
};

struct GazeboPoseSelectionResult {
  bool ok{false};
  std::string reason;
  std::size_t source_pose_index{std::numeric_limits<std::size_t>::max()};
  GazeboDirectPose pose;
};

struct GazeboPositionResult {
  bool ok{false};
  std::string reason;
  control::TimedValue<control::Vec3> position_ned_m;
};

GazeboPoseSelectionResult selectGazeboModelPose(const GazeboPoseVectorSample &sample,
                                                std::string_view world_frame,
                                                std::string_view model_name);

GazeboPositionResult makeCanonicalGazeboPosition(const GazeboDirectPose &pose,
                                                 std::string_view expected_world_frame,
                                                 std::string_view expected_model_name);

}  // namespace simulation
