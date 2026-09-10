#pragma once

#include <gz/msgs.hh>

#include <string>
#include <string_view>

namespace simulation::runtime {

struct GazeboDirectPoseMessageResult {
  bool ok{false};
  std::string reason;
  gz::msgs::Pose pose;
};

GazeboDirectPoseMessageResult makeGazeboDirectPose(const gz::msgs::Pose_V &poses,
                                                   std::string_view world_frame,
                                                   std::string_view model_name);

}  // namespace simulation::runtime
