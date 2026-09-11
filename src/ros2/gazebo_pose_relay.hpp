#pragma once
#if __has_include(<gz/msgs/pose_v.pb.h>)
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <string>
#include <string_view>
namespace px4_offboard::ros2_runtime {
struct GazeboDirectPoseMessageResult {
  bool ok{false};
  std::string reason;
  gz::msgs::Pose pose;
};
GazeboDirectPoseMessageResult makeGazeboDirectPose(const gz::msgs::Pose_V &poses,
                                                   std::string_view world_frame,
                                                   std::string_view model_name);
}  // namespace px4_offboard::ros2_runtime
#endif
