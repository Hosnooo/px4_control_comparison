#include "px4_control_comparison_simulation/gazebo_position_ros.hpp"

#include <cstdint>

namespace simulation::runtime {
namespace {

bool validNanoseconds(std::uint32_t nanoseconds) {
  return nanoseconds < 1000000000U;
}

}  // namespace

GazeboPositionResult makeCanonicalPositionFromRosTransform(
    const geometry_msgs::msg::TransformStamped &transform,
    std::string_view expected_world_frame, std::string_view expected_model_name) {
  GazeboPositionResult result{};
  if (transform.header.stamp.sec < 0 || !validNanoseconds(transform.header.stamp.nanosec)) {
    result.reason = "invalid ROS simulation timestamp";
    return result;
  }

  GazeboDirectPose pose{};
  pose.frame_id = transform.header.frame_id;
  pose.child_frame_id = transform.child_frame_id;
  pose.position_enu_m = {transform.transform.translation.x, transform.transform.translation.y,
                         transform.transform.translation.z};
  pose.timestamp_s = static_cast<double>(transform.header.stamp.sec) +
                     1e-9 * static_cast<double>(transform.header.stamp.nanosec);
  return makeCanonicalGazeboPosition(pose, expected_world_frame, expected_model_name);
}

}  // namespace simulation::runtime
