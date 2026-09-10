#include "px4_control_comparison_simulation/gazebo_pose_relay.hpp"

#include "simulation/gazebo_position.hpp"


namespace simulation::runtime {
namespace {

bool validStamp(const gz::msgs::Time &stamp) {
  return stamp.sec() >= 0 && stamp.nsec() >= 0 && stamp.nsec() < 1000000000;
}

double stampSeconds(const gz::msgs::Time &stamp) {
  return static_cast<double>(stamp.sec()) + 1e-9 * static_cast<double>(stamp.nsec());
}

}  // namespace

GazeboDirectPoseMessageResult makeGazeboDirectPose(const gz::msgs::Pose_V &poses,
                                                   std::string_view world_frame,
                                                   std::string_view model_name) {
  GazeboDirectPoseMessageResult result{};
  if (!poses.has_header() || !poses.header().has_stamp() || !validStamp(poses.header().stamp())) {
    result.reason = "invalid Gazebo Pose_V timestamp";
    return result;
  }

  GazeboPoseVectorSample sample{};
  sample.timestamp_s = stampSeconds(poses.header().stamp());
  sample.poses.reserve(static_cast<std::size_t>(poses.pose_size()));
  for (const auto &pose : poses.pose()) {
    sample.poses.push_back(
        {pose.name(), {pose.position().x(), pose.position().y(), pose.position().z()}});
  }

  const auto selected = selectGazeboModelPose(sample, world_frame, model_name);
  if (!selected.ok) {
    result.reason = selected.reason;
    return result;
  }
  if (selected.source_pose_index >= static_cast<std::size_t>(poses.pose_size())) {
    result.reason = "Gazebo pose selection index out of range";
    return result;
  }

  result.pose.CopyFrom(poses.pose(static_cast<int>(selected.source_pose_index)));
  auto *header = result.pose.mutable_header();
  header->Clear();
  header->mutable_stamp()->CopyFrom(poses.header().stamp());

  auto *frame = header->add_data();
  frame->set_key("frame_id");
  frame->add_value(std::string(world_frame));

  auto *child = header->add_data();
  child->set_key("child_frame_id");
  child->add_value(std::string(model_name));

  result.ok = true;
  return result;
}

}  // namespace simulation::runtime
