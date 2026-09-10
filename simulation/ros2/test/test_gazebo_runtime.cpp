#include "px4_control_comparison_simulation/gazebo_pose_relay.hpp"
#include "px4_control_comparison_simulation/gazebo_position_ros.hpp"

#include "test_support.hpp"

#include <cstdint>
#include <string>

using simulation::runtime::makeCanonicalPositionFromRosTransform;
using simulation::runtime::makeGazeboDirectPose;

namespace {

void checkHeaderValue(const gz::msgs::Header &header, const std::string &key,
                      const std::string &expected) {
  for (int index = 0; index < header.data_size(); ++index) {
    const auto &entry = header.data(index);
    if (entry.key() == key) {
      check(entry.value_size() == 1, key + " must have one value");
      check(entry.value(0) == expected, key + " value");
      return;
    }
  }
  check(false, "missing header key " + key);
}

void testGazeboRelayCopiesExactSelectedPose() {
  gz::msgs::Pose_V poses;
  poses.mutable_header()->mutable_stamp()->set_sec(12);
  poses.mutable_header()->mutable_stamp()->set_nsec(345000000);

  auto *other = poses.add_pose();
  other->set_name("ground_plane");
  other->set_id(11);
  other->mutable_position()->set_x(99.0);

  auto *vehicle = poses.add_pose();
  vehicle->set_name("f450_0");
  vehicle->set_id(77);
  vehicle->mutable_position()->set_x(2.0);
  vehicle->mutable_position()->set_y(-4.0);
  vehicle->mutable_position()->set_z(1.5);
  vehicle->mutable_orientation()->set_w(0.8);
  vehicle->mutable_orientation()->set_x(0.1);
  vehicle->mutable_orientation()->set_y(0.2);
  vehicle->mutable_orientation()->set_z(0.3);

  const auto result = makeGazeboDirectPose(poses, "default", "f450_0");
  check(result.ok, "relay selects configured vehicle");
  check(result.pose.name() == "f450_0", "relay preserves source pose name");
  check(result.pose.id() == 77, "relay preserves source entity id");
  checkNear(result.pose.position().x(), 2.0, 1e-12, "relay x");
  checkNear(result.pose.orientation().w(), 0.8, 1e-12, "relay orientation");
  check(result.pose.header().stamp().sec() == 12, "relay preserves stamp seconds");
  check(result.pose.header().stamp().nsec() == 345000000,
        "relay preserves stamp nanoseconds");
  checkHeaderValue(result.pose.header(), "frame_id", "default");
  checkHeaderValue(result.pose.header(), "child_frame_id", "f450_0");
}

void testGazeboRelayRejectsInvalidSourceStamp() {
  gz::msgs::Pose_V poses;
  poses.mutable_header()->mutable_stamp()->set_sec(1);
  poses.mutable_header()->mutable_stamp()->set_nsec(1000000000);
  auto *vehicle = poses.add_pose();
  vehicle->set_name("f450_0");
  check(!makeGazeboDirectPose(poses, "default", "f450_0").ok,
        "relay rejects invalid nanoseconds");
}

void testRosTransformConvertsPositionOnce() {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp.sec = 12;
  transform.header.stamp.nanosec = 345000000U;
  transform.header.frame_id = "default";
  transform.child_frame_id = "f450_0";
  transform.transform.translation.x = 2.0;
  transform.transform.translation.y = -4.0;
  transform.transform.translation.z = 1.5;

  const auto result =
      makeCanonicalPositionFromRosTransform(transform, "default", "f450_0");
  check(result.ok, "ROS transform accepted");
  checkNear(result.position_ned_m.value.x, -4.0, 1e-12, "north from ENU y");
  checkNear(result.position_ned_m.value.y, 2.0, 1e-12, "east from ENU x");
  checkNear(result.position_ned_m.value.z, -1.5, 1e-12, "down from ENU z");
  checkNear(result.position_ned_m.timestamp_s, 12.345, 1e-12,
            "ROS stamp preserved");
}

void testRosTransformRejectsWrongIdentity() {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp.sec = 1;
  transform.header.frame_id = "default";
  transform.child_frame_id = "other_vehicle";
  check(!makeCanonicalPositionFromRosTransform(transform, "default", "f450_0").ok,
        "wrong child frame rejected");
}

}  // namespace

int main() {
  testGazeboRelayCopiesExactSelectedPose();
  testGazeboRelayRejectsInvalidSourceStamp();
  testRosTransformConvertsPositionOnce();
  testRosTransformRejectsWrongIdentity();
  return 0;
}
