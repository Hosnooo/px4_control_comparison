#include "simulation/gazebo_position.hpp"
#include "test_support.hpp"

#include <limits>

using control::Vec3;
using simulation::GazeboDirectPose;
using simulation::GazeboNamedPose;
using simulation::GazeboPoseVectorSample;

namespace {

void checkVecNear(const Vec3 &actual, const Vec3 &expected, const char *message) {
  checkNear(actual.x, expected.x, 1e-12, std::string(message) + " x");
  checkNear(actual.y, expected.y, 1e-12, std::string(message) + " y");
  checkNear(actual.z, expected.z, 1e-12, std::string(message) + " z");
}

void testSelectsExactF450AndPreservesSourceMetadata() {
  GazeboPoseVectorSample sample{};
  sample.timestamp_s = 12.345;
  sample.poses = {
      GazeboNamedPose{"ground_plane", {9.0, 9.0, 0.0}},
      GazeboNamedPose{"f450_0", {2.0, 5.0, 1.5}},
      GazeboNamedPose{"sunUTC", {-3.0, 7.0, 10.0}},
  };

  const auto result = simulation::selectGazeboModelPose(sample, "default", "f450_0");
  check(result.ok, "exact F450 pose selected");
  check(result.source_pose_index == 1, "source pose index preserved for raw relay copy");
  check(result.pose.frame_id == "default", "world frame preserved");
  check(result.pose.child_frame_id == "f450_0", "model identity preserved");
  checkNear(result.pose.timestamp_s, 12.345, 1e-12, "simulation timestamp preserved");
  checkVecNear(result.pose.position_enu_m, {2.0, 5.0, 1.5}, "ENU position preserved");
}

void testSelectionFailsClosed() {
  GazeboPoseVectorSample sample{};
  sample.timestamp_s = 1.0;
  sample.poses = {GazeboNamedPose{"x500", {1.0, 2.0, 3.0}}};
  check(!simulation::selectGazeboModelPose(sample, "default", "f450_0").ok,
        "missing F450 instance rejected");

  sample.poses = {
      GazeboNamedPose{"f450_0", {1.0, 2.0, 3.0}},
      GazeboNamedPose{"f450_0", {4.0, 5.0, 6.0}},
  };
  check(!simulation::selectGazeboModelPose(sample, "default", "f450_0").ok,
        "duplicate F450 instance poses rejected as ambiguous");

  sample.poses = {GazeboNamedPose{"f450_0", {1.0, 2.0, 3.0}}};
  sample.timestamp_s = -0.001;
  check(!simulation::selectGazeboModelPose(sample, "default", "f450_0").ok,
        "negative simulation timestamp rejected");

  sample.timestamp_s = 0.0;
  sample.poses[0].position_enu_m.z = std::numeric_limits<double>::quiet_NaN();
  check(!simulation::selectGazeboModelPose(sample, "default", "f450_0").ok,
        "non-finite F450 instance position rejected");

  sample.poses[0].position_enu_m = {1.0, 2.0, 3.0};
  check(!simulation::selectGazeboModelPose(sample, "", "f450_0").ok,
        "empty world frame rejected");
  check(!simulation::selectGazeboModelPose(sample, "default", "").ok,
        "empty model identity rejected");
}

void testConvertsPositionToCanonicalNedExactlyOnce() {
  const GazeboDirectPose pose{"default", "f450_0", {2.0, 5.0, 1.5}, 8.25};
  const auto result = simulation::makeCanonicalGazeboPosition(pose, "default", "f450_0");
  check(result.ok, "valid direct Gazebo pose accepted");
  checkVecNear(result.position_ned_m.value, {5.0, 2.0, -1.5}, "single ENU to NED conversion");
  checkNear(result.position_ned_m.timestamp_s, 8.25, 1e-12, "canonical timestamp preserved");
}

void testCanonicalAdapterRejectsIdentityFrameAndNumericErrors() {
  const GazeboDirectPose valid{"default", "f450_0", {2.0, 5.0, 1.5}, 8.25};

  auto pose = valid;
  pose.frame_id = "other_world";
  check(!simulation::makeCanonicalGazeboPosition(pose, "default", "f450_0").ok,
        "wrong world frame rejected");

  pose = valid;
  pose.child_frame_id = "x500";
  check(!simulation::makeCanonicalGazeboPosition(pose, "default", "f450_0").ok,
        "wrong model identity rejected");

  pose = valid;
  pose.timestamp_s = std::numeric_limits<double>::infinity();
  check(!simulation::makeCanonicalGazeboPosition(pose, "default", "f450_0").ok,
        "non-finite timestamp rejected");

  pose = valid;
  pose.position_enu_m.x = std::numeric_limits<double>::infinity();
  check(!simulation::makeCanonicalGazeboPosition(pose, "default", "f450_0").ok,
        "non-finite position rejected");
}

}  // namespace

int main() {
  testSelectsExactF450AndPreservesSourceMetadata();
  testSelectionFailsClosed();
  testConvertsPositionToCanonicalNedExactlyOnce();
  testCanonicalAdapterRejectsIdentityFrameAndNumericErrors();
  return 0;
}
