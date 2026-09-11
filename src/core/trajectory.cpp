#include "px4_offboard_controllers/core/trajectory.hpp"

#include <cmath>

namespace px4_offboard {
namespace {

bool finite(const std::optional<Vec3> &value) {
  return !value || value->finite();
}

bool finite(const std::optional<double> &value) {
  return !value || std::isfinite(*value);
}

}  // namespace

bool validateReference(const TrajectoryReference &reference, ReferenceProfile profile) {
  if (!finite(reference.position) || !finite(reference.velocity) ||
      !finite(reference.acceleration) || !finite(reference.jerk) || !finite(reference.snap) ||
      !finite(reference.yaw) || !finite(reference.yaw_rate) || !finite(reference.yaw_accel)) {
    return false;
  }

  switch (profile) {
    case ReferenceProfile::PositionOnly:
      return reference.position.has_value();
    case ReferenceProfile::VelocityOnly:
      return reference.velocity.has_value();
    case ReferenceProfile::GeometricAcceleration:
      return reference.position && reference.velocity && reference.acceleration;
    case ReferenceProfile::GeometricRate:
      return reference.position && reference.velocity && reference.acceleration && reference.yaw;
    case ReferenceProfile::LeeFull:
      return reference.position && reference.velocity && reference.acceleration && reference.jerk &&
             reference.snap && reference.yaw && reference.yaw_rate && reference.yaw_accel;
  }
  return false;
}

}  // namespace px4_offboard
