#pragma once

#include "px4_offboard_controllers/core/vector3.hpp"

#include <optional>

namespace px4_offboard {

// Optional NED trajectory derivatives. Simple PX4 position/velocity modes intentionally do not
// require derivatives they never consume. Yaw derivatives are radians, rad/s and rad/s^2.
struct TrajectoryReference {
  std::optional<Vec3> position;
  std::optional<Vec3> velocity;
  std::optional<Vec3> acceleration;
  std::optional<Vec3> jerk;
  std::optional<Vec3> snap;
  std::optional<double> yaw;
  std::optional<double> yaw_rate;
  std::optional<double> yaw_accel;
};

enum class ReferenceProfile {
  PositionOnly,
  VelocityOnly,
  GeometricAcceleration,
  GeometricRate,
  LeeFull,
};

bool validateReference(const TrajectoryReference &reference, ReferenceProfile profile);

}  // namespace px4_offboard
