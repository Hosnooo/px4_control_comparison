#pragma once

#include "px4_offboard_controllers/core/state.hpp"

namespace px4_offboard {

// First PX4 offboard loop that remains active for a command path.
enum class OffboardControlLevel { Position, Velocity, Acceleration, Attitude, BodyRate, Wrench };

constexpr OffboardControlLevel controlLevelFor(ControllerKind kind) {
  switch (kind) {
    case ControllerKind::Px4Position:
      return OffboardControlLevel::Position;
    case ControllerKind::Px4Velocity:
      return OffboardControlLevel::Velocity;
    case ControllerKind::GeometricAcceleration:
      return OffboardControlLevel::Acceleration;
    case ControllerKind::GeometricAttitude:
      return OffboardControlLevel::Attitude;
    case ControllerKind::GeometricRate:
    case ControllerKind::Px4AttitudeRateMirror:
      return OffboardControlLevel::BodyRate;
    case ControllerKind::LeeWrench:
      return OffboardControlLevel::Wrench;
  }
  return OffboardControlLevel::Position;
}

}  // namespace px4_offboard
