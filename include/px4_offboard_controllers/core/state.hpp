#pragma once

#include "px4_offboard_controllers/core/math.hpp"

#include <limits>
#include <optional>

namespace px4_offboard {

template <class T>
struct TimedValue {
  T value{};
  double timestamp_s{std::numeric_limits<double>::quiet_NaN()};
};

enum class ControllerKind {
  Px4Position,
  Px4Velocity,
  GeometricAcceleration,
  GeometricAttitude,
  GeometricRate,
  Px4AttitudeRateMirror,
  LeeWrench,
};

struct StateRequirements {
  bool position{false};
  bool velocity{false};
  bool attitude{false};
  bool body_rate{false};
  bool body_angular_acceleration{false};
};

// Canonical controller state. Position/velocity are world NED; attitude rotates FRD body vectors
// into NED; body rates and angular acceleration are FRD. Every field retains the timestamp owned by
// its source boundary so freshness policy can be applied without substituting another source.
struct CanonicalState {
  std::optional<TimedValue<Vec3>> position_ned;
  std::optional<TimedValue<Vec3>> velocity_ned;
  std::optional<TimedValue<Quat>> attitude_ned_frd;
  std::optional<TimedValue<Vec3>> body_rate_frd;
  std::optional<TimedValue<Vec3>> body_angular_accel_frd;
};

StateRequirements requirementsFor(ControllerKind kind);
bool validateState(const CanonicalState &state, StateRequirements requirements);

}  // namespace px4_offboard
