#pragma once

#include "px4_offboard_controllers/core/math.hpp"

#include <cstdint>
#include <optional>

namespace px4_offboard {

template <class T>
struct TimedValue {
  T value{};
  double timestamp_s{0.0};
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
};

// Canonical controller state. Position/velocity are world NED; attitude rotates FRD body vectors
// into NED; body rates are FRD rad/s. timestamp_us is owned by the state-source boundary.
struct CanonicalState {
  std::uint64_t timestamp_us{0};
  std::optional<Vec3> position_ned;
  std::optional<Vec3> velocity_ned;
  std::optional<Quat> attitude_ned_frd;
  std::optional<Vec3> body_rate_frd;
};

StateRequirements requirementsFor(ControllerKind kind);
bool validateState(const CanonicalState &state, StateRequirements requirements);

}  // namespace px4_offboard
