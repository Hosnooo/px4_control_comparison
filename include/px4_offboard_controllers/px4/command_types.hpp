#pragma once

#include "px4_offboard_controllers/core/math.hpp"

#include <cmath>
#include <cstdint>

namespace px4_offboard {

// Controller-domain commands are message-independent. All world vectors are NED and all body
// vectors are FRD. timestamp_us is the PX4 publication timestamp owned by the runtime boundary.
struct PositionCommand {
  Vec3 position_ned{};
  std::uint64_t timestamp_us{0};
  bool valid() const { return timestamp_us > 0 && position_ned.finite(); }
};

struct VelocityCommand {
  Vec3 velocity_ned{};
  std::uint64_t timestamp_us{0};
  bool valid() const { return timestamp_us > 0 && velocity_ned.finite(); }
};

struct AccelerationCommand {
  Vec3 acceleration_ned{};
  std::uint64_t timestamp_us{0};
  bool valid() const { return timestamp_us > 0 && acceleration_ned.finite(); }
};

struct AttitudeCommand {
  Quat attitude_ned_frd{};
  double normalized_thrust{0.0};
  std::uint64_t timestamp_us{0};
  bool valid() const {
    return timestamp_us > 0 && attitude_ned_frd.finite() && std::isfinite(normalized_thrust) &&
           normalized_thrust >= -1.0 && normalized_thrust <= 1.0;
  }
};

struct BodyRateCommand {
  Vec3 body_rate_frd{};
  double normalized_thrust{0.0};
  std::uint64_t timestamp_us{0};
  bool valid() const {
    return timestamp_us > 0 && body_rate_frd.finite() && std::isfinite(normalized_thrust) &&
           normalized_thrust >= -1.0 && normalized_thrust <= 1.0;
  }
};

struct NormalizedWrenchCommand {
  Vec3 torque_frd{};
  Vec3 thrust_frd{};
  std::uint64_t timestamp_us{0};

  bool valid() const {
    const auto in_unit_range = [](double value) {
      return std::isfinite(value) && value >= -1.0 && value <= 1.0;
    };
    return timestamp_us > 0 && in_unit_range(torque_frd.x) && in_unit_range(torque_frd.y) &&
           in_unit_range(torque_frd.z) && in_unit_range(thrust_frd.x) &&
           in_unit_range(thrust_frd.y) && in_unit_range(thrust_frd.z);
  }
};

}  // namespace px4_offboard
