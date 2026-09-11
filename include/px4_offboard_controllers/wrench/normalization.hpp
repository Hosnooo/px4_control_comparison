#pragma once

#include "px4_offboard_controllers/px4/command_types.hpp"

#include <optional>
#include <string>

namespace px4_offboard {

// Generic fail-closed physical-wrench normalization result. Airframe/sensor-specific providers
// live outside controller and PX4 layers and fill normalized FRD torque/thrust only on success.
struct WrenchNormalizationResult {
  bool ok{false};
  std::string reason;
  Vec3 normalized_torque_frd{};
  Vec3 normalized_thrust_frd{};
};

inline std::optional<NormalizedWrenchCommand> toPx4WrenchCommand(
    const WrenchNormalizationResult &result, std::uint64_t timestamp_us) {
  if (!result.ok) return std::nullopt;

  NormalizedWrenchCommand command{result.normalized_torque_frd, result.normalized_thrust_frd,
                                  timestamp_us};
  return command.valid() ? std::optional<NormalizedWrenchCommand>{command} : std::nullopt;
}

}  // namespace px4_offboard
