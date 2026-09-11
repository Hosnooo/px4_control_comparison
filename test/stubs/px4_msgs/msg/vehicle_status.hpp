#pragma once

#include <cstdint>
#include <memory>

namespace px4_msgs::msg {
struct VehicleStatus {
  using SharedPtr = std::shared_ptr<VehicleStatus>;
  static constexpr std::uint8_t ARMING_STATE_ARMED = 2;
  static constexpr std::uint8_t NAVIGATION_STATE_OFFBOARD = 14;
  std::uint64_t timestamp{};
  std::uint8_t arming_state{};
  std::uint8_t nav_state{};
};
}  // namespace px4_msgs::msg
