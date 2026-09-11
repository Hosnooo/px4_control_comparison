#pragma once

#include <array>
#include <cstdint>
#include <memory>

namespace px4_msgs::msg {
struct VehicleAngularVelocity {
  using SharedPtr = std::shared_ptr<VehicleAngularVelocity>;
  std::uint64_t timestamp{};
  std::uint64_t timestamp_sample{};
  std::array<float, 3> xyz{};
};
}  // namespace px4_msgs::msg
