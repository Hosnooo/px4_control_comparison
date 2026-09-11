#pragma once

#include <array>
#include <cstdint>
#include <memory>

namespace px4_msgs::msg {
struct VehicleAttitude {
  using SharedPtr = std::shared_ptr<VehicleAttitude>;
  std::uint64_t timestamp{};
  std::uint64_t timestamp_sample{};
  std::array<float, 4> q{};
};
}  // namespace px4_msgs::msg
