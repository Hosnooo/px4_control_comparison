#pragma once

#include <cstdint>

namespace px4_msgs::msg {
struct VehicleCommand {
  std::uint64_t timestamp{};
};
}  // namespace px4_msgs::msg
