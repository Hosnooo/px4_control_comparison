#pragma once

#include <cstdint>
#include <memory>

namespace px4_msgs::msg {
struct VehicleLocalPosition {
  using SharedPtr = std::shared_ptr<VehicleLocalPosition>;
  std::uint64_t timestamp{};
  std::uint64_t timestamp_sample{};
  bool xy_valid{};
  bool z_valid{};
  bool v_xy_valid{};
  bool v_z_valid{};
  float x{};
  float y{};
  float z{};
  float vx{};
  float vy{};
  float vz{};
};
}  // namespace px4_msgs::msg
