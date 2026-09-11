#pragma once

#include <cstdint>

namespace px4_msgs::msg {
struct VehicleCommand {
  static constexpr std::uint32_t VEHICLE_CMD_DO_SET_MODE = 176;
  static constexpr std::uint32_t VEHICLE_CMD_COMPONENT_ARM_DISARM = 400;
  static constexpr std::int8_t ARMING_ACTION_DISARM = 0;
  static constexpr std::int8_t ARMING_ACTION_ARM = 1;

  std::uint64_t timestamp{};
  float param1{};
  float param2{};
  float param3{};
  float param4{};
  double param5{};
  double param6{};
  float param7{};
  std::uint32_t command{};
  std::uint8_t target_system{};
  std::uint8_t target_component{};
  std::uint8_t source_system{};
  std::uint16_t source_component{};
  std::uint8_t confirmation{};
  bool from_external{};
};
}  // namespace px4_msgs::msg
