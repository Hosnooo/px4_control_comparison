#pragma once
#include <array>
#include <cstdint>
namespace px4_msgs::msg { struct VehicleThrustSetpoint{std::uint64_t timestamp{},timestamp_sample{};std::array<float,3> xyz{};};}
