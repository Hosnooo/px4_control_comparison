#pragma once
#include <array>
#include <cstdint>
namespace px4_msgs::msg { struct VehicleRatesSetpoint{std::uint64_t timestamp{};float roll{},pitch{},yaw{};std::array<float,3> thrust_body{};bool reset_integral{};};}
