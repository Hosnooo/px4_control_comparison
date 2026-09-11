#pragma once
#include <array>
#include <cstdint>
#include <limits>
namespace px4_msgs::msg { struct TrajectorySetpoint{std::uint64_t timestamp{};std::array<float,3> position{},velocity{},acceleration{},jerk{};float yaw{},yawspeed{};};}
