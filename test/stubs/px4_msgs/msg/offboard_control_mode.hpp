#pragma once
#include <cstdint>
namespace px4_msgs::msg { struct OffboardControlMode{std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};}
