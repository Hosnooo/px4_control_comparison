#pragma once
#include <array>
#include <cstdint>
namespace px4_msgs::msg { struct VehicleAttitudeSetpoint{std::uint64_t timestamp{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};float yaw_sp_move_rate{};bool reset_integral{},fw_control_yaw_wheel{};};}
