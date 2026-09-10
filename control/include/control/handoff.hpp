#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace control {

enum class HandoffMode {
  attitude_handoff,
  rate_handoff,
  px4_mirror,
  lee_wrench,
};

struct HandoffTiming {
  std::optional<std::uint64_t> publish_timestamp_us;
  std::optional<std::uint64_t> sample_timestamp_us;
};

struct HandoffValidationLimits {
  double quaternion_norm_tolerance{0.0};
};

struct HandoffInputs {
  HandoffTiming timing{};
  std::array<double, 4> desired_attitude_q_ned_frd{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  double yaw_sp_move_rate_radps{std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 3> body_rate_setpoint_frd_radps{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 3> normalized_torque_frd{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 3> normalized_thrust_body_frd{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  std::optional<bool> reset_integral;
  bool physical_wrench_normalization_ok{false};
};

// Source-shaped mirrors of the pinned px4_msgs fields used at the ROS boundary.
// They deliberately contain no ROS dependency so message semantics are unit-testable.
struct OffboardControlModeCommand {
  std::uint64_t timestamp{0};
  bool position{false};
  bool velocity{false};
  bool acceleration{false};
  bool attitude{false};
  bool body_rate{false};
  bool thrust_and_torque{false};
  bool direct_actuator{false};
};

struct VehicleAttitudeSetpointCommand {
  std::uint64_t timestamp{0};
  float yaw_sp_move_rate{0.0F};
  std::array<float, 4> q_d{};
  std::array<float, 3> thrust_body{};
  bool reset_integral{false};
  bool fw_control_yaw_wheel{false};
};

struct VehicleRatesSetpointCommand {
  std::uint64_t timestamp{0};
  float roll{0.0F};
  float pitch{0.0F};
  float yaw{0.0F};
  std::array<float, 3> thrust_body{};
  bool reset_integral{false};
};

struct VehicleTorqueSetpointCommand {
  std::uint64_t timestamp{0};
  std::uint64_t timestamp_sample{0};
  std::array<float, 3> xyz{};
};

struct VehicleThrustSetpointCommand {
  std::uint64_t timestamp{0};
  std::uint64_t timestamp_sample{0};
  std::array<float, 3> xyz{};
};

struct Px4HandoffCommands {
  HandoffMode mode{HandoffMode::attitude_handoff};
  OffboardControlModeCommand offboard_control_mode{};
  std::optional<VehicleAttitudeSetpointCommand> attitude_setpoint;
  std::optional<VehicleRatesSetpointCommand> rates_setpoint;
  std::optional<VehicleTorqueSetpointCommand> torque_setpoint;
  std::optional<VehicleThrustSetpointCommand> thrust_setpoint;
};

struct Px4HandoffResult {
  bool ok{false};
  std::string reason;
  Px4HandoffCommands commands{};
};

Px4HandoffResult buildPx4Handoff(HandoffMode mode, const HandoffInputs &input,
                                 const HandoffValidationLimits &limits);

}  // namespace control
