#pragma once

#include "px4_offboard_controllers/px4/control_level.hpp"

#include <string_view>

namespace px4_offboard {

struct OffboardControlFlags {
  bool position{false};
  bool velocity{false};
  bool acceleration{false};
  bool attitude{false};
  bool body_rate{false};
  bool thrust_and_torque{false};
  bool direct_actuator{false};
};

// PX4 interprets the first true OffboardControlMode field as the active control boundary.
constexpr OffboardControlFlags offboardFlags(OffboardControlLevel level) {
  OffboardControlFlags flags{};
  switch (level) {
    case OffboardControlLevel::Position:
      flags.position = true;
      break;
    case OffboardControlLevel::Velocity:
      flags.velocity = true;
      break;
    case OffboardControlLevel::Acceleration:
      flags.acceleration = true;
      break;
    case OffboardControlLevel::Attitude:
      flags.attitude = true;
      break;
    case OffboardControlLevel::BodyRate:
      flags.body_rate = true;
      break;
    case OffboardControlLevel::Wrench:
      flags.thrust_and_torque = true;
      break;
  }
  return flags;
}

inline constexpr std::string_view kOffboardControlModeTopic = "/fmu/in/offboard_control_mode";
inline constexpr std::string_view kTrajectorySetpointTopic = "/fmu/in/trajectory_setpoint";
inline constexpr std::string_view kVehicleAttitudeSetpointTopic =
    "/fmu/in/vehicle_attitude_setpoint";
inline constexpr std::string_view kVehicleRatesSetpointTopic = "/fmu/in/vehicle_rates_setpoint";
inline constexpr std::string_view kVehicleThrustSetpointTopic =
    "/fmu/in/vehicle_thrust_setpoint";
inline constexpr std::string_view kVehicleTorqueSetpointTopic =
    "/fmu/in/vehicle_torque_setpoint";
inline constexpr std::string_view kVehicleCommandTopic = "/fmu/in/vehicle_command";

}  // namespace px4_offboard
