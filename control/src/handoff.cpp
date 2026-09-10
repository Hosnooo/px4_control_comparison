#include "control/handoff.hpp"

#include <cmath>
#include <limits>

namespace control {
namespace {

bool representableAsFloat(double value) {
  return std::isfinite(value) &&
         std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}

template <std::size_t N>
bool finiteArray(const std::array<double, N> &values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

template <std::size_t N>
bool floatRepresentableArray(const std::array<double, N> &values) {
  for (double value : values) {
    if (!representableAsFloat(value)) {
      return false;
    }
  }
  return true;
}

template <std::size_t N>
std::array<float, N> toFloatArray(const std::array<double, N> &values) {
  std::array<float, N> result{};
  for (std::size_t index = 0; index < N; ++index) {
    result[index] = static_cast<float>(values[index]);
  }
  return result;
}

bool validMulticopterThrust(const std::array<double, 3> &thrust_body_frd) {
  return finiteArray(thrust_body_frd) && thrust_body_frd[0] == 0.0 &&
         thrust_body_frd[1] == 0.0 && thrust_body_frd[2] >= -1.0 &&
         thrust_body_frd[2] <= 0.0;
}

bool validNormalizedTorque(const std::array<double, 3> &torque_frd) {
  if (!finiteArray(torque_frd)) {
    return false;
  }
  for (double value : torque_frd) {
    if (value < -1.0 || value > 1.0) {
      return false;
    }
  }
  return true;
}

bool validUnitQuaternion(const std::array<double, 4> &quaternion,
                         double norm_tolerance) {
  if (!finiteArray(quaternion) || !std::isfinite(norm_tolerance) ||
      !(norm_tolerance > 0.0)) {
    return false;
  }
  double squared_norm = 0.0;
  for (double value : quaternion) {
    squared_norm += value * value;
  }
  return std::abs(std::sqrt(squared_norm) - 1.0) <= norm_tolerance;
}

OffboardControlModeCommand offboardForMode(HandoffMode mode,
                                            std::uint64_t timestamp_us) {
  OffboardControlModeCommand command{};
  command.timestamp = timestamp_us;
  switch (mode) {
    case HandoffMode::attitude_handoff:
      command.attitude = true;
      break;
    case HandoffMode::rate_handoff:
      command.body_rate = true;
      break;
    case HandoffMode::px4_mirror:
    case HandoffMode::lee_wrench:
      command.thrust_and_torque = true;
      break;
  }
  return command;
}

Px4HandoffResult reject(HandoffMode mode, const std::string &reason) {
  Px4HandoffResult result{};
  result.reason = reason;
  result.commands.mode = mode;
  return result;
}

}  // namespace

Px4HandoffResult buildPx4Handoff(HandoffMode mode, const HandoffInputs &input,
                                 const HandoffValidationLimits &limits) {
  if (!input.timing.publish_timestamp_us.has_value()) {
    return reject(mode, "PX4 publish timestamp must be supplied explicitly");
  }
  const std::uint64_t publish_timestamp_us = *input.timing.publish_timestamp_us;
  if (!validMulticopterThrust(input.normalized_thrust_body_frd)) {
    return reject(mode,
                  "multicopter normalized thrust must be body-FRD [0, 0, z], z in [-1, 0]");
  }

  Px4HandoffResult result{};
  result.commands.mode = mode;
  result.commands.offboard_control_mode =
      offboardForMode(mode, publish_timestamp_us);

  switch (mode) {
    case HandoffMode::attitude_handoff: {
      if (!std::isfinite(limits.quaternion_norm_tolerance) ||
          !(limits.quaternion_norm_tolerance > 0.0)) {
        return reject(mode,
                      "quaternion norm tolerance must be configured explicitly");
      }
      if (!validUnitQuaternion(input.desired_attitude_q_ned_frd,
                               limits.quaternion_norm_tolerance)) {
        return reject(mode,
                      "desired body-FRD-to-NED quaternion is not unit length");
      }
      if (!representableAsFloat(input.yaw_sp_move_rate_radps)) {
        return reject(mode, "yaw move rate cannot be represented by PX4 float32");
      }
      if (!input.reset_integral.has_value()) {
        return reject(mode, "attitude reset_integral choice must be explicit");
      }
      VehicleAttitudeSetpointCommand command{};
      command.timestamp = publish_timestamp_us;
      command.yaw_sp_move_rate = static_cast<float>(input.yaw_sp_move_rate_radps);
      command.q_d = toFloatArray(input.desired_attitude_q_ned_frd);
      command.thrust_body = toFloatArray(input.normalized_thrust_body_frd);
      command.reset_integral = *input.reset_integral;
      command.fw_control_yaw_wheel = false;
      result.commands.attitude_setpoint = command;
      break;
    }

    case HandoffMode::rate_handoff: {
      if (!floatRepresentableArray(input.body_rate_setpoint_frd_radps)) {
        return reject(mode,
                      "body-FRD rate setpoint cannot be represented by PX4 float32");
      }
      if (!input.reset_integral.has_value()) {
        return reject(mode, "rate reset_integral choice must be explicit");
      }
      VehicleRatesSetpointCommand command{};
      command.timestamp = publish_timestamp_us;
      command.roll = static_cast<float>(input.body_rate_setpoint_frd_radps[0]);
      command.pitch = static_cast<float>(input.body_rate_setpoint_frd_radps[1]);
      command.yaw = static_cast<float>(input.body_rate_setpoint_frd_radps[2]);
      command.thrust_body = toFloatArray(input.normalized_thrust_body_frd);
      command.reset_integral = *input.reset_integral;
      result.commands.rates_setpoint = command;
      break;
    }

    case HandoffMode::px4_mirror:
    case HandoffMode::lee_wrench: {
      if (!input.timing.sample_timestamp_us.has_value()) {
        return reject(mode, "PX4 sample timestamp must be supplied for torque/thrust");
      }
      const std::uint64_t sample_timestamp_us = *input.timing.sample_timestamp_us;
      if (sample_timestamp_us > publish_timestamp_us) {
        return reject(mode,
                      "sample timestamp cannot be later than publish timestamp");
      }
      if (mode == HandoffMode::lee_wrench &&
          !input.physical_wrench_normalization_ok) {
        return reject(mode,
                      "lee_wrench requires successful physical-wrench normalization");
      }
      if (!validNormalizedTorque(input.normalized_torque_frd)) {
        return reject(mode,
                      "normalized body-FRD torque is outside [-1, 1]");
      }
      VehicleTorqueSetpointCommand torque{};
      torque.timestamp = publish_timestamp_us;
      torque.timestamp_sample = sample_timestamp_us;
      torque.xyz = toFloatArray(input.normalized_torque_frd);
      result.commands.torque_setpoint = torque;

      VehicleThrustSetpointCommand thrust{};
      thrust.timestamp = publish_timestamp_us;
      thrust.timestamp_sample = sample_timestamp_us;
      thrust.xyz = toFloatArray(input.normalized_thrust_body_frd);
      result.commands.thrust_setpoint = thrust;
      break;
    }
  }

  result.ok = true;
  return result;
}

}  // namespace control
