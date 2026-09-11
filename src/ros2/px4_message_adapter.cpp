#include "px4_offboard_controllers/px4/message_adapter.hpp"

#include <limits>
#include <stdexcept>

namespace px4_offboard {
namespace {

float nanf() { return std::numeric_limits<float>::quiet_NaN(); }

px4_msgs::msg::TrajectorySetpoint blankTrajectory(std::uint64_t timestamp_us) {
  px4_msgs::msg::TrajectorySetpoint message{};
  message.timestamp = timestamp_us;
  message.position = {nanf(), nanf(), nanf()};
  message.velocity = {nanf(), nanf(), nanf()};
  message.acceleration = {nanf(), nanf(), nanf()};
  message.jerk = {nanf(), nanf(), nanf()};
  message.yaw = nanf();
  message.yawspeed = nanf();
  return message;
}

}  // namespace

px4_msgs::msg::OffboardControlMode toPx4OffboardControlMode(OffboardControlLevel level,
                                                             std::uint64_t timestamp_us) {
  if (!timestamp_us) {
    throw std::invalid_argument("zero PX4 timestamp");
  }

  const auto flags = offboardFlags(level);
  px4_msgs::msg::OffboardControlMode message{};
  message.timestamp = timestamp_us;
  message.position = flags.position;
  message.velocity = flags.velocity;
  message.acceleration = flags.acceleration;
  message.attitude = flags.attitude;
  message.body_rate = flags.body_rate;
  message.thrust_and_torque = flags.thrust_and_torque;
  message.direct_actuator = flags.direct_actuator;
  return message;
}

px4_msgs::msg::TrajectorySetpoint toPx4TrajectorySetpoint(const PositionCommand &command) {
  if (!command.valid()) {
    throw std::invalid_argument("invalid position command");
  }

  auto message = blankTrajectory(command.timestamp_us);
  message.position = {float(command.position_ned.x), float(command.position_ned.y),
                      float(command.position_ned.z)};
  return message;
}

px4_msgs::msg::TrajectorySetpoint toPx4TrajectorySetpoint(const VelocityCommand &command) {
  if (!command.valid()) {
    throw std::invalid_argument("invalid velocity command");
  }

  auto message = blankTrajectory(command.timestamp_us);
  message.velocity = {float(command.velocity_ned.x), float(command.velocity_ned.y),
                      float(command.velocity_ned.z)};
  return message;
}

px4_msgs::msg::TrajectorySetpoint toPx4TrajectorySetpoint(const AccelerationCommand &command) {
  if (!command.valid()) {
    throw std::invalid_argument("invalid acceleration command");
  }

  auto message = blankTrajectory(command.timestamp_us);
  message.acceleration = {float(command.acceleration_ned.x), float(command.acceleration_ned.y),
                          float(command.acceleration_ned.z)};
  return message;
}

px4_msgs::msg::VehicleAttitudeSetpoint toPx4VehicleAttitudeSetpoint(
    const AttitudeCommand &command) {
  if (!command.valid()) {
    throw std::invalid_argument("invalid attitude command");
  }

  const auto attitude = command.attitude_ned_frd.normalized();
  px4_msgs::msg::VehicleAttitudeSetpoint message{};
  message.timestamp = command.timestamp_us;
  message.q_d = {float(attitude.w), float(attitude.x), float(attitude.y), float(attitude.z)};
  message.thrust_body = {0, 0, float(-command.normalized_thrust)};
  return message;
}

px4_msgs::msg::VehicleRatesSetpoint toPx4VehicleRatesSetpoint(const BodyRateCommand &command) {
  if (!command.valid()) {
    throw std::invalid_argument("invalid body-rate command");
  }

  px4_msgs::msg::VehicleRatesSetpoint message{};
  message.timestamp = command.timestamp_us;
  message.roll = float(command.body_rate_frd.x);
  message.pitch = float(command.body_rate_frd.y);
  message.yaw = float(command.body_rate_frd.z);
  message.thrust_body = {0, 0, float(-command.normalized_thrust)};
  return message;
}

px4_msgs::msg::VehicleTorqueSetpoint toPx4VehicleTorqueSetpoint(
    const NormalizedWrenchCommand &command) {
  if (!command.valid()) {
    throw std::invalid_argument("invalid wrench command");
  }

  px4_msgs::msg::VehicleTorqueSetpoint message{};
  message.timestamp = command.timestamp_us;
  message.timestamp_sample = command.timestamp_us;
  message.xyz = {float(command.torque_frd.x), float(command.torque_frd.y),
                 float(command.torque_frd.z)};
  return message;
}

px4_msgs::msg::VehicleThrustSetpoint toPx4VehicleThrustSetpoint(
    const NormalizedWrenchCommand &command) {
  if (!command.valid()) {
    throw std::invalid_argument("invalid wrench command");
  }

  px4_msgs::msg::VehicleThrustSetpoint message{};
  message.timestamp = command.timestamp_us;
  message.timestamp_sample = command.timestamp_us;
  message.xyz = {float(command.thrust_frd.x), float(command.thrust_frd.y),
                 float(command.thrust_frd.z)};
  return message;
}

}  // namespace px4_offboard
