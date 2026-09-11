#include "runtime.hpp"

#include "px4_offboard_controllers/px4/message_adapter.hpp"

namespace px4_offboard::ros2_runtime {
namespace {

inline constexpr const char *kOffboardModeTopic = "/fmu/in/offboard_control_mode";
inline constexpr const char *kTrajectoryTopic = "/fmu/in/trajectory_setpoint";
inline constexpr const char *kAttitudeSetpointTopic = "/fmu/in/vehicle_attitude_setpoint";
inline constexpr const char *kRatesSetpointTopic = "/fmu/in/vehicle_rates_setpoint";
inline constexpr const char *kThrustSetpointTopic = "/fmu/in/vehicle_thrust_setpoint";
inline constexpr const char *kTorqueSetpointTopic = "/fmu/in/vehicle_torque_setpoint";
inline constexpr const char *kVehicleCommandTopic = "/fmu/in/vehicle_command";

}  // namespace

Px4CommandPublisher::Px4CommandPublisher(rclcpp::Node &node) {
  offboard_mode_pub_ =
      node.create_publisher<px4_msgs::msg::OffboardControlMode>(kOffboardModeTopic, 10);
  trajectory_pub_ = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(kTrajectoryTopic, 10);
  attitude_pub_ = node.create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      kAttitudeSetpointTopic, 10);
  rates_pub_ =
      node.create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(kRatesSetpointTopic, 10);
  thrust_pub_ =
      node.create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(kThrustSetpointTopic, 10);
  torque_pub_ =
      node.create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(kTorqueSetpointTopic, 10);
  vehicle_command_pub_ =
      node.create_publisher<px4_msgs::msg::VehicleCommand>(kVehicleCommandTopic, 10);
}

void Px4CommandPublisher::publishControlMode(OffboardControlLevel level,
                                             std::uint64_t timestamp_us) {
  offboard_mode_pub_->publish(toPx4OffboardControlMode(level, timestamp_us));
}

void Px4CommandPublisher::publish(const PositionCommand &command) {
  trajectory_pub_->publish(toPx4TrajectorySetpoint(command));
}

void Px4CommandPublisher::publish(const VelocityCommand &command) {
  trajectory_pub_->publish(toPx4TrajectorySetpoint(command));
}

void Px4CommandPublisher::publish(const AccelerationCommand &command) {
  trajectory_pub_->publish(toPx4TrajectorySetpoint(command));
}

void Px4CommandPublisher::publish(const AttitudeCommand &command) {
  attitude_pub_->publish(toPx4VehicleAttitudeSetpoint(command));
}

void Px4CommandPublisher::publish(const BodyRateCommand &command) {
  rates_pub_->publish(toPx4VehicleRatesSetpoint(command));
}

void Px4CommandPublisher::publish(const NormalizedWrenchCommand &command) {
  torque_pub_->publish(toPx4VehicleTorqueSetpoint(command));
  thrust_pub_->publish(toPx4VehicleThrustSetpoint(command));
}

void Px4CommandPublisher::publishVehicleCommand(
    const px4_msgs::msg::VehicleCommand &command) {
  vehicle_command_pub_->publish(command);
}

}  // namespace px4_offboard::ros2_runtime
