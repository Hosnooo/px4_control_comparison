#pragma once

#include "px4_offboard_controllers/core/state.hpp"
#include "px4_offboard_controllers/px4/command_types.hpp"
#include "px4_offboard_controllers/px4/control_level.hpp"

#include <cstdint>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

namespace px4_offboard::ros2_runtime {

class Px4StateInput {
 public:
  Px4StateInput(rclcpp::Node &node, StateRequirements requirements);

  CanonicalState state() const;
  bool armed() const;
  bool inOffboardMode() const;

 private:
  mutable std::mutex mutex_;
  CanonicalState state_{};
  bool armed_{false};
  bool in_offboard_mode_{false};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
};

class Px4CommandPublisher {
 public:
  explicit Px4CommandPublisher(rclcpp::Node &node);

  void publishControlMode(OffboardControlLevel level, std::uint64_t timestamp_us);
  void publish(const PositionCommand &command);
  void publish(const VelocityCommand &command);
  void publish(const AccelerationCommand &command);
  void publish(const AttitudeCommand &command);
  void publish(const BodyRateCommand &command);
  void publish(const NormalizedWrenchCommand &command);
  void publishVehicleCommand(const px4_msgs::msg::VehicleCommand &command);

 private:
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
};

}  // namespace px4_offboard::ros2_runtime
