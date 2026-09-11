#pragma once

#include "px4_offboard_controllers/px4/command_builder.hpp"
#include "px4_offboard_controllers/px4/command_types.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

namespace px4_offboard {

// Generated px4_msgs conversion lives only at this boundary. Uncontrolled TrajectorySetpoint
// dimensions are NaN so PX4 does not interpret an unused axis/derivative as a zero command.
px4_msgs::msg::OffboardControlMode toPx4OffboardControlMode(OffboardControlLevel level,
                                                           std::uint64_t timestamp_us);
px4_msgs::msg::TrajectorySetpoint toPx4TrajectorySetpoint(const PositionCommand &command);
px4_msgs::msg::TrajectorySetpoint toPx4TrajectorySetpoint(const VelocityCommand &command);
px4_msgs::msg::TrajectorySetpoint toPx4TrajectorySetpoint(const AccelerationCommand &command);
px4_msgs::msg::VehicleAttitudeSetpoint toPx4VehicleAttitudeSetpoint(
    const AttitudeCommand &command);
px4_msgs::msg::VehicleRatesSetpoint toPx4VehicleRatesSetpoint(const BodyRateCommand &command);
px4_msgs::msg::VehicleTorqueSetpoint toPx4VehicleTorqueSetpoint(
    const NormalizedWrenchCommand &command);
px4_msgs::msg::VehicleThrustSetpoint toPx4VehicleThrustSetpoint(
    const NormalizedWrenchCommand &command);

}  // namespace px4_offboard
