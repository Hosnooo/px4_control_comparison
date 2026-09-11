# PX4 interface

The runtime publishes only native PX4 inputs:
`/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`,
`/fmu/in/vehicle_attitude_setpoint`, `/fmu/in/vehicle_rates_setpoint`,
`/fmu/in/vehicle_thrust_setpoint`, `/fmu/in/vehicle_torque_setpoint`, and
`/fmu/in/vehicle_command`.

It consumes native PX4 state from `/fmu/out/vehicle_local_position`,
`/fmu/out/vehicle_attitude`, `/fmu/out/vehicle_angular_velocity`, and
`/fmu/out/vehicle_status` as required by the selected controller. Streaming state subscriptions
use sensor-data QoS in the native runtime.

`OffboardControlMode` sets exactly the first active control level for the selected mode and the
runtime streams that heartbeat at 10 Hz. Domain commands are converted to generated `px4_msgs`
only at the ROS/PX4 boundary. Uncontrolled
`TrajectorySetpoint` position/velocity/acceleration/jerk/yaw dimensions are NaN rather than zero,
so PX4 does not interpret an unused dimension as an active zero command.

PX4 timestamps are explicit microseconds on command-domain values. Wrench messages copy that
command time into both `timestamp` and `timestamp_sample`. Arming and mode changes use explicit
`VehicleCommand` actions and are not controller equations or automatic side effects of selecting
a controller.
