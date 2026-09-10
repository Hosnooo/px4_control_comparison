# Architecture

## Goal

Compare four placements of the multirotor control boundary while keeping trajectory, canonical state, outer-loop implementation, simulation vehicle, diagnostics schema, and executable codebase fixed.

```text
         canonical TrajectoryReference
                    |
    direct position + PX4 estimator state
                    |
             Lee translation
                    |
        desired force / attitude
                    |
    +---------------+---------------+----------------+----------------+
    |                               |                |                |
 attitude_handoff              rate_handoff      px4_mirror       lee_wrench
    |                               |                |                |
 PX4 normalized thrust         geometric rate    PX4 attitude     Lee moment N m
    |                               |             + rate mirror       |
 VehicleAttitudeSetpoint       VehicleRates      normalized        physical torque
    |                          Setpoint           torque/thrust     normalization
 PX4 attitude+rate                  |                |                |
    +-------------------------------+----------------+----------------+
                                    |
                         PX4 control allocation
                                    |
                              motor outputs
```

## Canonical state

`CanonicalState` intentionally combines independently sourced signals and retains each source timestamp/age:

| Signal | Simulation | Experiment |
|---|---|---|
| primary position | direct Gazebo/model position, converted once to NED | raw Vicon position, converted once to NED |
| diagnostic position | PX4 EKF | PX4 EKF |
| velocity | PX4 EKF | PX4 EKF |
| attitude | PX4 estimator | PX4 estimator |
| body angular rate | PX4 `VehicleAngularVelocity.xyz` | same |
| angular acceleration | PX4 `VehicleAngularVelocity.xyz_derivative` when required | same |

No velocity is obtained by differentiating Vicon/Gazebo position. No independent rate derivative is created for the PX4 mirror.

## Four boundaries

### `attitude_handoff`

Offboard computes trajectory, Lee translational force, desired attitude and PX4-normalized thrust. It publishes `VehicleAttitudeSetpoint`; PX4 owns attitude, rate, allocation and motor control.

### `rate_handoff`

Offboard adds the separate `GeometricRateController` and publishes `VehicleRatesSetpoint`. PX4 owns rate, allocation and motors.

### `px4_mirror`

Offboard runs a source-faithful mirror of the selected PX4 attitude and rate loops. It publishes normalized `VehicleTorqueSetpoint` and `VehicleThrustSetpoint`; PX4 owns allocation and motors.

### `lee_wrench`

Offboard runs the full physical Lee force/moment controller. Physical collective thrust [N] is converted through the PX4 thrust-normalization semantics; physical moment [N·m] is converted through a separate, provenance-aware `PhysicalTorqueNormalization`. PX4 owns allocation and motors.

## Simulation and experiment adapters

Only position-source adapters differ. Both feed the same direct-position message into the same controller. The controller always obtains velocity, attitude and rates from PX4. Experiment additionally sends the Vicon pose to PX4 external vision for EKF fusion; it does not estimate velocity.

## Low-level observability

The frozen PX4 DDS YAML does not publish every signal required for research diagnostics. The ROS integration milestone will add a small auditable patch that enables only the required existing uORB publications over XRCE-DDS. That patch must not alter controller equations, allocator behavior, or state estimation.

## Safety boundary

Real hardware is never auto-armed by project default. `lee_wrench` hardware operation is rejected unless a measured hardware torque calibration passes schema, provenance, conditioning, residual and operating-range checks. Gazebo constants cannot satisfy that gate.
