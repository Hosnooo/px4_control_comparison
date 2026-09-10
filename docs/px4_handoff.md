# PX4 handoff contract

This document defines the ROS-independent command boundary for the four controller modes. The
transport layer will copy these source-shaped command payloads into the pinned `px4_msgs` types; it
must not recompute controller mathematics or reinterpret frames.

## Frozen authority

The command contract is audited against:

- `ANCL/PX4-Autopilot@c0a1a2ecbb4c36648e12b69e7db1f29fd335f2dc`;
- `ANCL/px4_msgs@392e831c1f659429ca83902e66820d7094591410`;
- PX4 `src/modules/commander/ModeUtil/control_mode.cpp` for offboard ownership;
- PX4 `src/modules/mc_rate_control/MulticopterRateControl.cpp` for torque/thrust sample stamping;
- `OffboardControlMode.msg`, `VehicleAttitudeSetpoint.msg`, `VehicleRatesSetpoint.msg`,
  `VehicleTorqueSetpoint.msg`, `VehicleThrustSetpoint.msg`, and `VehicleAttitude.msg` for exact
  message fields, units, and quaternion convention.

## Mode ownership and publication

| Mode | `OffboardControlMode` first active field | Setpoint payload | PX4 still owns |
|---|---|---|---|
| `attitude_handoff` | `attitude` | `VehicleAttitudeSetpoint` | attitude, rate, allocation, motors |
| `rate_handoff` | `body_rate` | `VehicleRatesSetpoint` | rate, allocation, motors |
| `px4_mirror` | `thrust_and_torque` | `VehicleTorqueSetpoint` + `VehicleThrustSetpoint` | allocation, motors |
| `lee_wrench` | `thrust_and_torque` | normalized `VehicleTorqueSetpoint` + `VehicleThrustSetpoint` | allocation, motors |

Every earlier and later `OffboardControlMode` field is false. This matters because the pinned PX4
commander evaluates these fields with an ordered `if`/`else if` chain.

## Frame, unit, and sign contract

The controller side remains canonical NED/FRD. `VehicleAttitudeSetpoint.q_d` is serialized in
Hamilton `[w,x,y,z]` order as the desired body-FRD-to-NED rotation. The project uses the same
quaternion direction documented by pinned `VehicleAttitude.q`; no ENU/FLU conversion occurs at this
boundary.

For this multicopter comparison, normalized body thrust is exactly `[0,0,z]` in FRD with
`z in [-1,0]`. A positive body-Z thrust or any lateral body thrust is rejected rather than silently
changed. Body-rate components are FRD roll/pitch/yaw in rad/s. Torque setpoint components are
normalized PX4 body-axis coordinates in `[-1,1]`; they are not N m. Thrust setpoint components are
normalized PX4 coordinates; they are not newtons.

Pinned `VehicleRatesSetpoint.msg` contains a comment calling `thrust_body` a "body NED" frame while
also defining the angular rates as body FRD. Pinned `VehicleAttitudeSetpoint.msg` explicitly labels
the same multicopter `thrust_body` convention body FRD. The project therefore keeps its canonical
body-FRD contract and records the upstream wording inconsistency rather than introducing a second
body frame.

## Timestamp contract

`OffboardControlMode`, attitude setpoints, and rate setpoints carry the caller-supplied PX4
boot-time publish timestamp in microseconds. Torque and thrust setpoints additionally carry an
explicit `timestamp_sample`. The builder never substitutes wall time, ROS time, or a zero sentinel
for an omitted timestamp.

Pinned PX4 multicopter rate control copies `VehicleAngularVelocity.timestamp_sample` into both its
thrust and torque setpoints and uses a fresh PX4 publication timestamp. The future ROS binding must
preserve equivalent semantics for `px4_mirror`. For `lee_wrench`, which depends on multiple
validated canonical-state samples, the orchestration layer must choose and document the sample
stamp after the existing state-freshness gate; the handoff builder only preserves the supplied
stamp and rejects `timestamp_sample > timestamp`.

## Fail-closed serialization

`HandoffInputs` has no plausible numeric command defaults: unset command values are NaN and
required timestamps are optionals. A rejected build carries no active offboard field and no
setpoint payload. Attitude serialization requires an explicitly configured quaternion-norm
tolerance and rejects a non-unit quaternion instead of normalizing it silently. Values that cannot
be represented by the pinned float32 message fields are rejected.

`lee_wrench` has one additional gate: the adapter carries the `ok` result from the simulation or
hardware physical-wrench normalizer, and serialization is rejected unless that result is true.
This prevents a failed full-wrench reconstruction from becoming a zero/default normalized command.

## Layering

`controller_handoff.cpp` selects data from the already-audited controller outputs. `handoff.cpp`
validates and serializes that data into source-shaped PX4 command payloads. Neither file contains
controller equations. A thin `rclcpp`/`px4_msgs` publisher binding remains a later ROS 2 Jazzy
milestone and must be compiled and tested with the pinned generated messages before it is marked
implemented.
