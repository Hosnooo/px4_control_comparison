# Source provenance

This document is the provenance authority for implementation details. The exact project requirements are retained verbatim in `docs/superpowers/specs/project_specification.md`.

## Frozen runtime baseline

| Component | Repository | Revision | Role |
|---|---|---|---|
| PX4 | `ANCL/PX4-Autopilot` | `c0a1a2ecbb4c36648e12b69e7db1f29fd335f2dc` | Flight stack, control semantics, F450 airframe, allocation |
| PX4 ROS messages | `ANCL/px4_msgs` | `392e831c1f659429ca83902e66820d7094591410` | ROS 2 message definitions matching the lab PX4 lineage |
| Vicon receiver | `ANCL/ros2-vicon-receiver` | `49a026301e0f009e0ae9b21f86bed1e7cae73f0d` | Experiment pose source |
| Micro XRCE-DDS Agent | `ANCL/Micro-XRCE-DDS-Agent` | `73622810d984349b80bbac0ef55fc0b694d62222` (`v2.4.3`) | ROS 2/PX4 DDS transport |
| F450 Gazebo models | `ANCL/PX4-gazebo-models` | `211175bba52482b8c43975e919f4d93aa2f51a4f` | Nested PX4 submodule selected by the frozen PX4 revision |

Supported host baseline: Ubuntu 24.04 LTS with ROS 2 Jazzy. This matches the audited lab workspace and the pinned Vicon receiver's documented Jazzy/Noble support.

`px4_ros_com` is not a runtime dependency of this project. Its lab pin (`ee2b41d808f31648a8094906745954298f6a6594`) was audited, but frame conversion is project-owned and tested so the controller does not depend on unrelated helper code.

## Why PX4 `c0a1a2e...` was selected

The `add_attitude_and_rate_control` workspace branch pins PX4 `aaf993e1f8ff1a4a80af69d22103ec020573d4ec`. The lab `main` revision `c0a1a2ecbb4c36648e12b69e7db1f29fd335f2dc` is 19 commits ahead of that revision. The compared changes add the Gazebo F450 airframe/model lineage and lab flight-mode work; they do not modify the standard multicopter attitude controller or the standard rate-control library used by this comparison. Selecting `c0a1a2e...` therefore retains the relevant inner-loop behavior while providing the exact F450 simulation lineage required by this repository.

## Reference-only sources

| Source | Revision | What was consulted | Use policy |
|---|---|---|---|
| `yliu213/SLSoffset` | branch `offsetQSF`; embedded PX4 pin `4f23cd316d3d043e850ff96ece07556a36ecf885` | Existing Lee/SLS behavior, F450 simulation, attitude/rate/wrench handoffs, SITL wrench inverse | Reference only. Its equations and inverse wrench mapping are not treated as authority. |
| `ANCL/fy690s_ws` | `main` and `add_attitude_and_rate_control`; branch tip observed `aa06027518b3732358261800ac5675838563a9e7` | Lab dependency pins, Vicon bridge, experiment history, F450/PX4 work | Reference only; dependencies are pinned directly. |
| `Jaeyoung-Lim/mavros_controllers` | `8b3fff0327b56c415aa24708bea5f37d76307404` | Geometric attitude-error sign cross-check | Reference only. Its moment-to-rate change and empirical thrust map are not mathematical truth. |
| Lee, Leok, McClamroch, CDC 2010 | DOI `10.1109/CDC.2010.5717652` | Equations (2)-(5), (6)-(14), especially physical thrust (12), moment (13), desired thrust direction (14) | Primary mathematical authority for `LeeController`. |

## Exact upstream files audited

### PX4 attitude and rate control

- `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`
  - reduced desired attitude using current/desired body-Z;
  - opposite-thrust-direction corner case;
  - yaw weighting and yaw gain compensation;
  - quaternion error `2 * canonical(q^-1 q_d).imag()`;
  - world-Z yaw-rate feed-forward expressed in body coordinates;
  - per-axis rate limiting.
- `src/lib/rate_control/rate_control.cpp`
  - normalized torque PID+FF;
  - derivative term consumes supplied angular acceleration;
  - integrator reduction for large rate error;
  - allocator-saturation anti-windup;
  - integrator limits.
- `src/modules/mc_rate_control/MulticopterRateControl.cpp/.hpp`
  - exact state inputs and `dt` behavior;
  - `VehicleAngularVelocity` source;
  - yaw torque low-pass filter;
  - optional battery scaling;
  - allocator saturation feedback.
- `src/modules/mc_rate_control/mc_rate_control_params.c`
  - default controller parameters and `MC_YAW_TQ_CUTOFF`.

### PX4 thrust and allocation

- `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`
- `src/modules/mc_pos_control/PositionControl/ControlMath.cpp`
  - acceleration-to-normalized-thrust logic;
  - hover-thrust/gravity scaling;
  - tilt limiting and body-Z construction;
  - normalized negative body-Z thrust convention.
- `src/lib/control_allocation/control_allocation/ControlAllocation.cpp`
- `src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp`
- `src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp`
- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMultirotor.*`
  - effectiveness and control-allocation normalization;
  - sequential desaturation behavior and priority.

### PX4 DDS/message semantics

Pinned `px4_msgs` definitions establish:

- `VehicleAttitude.q`: Hamilton `[w,x,y,z]`, body FRD to world NED.
- `VehicleAngularVelocity.xyz`: bias-corrected body-FRD rad/s.
- `VehicleAngularVelocity.xyz_derivative`: body-FRD rad/s².
- `VehicleAttitudeSetpoint.thrust_body`: normalized body-FRD thrust, multicopter collective normally negative Z.
- `VehicleRatesSetpoint`: body rates in FRD rad/s plus normalized thrust.
- `VehicleTorqueSetpoint.xyz`: normalized body-axis torque, not N·m.
- `VehicleThrustSetpoint.xyz`: normalized body-axis thrust, not newtons.
- `ControlAllocatorStatus`: allocation achievement/unallocated control and per-actuator saturation.
- `ActuatorMotors.control`: normalized actuator command.

At the frozen PX4 revision, `src/modules/uxrce_dds_client/dds_topics.yaml` exposes the required command inputs, but `vehicle_angular_velocity` is commented out in the publication list and several downstream comparison topics are not exported. This repository therefore carries an explicit source patch for research observability rather than silently changing the baseline or manufacturing derivatives.

## F450 simulation provenance

Frozen model: `PX4-gazebo-models@211175bba52482b8c43975e919f4d93aa2f51a4f`, `models/f450/model.sdf`.

Simulation-only quantities traced from that model include:

- mass: `2.0232 kg`;
- inertia diagonal: `[0.0206535, 0.0206535, 0.040464] kg m²`;
- rotor XY coordinates: `±0.1626345596714 m` in the model axes;
- maximum rotor velocity: `1032 rad/s`;
- motor thrust constant: `1.2e-5` in the Gazebo motor model;
- motor moment constant: `0.0137`;
- motor time constants: `0.0125 s` up/down;
- rotor directions: motors 0/1 CCW, 2/3 CW in the audited SDF.

These are never hardware defaults. The hardware wrench path requires measured calibration.

## Vicon provenance

The pinned receiver publishes `geometry_msgs/msg/PoseStamped` and documents ROS 2 Jazzy on Ubuntu 24.04. The lab bridge was consulted as behavioral precedent: pose is converted ENU/FLU to NED/FRD and sent as PX4 `VehicleOdometry`, while velocity and angular velocity are left invalid. This project owns and tests the conversion and additionally publishes direct position for the controller.

## Lee paper convention translation

The paper defines `R` as body to inertial, body angular velocity in the body frame, and dynamics

`m v_dot = m g e3 - f R e3` and `J Omega_dot + Omega x J Omega = M`.

Its propeller thrust acts along body `-b3`. This maps naturally to this project's NED/FRD convention: `e3` is Down and rotor force acts along body `-Z` (Up). We therefore retain the paper's signs directly after making frame names explicit.

For `e_x = x - x_d`, `e_v = v - v_d`:

`A = -k_x e_x - k_v e_v - m g e3 + m x_ddot_d`

`f = -A dot (R e3)`

`b3_d = -A / ||A||`

`e_R = 0.5 * vee(R_d^T R - R^T R_d)`

`e_Omega = Omega - R^T R_d Omega_d`

`M = -k_R e_R - k_Omega e_Omega + Omega x J Omega - J(hat(Omega) R^T R_d Omega_d - R^T R_d Omega_dot_d)`.

All code comments refer back to these equations; no ENU/FLU quantity enters `LeeController`.
