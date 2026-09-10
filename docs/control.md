# Control definitions

The exact mathematical/source provenance is in `sources.md`; frame signs are authoritative in `frames.md`.

## Lee physical controller

`LeeController` is ROS-independent and implements the physical SE(3) controller of Lee, Leok and McClamroch, CDC 2010.

With `e_x = x - x_d`, `e_v = v - v_d`, NED `e3=[0,0,1]`, body-to-world `R`, mass `m`, inertia `J`:

`A = -k_x e_x - k_v e_v - m g e3 + m a_d`

`collective_thrust_n = -A dot (R e3)`

`b3_d = -A / ||A||`.

The desired heading and `b3_d` construct `R_d`. Desired angular velocity/acceleration are obtained from analytic `R_d_dot`, `R_d_ddot` derived from the analytic trajectory/reference derivatives and nominal translational closed-loop dynamics; production code does not finite-difference measured state.

`e_R = 0.5 vee(R_d^T R - R^T R_d)`

`e_Omega = Omega - R^T R_d Omega_d`

`M = -k_R e_R - k_Omega e_Omega + Omega x J Omega - J(hat(Omega) R^T R_d Omega_d - R^T R_d Omega_dot_d)`.

The controller exposes errors, force, thrust direction, desired attitude/rates/acceleration, physical thrust and physical body moment for diagnostics.

## Geometric rate handoff

`GeometricRateController` is deliberately separate from the physical Lee moment law. It uses the SO(3) attitude error with feed-forward desired body rate:

`Omega_cmd = R^T R_d Omega_d - K_R e_R`

with explicit configurable axis limits. This is a kinematic geometric attitude-to-rate controller used only for `rate_handoff`; it is not labeled as the Lee moment controller.

## PX4 thrust normalization

`Px4ThrustNormalization` mirrors the selected PX4 position-control acceleration-to-thrust behavior rather than applying a single force ratio. The implementation includes the selected revision's gravity/hover-thrust scaling, body-Z construction, tilt limit, vertical thrust limits, horizontal thrust allocation and normalized negative body-Z convention. Golden tests use independently calculated cases from the pinned PX4 logic.

A physical force is first converted to equivalent desired acceleration using the configured mass. No simulation motor constant participates in this mapping.

## PX4 attitude/rate mirror

`Px4AttitudeRateController` mirrors the frozen PX4 multicopter attitude and rate algorithms:

- reduced attitude prioritizing current-to-desired body-Z;
- PX4 opposite-thrust-direction corner case;
- yaw weighting with yaw-gain compensation;
- canonical quaternion error;
- world-Z yaw-rate feed-forward transformed to body coordinates;
- per-axis rate limits;
- rate P/I/D/FF producing normalized torque;
- D term using PX4-provided body angular acceleration;
- integrator error-dependent reduction and configured limits;
- control-allocator saturation anti-windup;
- selected `dt` clipping semantics;
- PX4 alpha-filter behavior for yaw torque;
- optional battery scaling.

The mirror is intentionally not "improved" relative to PX4.

## Physical torque normalization (planned milestone 3)

This layer is not implemented in the current controller-core milestone. There is no universal PX4 N·m-to-normalized-torque conversion. This project therefore separates simulation and hardware authority.

### Simulation

The F450 path derives desired individual rotor thrusts from physical `[collective thrust, Mx, My, Mz]`, maps thrust to rotor speed with the frozen Gazebo motor model, maps rotor speed through the simulator actuator/ESC command relation, and then solves the frozen PX4 normalized effectiveness/allocation coordinate relationship. Forward reconstruction must recover the requested physical wrench within tolerance and the candidate must not require allocator desaturation.

### Hardware

Hardware normalization loads only measured calibration data. Calibration records identify vehicle, date, method, units, validity range, coefficient matrix, residual metrics and conditioning. Invalid, simulation-tagged, singular or out-of-range calibration disables hardware `lee_wrench`.
