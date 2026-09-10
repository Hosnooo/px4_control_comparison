# Project specification — retained requirements authority

Original user-supplied specification SHA-256: `55ff7071b27eeb527887a27b0712dc3a00d032d03d6fee31c406d477ef9411a2` (33,645 bytes). The implementation is reviewed against the original 1,274-line specification. This in-repository authority preserves every operative requirement in structured form.

## Authorization and destination

- GitHub account: `Hosnooo`.
- Public repository: `Hosnooo/px4_control_comparison`.
- Source/reference repositories are read-only references and must not be modified.
- Final work must be committed and pushed to this public repository.
- Work systematically through all tasks possible in the environment; do not stop for ordinary clarification.
- Use planning, TDD, systematic debugging, verification-before-completion and code review.
- The architecture is already approved; do not redesign it into a more complicated framework.
- Do not guess to make tests pass.
- Do not ask about choices fixed here.
- For unresolved physical/hardware quantities, do not invent values: implement calibration, validation, safety gates and documentation.

## Project objective

Build a clean ROS 2/PX4 research project comparing where the multirotor control boundary is placed between offboard ROS 2 and onboard PX4.

Exactly four primary modes:

1. `attitude_handoff`
2. `rate_handoff`
3. `px4_mirror`
4. `lee_wrench`

All four share the same trajectory/reference implementation, canonical state, outer loop where applicable, simulation vehicle, diagnostics schema and controller executable/codebase for simulation and experiment.

## Sources that must be audited

### SLSoffset

- `yliu213/SLSoffset`, important branch `offsetQSF`.
- Embedded SLS PX4 pin: `4f23cd316d3d043e850ff96ece07556a36ecf885`.
- Use to understand existing Lee/SLS behavior, F450 simulation, handoffs, normalization, experiment history and problems.
- Do not automatically trust its architecture/equations.
- Explicitly audit the SITL-specific physical wrench inverse; do not reuse it as hardware truth.

### Lab workspace

- `ANCL/fy690s_ws`.
- Audit `main`, `add_attitude_and_rate_control`, and relevant inner-loop/F450/allocation/experiment/Vicon/torque-thrust branches.
- Verify known pins: PX4 `c0a1a2ecbb4c36648e12b69e7db1f29fd335f2dc`, `px4_msgs` `392e831c1f659429ca83902e66820d7094591410`, `px4_ros_com` `ee2b41d808f31648a8094906745954298f6a6594`, Vicon receiver `49a026301e0f009e0ae9b21f86bed1e7cae73f0d`.
- Relevant branch tip observed: `aa06027518b3732358261800ac5675838563a9e7`, with PX4 `aaf993e1f8ff1a4a80af69d22103ec020573d4ec`.
- Do not choose `main` just because it is default. Compare relevant PX4 revisions and deliberately select one exact revision with matching dependencies; document choice and differences.

### Geometric controller reference

- `Jaeyoung-Lim/mavros_controllers@8b3fff0327b56c415aa24708bea5f37d76307404`.
- Secondary cross-check only.
- Its moment-to-rate substitution and empirical affine normalized-thrust mapping are not mathematical authority.

### Lee paper

- Taeyoung Lee, Melvin Leok, N. Harris McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3),” CDC 2010, DOI `10.1109/CDC.2010.5717652`.
- Retrieve/inspect the paper; do not implement equations from memory.
- Trace each implemented physical-controller equation to the paper and translate conventions explicitly to NED/FRD.

## Precision standard

No hidden approximations, undocumented assumptions, unexplained constants, or unexplained frame/unit/sign conversions.

For important parameters record whether the source is source code, PX4 parameter, simulation model, manufacturer data, measured calibration or experimental identification. Measurement uncertainty is documented rather than hidden. Simulation constants never silently become real-aircraft configuration.

## Canonical frames and units

Controller mathematics:

- world: NED;
- body: FRD;
- `R_ned_frd` maps a body-FRD vector into world NED.

Quantities:

- position NED m;
- velocity NED m/s;
- acceleration NED m/s²;
- body angular rate FRD rad/s;
- angular acceleration FRD rad/s²;
- desired/world force NED N;
- physical body moment FRD N·m;
- PX4 thrust/torque commands dimensionless normalized coordinates.

ENU/FLU conversions exist only at boundaries and must not leak into `LeeController`. Dangerous quantities use explicit frame/unit names where useful.

## Primary vehicle state

- Experiment primary position: raw Vicon, converted once to NED.
- Simulation primary position: raw Gazebo/model position, converted once to NED.
- Velocity: PX4 EKF; never differentiate Vicon/Gazebo position.
- Attitude: PX4 estimator.
- Body angular velocity: exact PX4 body-rate output appropriate to the selected version, preferably the signal consumed by PX4 rate control.
- Angular acceleration for exact mirror: same PX4 angular-velocity derivative/acceleration signal consumed by selected rate controller; do not build an independent derivative if PX4 already supplies it.
- PX4 EKF position: diagnostics only, not primary position feedback.
- Do not introduce project LPFs/numerical derivatives because old code did. Mirror only filters that exact PX4 behavior requires.
- Retain timestamps/measurement ages from independent pipelines; make source-age mismatch observable; reject stale/invalid critical state using explicit thresholds.

## TrajectoryReference

One canonical reference includes NED position, velocity, acceleration, jerk, snap, yaw, yaw rate, yaw acceleration and timestamp.

Built-in trajectories include hover, smooth/step translation, circle and dynamic comparison trajectory. Generate analytical derivatives where possible; do not numerically differentiate standard trajectories. Keep trajectory code simple.

## LeeController

ROS-independent. It must not contain ROS subscriptions, PX4 formatting, ENU conversion or actuator normalization.

Expose at least position/velocity errors, desired force, desired thrust direction, desired attitude, mathematically derived desired body rate/acceleration where required, attitude/rate errors, physical collective thrust [N] and physical body moment [N·m].

Implement the exact physical Lee moment law including required feed-forward terms. Do not label an angular-rate approximation as the exact Lee moment controller.

## GeometricRateController

Separate component used for `rate_handoff`. Desired attitude + current attitude + appropriate feed-forward information -> desired body-rate command. Document its mathematical definition/source. Do not hide it inside `LeeController`; use `mavros_controllers` only as a sign/convention cross-check.

## Px4ThrustNormalization

Mirror relevant behavior of the exact selected PX4 revision. PX4 position-control source is the authority, not an F450 inverse motor model.

Audit/reproduce applicable hover thrust, gravity normalization, tilt projection, sign, min/max thrust, horizontal/vertical limits and intentional hover-thrust estimator behavior. A simple `hover_thrust * physical_thrust/(mass*g)` ratio is explanatory only, not the whole implementation if PX4 applies constraints/projections. Write golden/numerical equivalence tests.

## PhysicalTorqueNormalization

Separate physical Lee moment [N·m] -> PX4 normalized torque component. PX4 has no universal N·m mapping in the normal multicopter rate path.

Simulation mapping must be derived from the exact selected F450 simulator/effectiveness/allocation model: rotor positions, thrust coefficients, yaw/moment coefficients, actuator command mapping and PX4 allocation normalization. Validate numerically.

Hardware must never reuse Gazebo constants. Inspect lab motor/wrench tools. Provide measured calibration/identification procedure and data format. If trustworthy data are unavailable, implement calibration tooling/configuration sanity checks and require valid measured calibration before hardware `lee_wrench`; state hardware wrench flight is unvalidated until measured.

## Px4AttitudeRateController

Offboard mirror of the selected PX4 multicopter attitude + rate behavior. Reproduce relevant attitude error, reduced attitude, yaw weighting/feed-forward, rate limits, P/I/D/FF, integrator, anti-windup, allocator saturation feedback, derivative semantics, yaw torque filter, battery scaling, dt and parameters/limits. Do not improve or simplify the mirror. If an internal signal cannot be reproduced, document the exact limitation and closest verifiable behavior without claiming exactness. Add golden/reference tests, preferably against original host-side PX4 classes where feasible.

## Four mode semantics

### attitude_handoff

Offboard: trajectory, Lee translation, desired attitude, physical collective thrust, PX4 normalized thrust. Publish `VehicleAttitudeSetpoint`. PX4: attitude, rate, allocation, motors.

### rate_handoff

Offboard: translation, desired attitude, `GeometricRateController`, normalized thrust. Publish `VehicleRatesSetpoint`. PX4: rate, allocation, motors.

### px4_mirror

Offboard: common outer/reference, desired attitude, exact mirrored PX4 attitude and rate loops, normalized thrust/torque. Publish `VehicleTorqueSetpoint` and `VehicleThrustSetpoint`. PX4: allocation, motors.

### lee_wrench

Offboard: full physical Lee translation/rotation, physical thrust [N], physical moment [N·m], PX4 thrust normalization and physical torque normalization. Publish torque/thrust setpoints. PX4: allocation, motors.

Do not add unnecessary modes.

## Exact PX4 message audit

Audit selected `px4_msgs`, not memory. Verify units/frames for `OffboardControlMode`, attitude/rates/torque/thrust setpoints, `VehicleOdometry`, vehicle attitude/local position/angular velocity, actuator/motor outputs and allocator status. Re-verify that torque/thrust setpoint topics are normalized control coordinates, not physical N/N·m.

## Experiment/Vicon architecture

Pin exact ROS 2 Vicon receiver. Project-owned `experiment/vicon_bridge/` receives Vicon pose, performs tested ENU/FLU↔NED/FRD conversion, publishes direct position used by controller and correctly formatted PX4 external-vision `VehicleOdometry`, retains timestamps and configurable rigid-body/topic/network settings. It does not estimate velocity.

Experiment state remains direct Vicon position + PX4 EKF velocity + PX4 attitude + PX4 rates + PX4 angular acceleration where required.

## Simulation architecture

Use same audited F450 lineage. Simulation publishes direct Gazebo/model position into the same canonical path; PX4 supplies EKF velocity/attitude/rates. No separate controller implementation; adapters/config differ only.

## Repository layout/policy

Use literal directories: `control`, `control_messages`, `experiment/vicon_bridge`, `experiment/calibration`, `simulation/gazebo_position`, root `launch`, `config`, `analysis`, `dependencies`, `scripts`, `tests`, `docs`, README/license/git files.

Do not create a meaningless `ros2_ws/`; do not use `third_party/`; do not create generic manager/factory/backend/utils classes without a concrete reason.

`dependencies/` contains only genuine runtime/build dependencies. Prefer exact git submodules for selected PX4, matching `px4_msgs`, `px4_ros_com` only if required, and Vicon receiver. Reference repos remain references, not runtime dependencies. Pin OS/ROS and XRCE-DDS Agent revision. Clean clone uses `git clone --recurse-submodules`.

## Code style

Prioritize readability/debugging over abstraction. One responsibility per file/class; pure controller math without ROS; callbacks translate state/messages rather than contain long equations; no giant controller node or utils file; no duplicated frame conversions, magic constants, unexplained signs, unnecessary inheritance, plugin architecture or premature generic abstractions. Use meaningful structures/enums, explicit units/frames and consistent formatting. Comments explain non-obvious reasons, frames/units/signs, source equations/PX4 adaptation and safety constraints; do not narrate obvious C++.

## Diagnostics

Common `FlightDiagnostics` for every mode; inactive quantities explicitly invalid/NaN where practical. Diagnostics do not affect control.

Record metadata (mode, sim/experiment, config identity, git revision); timestamps/ages/dt; full reference; raw external position, EKF position/velocity, attitude, body rate/acceleration; position/velocity/attitude/rate errors; desired force/body axis/attitude/rate/acceleration; physical thrust/moment; interface attitude/rate/normalized thrust/torque; available PX4 downstream rate/torque/thrust setpoints, allocator status/saturation and normalized actuator/motor commands.

## Logging and analysis

ROS 2 bag is primary unified research dataset; PX4 ULog accompanies each SITL/flight where possible. Provide simple recording scripts. Do not compute complicated metrics in real-time controller.

Offline tools compute/plot at minimum position/velocity RMSE, attitude/rate error, physical thrust/torque demand, normalized thrust/torque, motor saturation and percentage/time saturated, loop timing and source/command ages. Plots diagnose where degradation appears through hierarchy; no decorative dashboard.

## Safety / invalid state

Before active publication validate finite/fresh external position and EKF velocity, finite/normalized attitude, finite/fresh body rate, required angular acceleration, valid trajectory, finite outputs and normalized bounds. Stale thresholds are explicit.

Do not auto-arm real hardware by default. Simulation may expose explicit `auto_arm`. Hardware `lee_wrench` cannot be enabled with invented/default torque calibration.

## Required tests

Use TDD for important math/interfaces.

Frames: basis vectors, known rotations, ENU↔NED, FLU↔FRD for position/velocity/body rates/quaternions and round trips.

Lee: hover/zero-error, force sign/direction, known attitude error sign, body-rate error, moment signs, feed-forward and finite normal behavior.

PX4 thrust: golden equivalence against selected revision.

PX4 mirror: golden/reference equivalence, using upstream host class if feasible.

ROS/PX4: correct OffboardControlMode flags, outgoing message type, frame, unit, thrust sign and torque interpretation for every mode.

Diagnostics: common schema fields populated consistently.

Calibration: missing/invalid hardware calibration prevents hardware `lee_wrench`.

## SITL validation progression

Do not stop at unit tests. Build exact F450 SITL and progressively verify: PX4 boot; ROS communication; state topics; hover/reference; attitude_handoff hover/trajectory; rate_handoff hover/trajectory; px4_mirror hover/trajectory; simulation torque-normalization validation; lee_wrench hover/trajectory; cross-mode diagnostics/analysis. Inspect logs on failure; do not tune around software/frame bugs. Record exact successful commands. Use headless simulation when GUI is unavailable.

## Hardware experiment support

Implement everything verifiable without vehicle: Vicon receiver/bridge/direct position/external vision, controller subscriptions/config/diagnostics, calibration tooling, launch, preflight validation, logging, experiment procedure. Never claim real F450 validation without actual flight. Documentation distinguishes software-verified, SITL-verified, requires lab hardware and requires measured calibration.

## Documentation

Technical, concise, non-repetitive, one authority per fact. README remains short and contains purpose, four-mode table, one architecture diagram, layout, exact baseline, prerequisites, quick sim/experiment entry points, mode selection, recording, doc links and current validation status. Detailed architecture/control/frames/simulation/experiment/validation/sources documents carry their respective facts.

## Licensing/hygiene

Use permissive license, preferably BSD-3-Clause. Preserve required upstream notices for copied/adapted code; avoid unnecessary copying. Never commit passwords, private IP credentials, tokens, user-specific home paths, ROS bags, ULogs, build directories or large generated data. Provide example machine-specific configuration instead.

## CI

At minimum format check, ROS 2 build in supported environment and unit tests. Add feasible headless/manual SITL workflow. CI must not pretend to test Vicon hardware.

## Implementation order

Follow the approved dependency order: repository/audit/baseline -> frames/dependencies/data/reference/state -> Gazebo/Vicon adapters -> Lee -> PX4 thrust -> attitude handoff -> geometric rate/rate handoff -> PX4 mirror -> simulation torque -> hardware calibration -> lee_wrench -> diagnostics/logging/analysis -> unit/integration/SITL debugging -> docs -> clean clone/final verification/review -> push/inspect CI -> truthful completion report.

## Definition of done

Completion requires everything executable without lab hardware: public repo; exact dependency pins/provenance; clean ROS build; unit/frame/Lee/PX4-thrust/PX4-mirror/diagnostic tests; reproducible SITL boot/state; all four modes in SITL including validated simulation wrench mapping; experiment Vicon path; real-aircraft calibration procedure/safety gate; bag+ULog workflow; working analysis scripts; concise consistent docs; CI where supported; clean-clone reproduction; no undocumented physical constants; no hardware-validation claims without evidence.

## Final engineering trace principle

Every normalized torque/motor command must be traceable backward through PX4 allocation, normalized command, controller/mapping, error or physical Lee moment, measured state, exact source/timestamp and explicit frame conversion. For `lee_wrench`, normalized torque must additionally trace through physical torque normalization -> body moment N·m -> exact Lee equation -> exact mass/inertia/vehicle parameters -> documented source or measured calibration. If that trace is impossible, the implementation is unfinished.
