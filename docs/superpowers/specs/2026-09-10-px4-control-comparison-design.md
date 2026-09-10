# PX4 Control Comparison — frozen design

This design implements the already-approved project specification retained alongside this file. It does not introduce a manager/factory/plugin architecture.

## Frozen decisions

1. Canonical controller math is NED world / FRD body. `R_ned_frd` maps body vectors to world.
2. Direct external position is the primary position feedback: Gazebo/model in simulation, Vicon in experiment. PX4 EKF provides velocity; PX4 estimator provides attitude; PX4 provides body angular velocity and, for the mirror, its published angular-velocity derivative.
3. One `TrajectoryReference`, one `CanonicalState`, one controller executable, one diagnostics schema.
4. Four modes only: `attitude_handoff`, `rate_handoff`, `px4_mirror`, `lee_wrench`.
5. `LeeController` produces physical force/moment and is ROS-free. `GeometricRateController` is separate and is not mislabeled as exact Lee moment control.
6. `Px4ThrustNormalization` follows the frozen PX4 position-control logic. `Px4AttitudeRateController` follows the frozen PX4 attitude/rate logic.
7. PX4 torque/thrust topics are normalized control coordinates. `lee_wrench` therefore has a separate physical torque normalization layer.
8. F450 model constants are simulation-only. Hardware `lee_wrench` requires measured calibration and cannot be enabled by defaults.
9. A minimal PX4 DDS-topic patch is allowed solely to expose existing uORB diagnostic/state signals needed for exact comparison. The base revision remains pinned and the patch is explicit/reviewable.
10. ROS 2 Jazzy / Ubuntu 24.04 is the supported environment. Micro XRCE-DDS Agent is pinned to lab `v2.4.3` commit `73622810...`.

## File boundaries

- `control/include/control/math.hpp`: small fixed-size vector/matrix/quaternion math required by the ROS-independent core.
- `frames.*`: frame conversions only.
- `trajectory.*`: analytic references only.
- `state.*`: canonical state/reference structures and validity.
- `lee_controller.*`: paper controller only.
- `geometric_rate_controller.*`: desired attitude to body-rate command only.
- `px4_thrust_normalization.*`: frozen PX4 acceleration/thrust mapping only.
- `px4_attitude_rate_controller.*`: frozen PX4 attitude/rate mirror only.
- `physical_torque_normalization.*`: simulation and measured-hardware physical-to-normalized conversion only.
- ROS source files are adapters/orchestration; long equations stay in the pure classes.

## Verification strategy

Host-side CTest verifies math, frames, trajectory, state gates, Lee control, PX4 normalization/mirror, F450 inversion and hardware calibration without ROS. ROS 2 CI on Ubuntu 24.04/Jazzy builds packages and runs interface tests. A manually-dispatched SITL workflow performs progressive F450 checks where GitHub runner resources permit. No CI job claims to validate Vicon or real-aircraft hardware.
