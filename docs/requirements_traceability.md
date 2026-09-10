# Requirements traceability

This table connects the project requirements to their authoritative implementation or validation
location. `Implemented` means the item exists and has the listed validation. `Runtime-pending` means
the source implementation exists but still requires validation on the target ROS/Gazebo runtime.
`Planned` means the implementation belongs to a later milestone.

| Requirement area | Status | Authority / implementation |
|---|---|---|
| coding and commenting conventions | Implemented | `docs/engineering_guidelines.md`, `.clang-format` |
| source audit and exact revisions | Implemented | `docs/sources.md`, `docs/px4_handoff.md`, `.gitmodules` |
| NED/FRD frames and conversion | Implemented | `docs/frames.md`, `control/include/control/frames.hpp`, frame tests |
| canonical state sources and freshness | Implemented | `CanonicalState`, `validateState`, state tests |
| analytical trajectory derivatives | Implemented | trajectory library and tests |
| physical Lee controller | Implemented | `LeeController`, `docs/control.md`, Lee tests |
| rate handoff mathematics | Implemented | `GeometricRateController`, rate tests |
| PX4 thrust normalization | Implemented | `Px4ThrustNormalization`, source-reference tests |
| PX4 attitude/rate mirror | Implemented | `Px4AttitudeRateController`, source-reference tests |
| simulation physical wrench normalization | Implemented | `F450WrenchModel`, frozen-chain and forward-reconstruction tests, `docs/physical_wrench_normalization.md` |
| hardware wrench calibration | Implemented | `HardwareWrenchCalibration`, `experiment/calibration`, fail-closed parser/inversion/fitter tests |
| four-mode PX4 handoff contract | Implemented | `handoff`, `controller_handoff`, handoff tests, `docs/px4_handoff.md` |
| Gazebo direct-position core contract | Implemented | `simulation_position`, `test_gazebo_position`, `docs/gazebo_position.md` |
| Gazebo relay / `ros_gz_bridge` runtime binding | Runtime-pending | `simulation/ros2`, stub-interface compile/tests, Jazzy/Harmonic native build and SITL validation outstanding |
| ROS 2 `rclcpp`/`px4_msgs` transport binding | Planned | ROS node and generated-message interface tests |
| Vicon experiment adapter | Planned | `experiment/vicon_bridge`, experiment docs |
| PX4 DDS observability additions | Planned | auditable PX4 patch plus interface tests |
| common diagnostics | Planned | `FlightDiagnostics.msg`, diagnostics tests |
| ROS bag and ULog workflow | Planned | recording scripts and validation docs |
| analysis metrics and plots | Planned | `analysis/metrics.py`, `analysis/analyze_flight.py`, tests |
| SITL progression | Planned | validation scripts and manual/CI workflow |
| hardware validation claims | Planned | `docs/validation.md`; physical evidence required |
| public repository hygiene/license | Implemented | BSD-3-Clause, `.gitignore`, provenance documentation |
