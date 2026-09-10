# Requirements traceability

The verbatim user specification is retained in `docs/superpowers/specs/project_specification.md`. This table points each major requirement area to its implementation/validation authority.

| Requirement area | Authority / implementation |
|---|---|
| authorization, public destination, workflow | original specification; git history; CI |
| source audit and exact revisions | `docs/sources.md`, `.gitmodules` |
| NED/FRD frames and conversion | `docs/frames.md`, `control/include/control/frames.hpp`, frame tests |
| state sources and timestamps | `CanonicalState`, ROS state adapter, state-validity tests |
| analytical trajectory derivatives | trajectory library/tests |
| physical Lee controller | `LeeController`, `docs/control.md`, Lee tests |
| rate handoff | `GeometricRateController`, rate tests |
| PX4 thrust normalization | `Px4ThrustNormalization`, golden tests |
| exact PX4 attitude/rate mirror | `Px4AttitudeRateController`, golden tests |
| simulation physical torque normalization | F450 model/inversion, forward-reconstruction tests |
| hardware torque calibration | `experiment/calibration`, calibration schema/gate/tests |
| four mode message semantics | mode dispatcher and ROS interface tests |
| Vicon architecture | `experiment/vicon_bridge`, frame tests, experiment docs |
| Gazebo direct position | `simulation/gazebo_position`, simulation docs |
| common diagnostics | `FlightDiagnostics.msg`, diagnostics tests |
| ROS bag + ULog workflow | scripts and validation docs |
| analysis metrics/plots | `analysis/metrics.py`, `analysis/analyze_flight.py`, tests |
| safety and stale-state gates | preflight/state validation and tests |
| SITL progression | `scripts/run_sitl_validation.sh`, `docs/validation.md`, CI/manual workflow |
| hardware claims | `docs/validation.md`; hardware remains unvalidated without physical evidence |
| public repository hygiene/license | BSD-3-Clause, `.gitignore`, CI checks |
