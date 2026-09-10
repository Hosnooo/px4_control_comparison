# PX4 Control Comparison Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify a public ROS 2/PX4 research repository that compares four offboard/PX4 multirotor control boundaries with common state, reference, diagnostics and F450 simulation.

**Architecture:** Pure C++ controller mathematics are independent of ROS. Thin ROS 2 adapters assemble canonical state and publish exactly one of the four audited PX4 setpoint interfaces. Simulation and experiment differ only in direct-position adapters; physical wrench normalization has separate simulation and measured-hardware authorities.

**Tech Stack:** C++17, CMake/CTest, ROS 2 Jazzy, `px4_msgs`, PX4 SITL/Gazebo Harmonic lineage, Python 3 offline analysis, GitHub Actions.

**Spec:** `docs/superpowers/specs/project_specification.md`

## Global constraints

- Preserve NED world / FRD body throughout controller mathematics.
- Never derive primary velocity from external position.
- Never create a replacement angular-acceleration derivative where the frozen PX4 mirror requires PX4's signal.
- Keep physical N/N·m distinct from normalized PX4 control coordinates.
- Never reuse Gazebo constants as hardware calibration.
- Hardware does not auto-arm by default.
- Hardware `lee_wrench` is gated on measured calibration.
- Keep only the four approved controller modes.
- Keep code literal, small and inspectable; no manager/factory/plugin framework.
- Every controller/source/constant has explicit provenance.

---

### Task 1: Freeze sources, frames and dependency pins

**Files:** `docs/sources.md`, `docs/frames.md`, `docs/architecture.md`, `docs/control.md`, `.gitmodules`, dependency gitlinks.

**Produces:** exact baseline and frame/unit authority used by all later tasks.

- [ ] Verify lab submodule SHAs and PX4 revision comparison.
- [ ] Verify message definitions and PX4 controller/allocation source files.
- [ ] Retain the original specification verbatim in-repo.
- [ ] Add exact dependency gitlinks and verify index modes are `160000`.
- [ ] Commit as the architecture/provenance milestone and push it to `implementation`.

### Task 2: Build ROS-independent controller core using TDD

**Files:** `control/include/control/*.hpp`, `control/src/*.cpp`, `tests/test_*.cpp`, root host `CMakeLists.txt`.

**Interfaces:** `TrajectoryReference`, `CanonicalState`, `LeeOutput`, `Px4ThrustOutput`, `Px4MirrorOutput`, `TorqueNormalizationResult`.

- [ ] Write frame basis/rotation/round-trip tests; run them red before frame implementation.
- [ ] Implement minimal fixed-size math and frame conversions; run green.
- [ ] Write analytic trajectory/state-freshness tests; run red, implement, run green.
- [ ] Write Lee hover/sign/feed-forward tests plus numerical checks of analytic desired-attitude derivatives; run red, implement exact equations, run green.
- [ ] Write PX4 thrust golden tests from the frozen algorithm; run red, implement, run green.
- [ ] Write geometric rate tests; run red, implement, run green.
- [ ] Write PX4 attitude/rate mirror golden tests for reduced attitude, yaw weighting, limits, PID/FF, integrator, anti-windup, angular acceleration, yaw filter, battery scaling and dt; run red, implement, run green.
- [ ] Run the complete host test suite and compiler warnings; fix root causes for any failure.
- [ ] Commit/push the verified controller-core milestone.

### Task 3: Build physical wrench normalization and hardware calibration gates

**Files:** `control/.../physical_torque_normalization.*`, `experiment/calibration/*`, related tests/config schema.

**Produces:** simulation physical-wrench inverse and hardware calibration authorization.

- [ ] Write F450 forward-model and inverse reconstruction tests from the exact model constants and PX4 allocation semantics; include saturation/infeasibility cases.
- [ ] Implement simulation physical wrench conversion without fixed N·m scaling; reject candidates requiring desaturation.
- [ ] Write hardware-calibration rejection tests for missing, malformed, simulation-tagged, singular, poor-fit and out-of-range data.
- [ ] Implement measured-calibration loader, fitting utility, schema, provenance and preflight gate.
- [ ] Run all host tests and a synthetic calibration fit round-trip.
- [ ] Commit/push the wrench/calibration milestone.

### Task 4: Add ROS 2 controller, Vicon/Gazebo adapters and four setpoint interfaces

**Files:** `control_messages`, `control/src/*_node.cpp`, `experiment/vicon_bridge`, `simulation/gazebo_position`, `launch`, `config`, `patches/px4`.

**Produces:** one mode-selectable executable for simulation/experiment and exact PX4 messages.

- [ ] Define `DirectPosition` and common `FlightDiagnostics` messages with explicit invalid/NaN fields.
- [ ] Add the explicit PX4 DDS observability patch for existing angular velocity/rate/torque/thrust/allocator/motor topics needed by the experiment.
- [ ] Implement state adapter with PX4 EKF velocity/attitude/rates and direct external position; retain timestamps and source ages.
- [ ] Implement four mode outputs and exact `OffboardControlMode` flags/message types.
- [ ] Implement Vicon PoseStamped -> direct NED position + PX4 external-vision `VehicleOdometry` without velocity estimation.
- [ ] Implement Gazebo/model pose -> the same direct-position path.
- [ ] Add output validity/bounds gates and hardware mode/calibration gates before every active publication.
- [ ] Add launch/config files and no-default-hardware-auto-arm policy.
- [ ] Add ROS interface tests and build them in Jazzy CI.
- [ ] Commit/push the ROS integration milestone.

### Task 5: Diagnostics, analysis, CI, SITL workflow and final documentation

**Files:** `analysis`, `scripts`, `.github/workflows`, remaining docs/README/license.

**Produces:** reproducible research workflow and evidence ledger.

- [ ] Populate common diagnostics for all modes; inactive fields are NaN where practical.
- [ ] Add ROS bag + PX4 ULog recording scripts/topic lists.
- [ ] Add offline metrics for position/velocity/attitude/rate errors, physical/normalized demands, saturation, timing and ages; add deterministic Python tests.
- [ ] Add concise README, simulation/experiment procedures and validation evidence table.
- [ ] Add formatting/build/unit-test CI on Ubuntu 24.04/Jazzy and manually dispatched headless SITL validation.
- [ ] Run local host verification, static repository checks and Python tests.
- [ ] Attempt ROS/SITL verification where the environment supports it; record unavailable stages as unexecuted rather than passed.
- [ ] Inspect for secrets, machine paths, bags/ULogs, build output, undocumented constants, duplicated frame conversions and stale files.
- [ ] Commit/push the final milestone, verify remote tree/CI, then fast-forward `main` only after final evidence supports it.
