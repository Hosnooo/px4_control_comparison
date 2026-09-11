# PX4 Offboard Controllers Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor `px4_control_comparison` into the readable, multirotor-general `px4_offboard_controllers` ROS 2 package described by the approved redesign spec, preserving audited controller behavior while adding first-class position, velocity, and geometric-acceleration paths.

**Architecture:** Use one top-level `ament_cmake` package with narrow internal CMake libraries. Keep controller math ROS-independent, isolate PX4 message construction in a thin adapter, keep Gazebo/Vicon/F450 as optional support layers, and use native `/fmu/in/*` and `/fmu/out/*` topics at the PX4 boundary. Internal controller stages communicate through C++ values, never project-specific ROS plumbing topics.

**Tech Stack:** C++17, CMake/ament_cmake, ROS 2 Jazzy, `px4_msgs`, `rclcpp`, `geometry_msgs`, Gazebo Harmonic transport/messages through vendor packages, `ros_gz_bridge`, Python 3 launch/tests.

**Spec:** `docs/superpowers/specs/2026-09-10-px4-offboard-controllers-redesign.md`

## Global Constraints

- Work locally for all source changes and keep source code off the remote until the complete local verification gate passes.
- Recovery checkpoint exception: at the user's explicit request, push one documentation-only checkpoint containing the approved spec and this plan before implementation.
- After that checkpoint, produce exactly one coherent implementation commit for the source refactor; do not create additional intermediate GitHub commits or CI scratch runs.
- Canonical world/body frames remain NED/FRD; frame conversion happens exactly once at explicit boundaries.
- `px4_position`, `px4_velocity`, `geometric_acceleration`, `geometric_attitude`, `geometric_rate`, `px4_attitude_rate_mirror`, and `lee_wrench` are the supported public controller selections.
- Use native generated `px4_msgs` and native `/fmu/in/*`/`/fmu/out/*` topics at the PX4 boundary.
- Do not add default `/custom/*` topics for internal controller plumbing.
- `ros_gz_bridge` is an installed package dependency and must not become a Git submodule.
- Generic core/controllers/PX4 code must not depend on F450, hardware calibration, Gazebo, or Vicon implementation code.
- Public APIs must document purpose, required state/reference fields, frames/units, outputs, remaining PX4-owned loops, and source/equation behavior.
- Implementation comments explain equations, frame/sign conversions, frozen PX4 behavior, normalization/saturation, timestamp ownership, and fail-closed decisions; they do not narrate obvious assignments/control flow.
- Preserve the exact audited source pins from the approved spec.
- Do not claim native Jazzy/Harmonic or SITL validation unless it is actually run on that environment.

---

## File Structure Locked by This Plan

The implementation owns files through these targets and directories:

- `include/px4_offboard_controllers/core/`, `src/core/` -> `px4_offboard_core`: math primitives, frames, state, trajectory.
- `include/px4_offboard_controllers/controllers/`, `src/controllers/` -> `px4_offboard_controllers_lib`: geometric acceleration/attitude/rate, Lee SE(3), PX4 attitude/rate mirror math.
- `include/px4_offboard_controllers/wrench/` -> generic physical-wrench normalization contract.
- `include/px4_offboard_controllers/px4/`, `src/px4/` -> `px4_offboard_px4`: controller-domain command types and conversion to generated `px4_msgs`.
- `include/px4_offboard_controllers/state_sources/`, `src/state_sources/` -> `px4_offboard_state_sources`: Gazebo direct-position pure adapter and source selection.
- `src/ros2/` -> `px4_offboard_ros2`: ROS subscriptions/publishers/orchestration and Gazebo relay executable.
- `vehicles/f450/include/`, `vehicles/f450/src/` -> `px4_offboard_f450`: F450-only wrench reconstruction.
- `experiment/calibration/` -> hardware calibration fitter and documentation.
- `research/control_boundary_comparison/` -> preserved mapping/reproducibility docs for the original four-boundary experiment.
- `test/{core,controllers,px4,state_sources,vehicles,ros2}/` -> tests aligned with component ownership.

A `.cpp` file is compiled by exactly one logical target. No target reaches sideways to compile another component's `.cpp` file.

---

### Task 1: Reconstruct and Lock the Baseline Locally

**Files:**
- Create local working tree from `Hosnooo/px4_control_comparison@986eceb5b39ec6d1ae046e51882ccbb5df922a36`.
- Copy approved spec to `docs/superpowers/specs/2026-09-10-px4-offboard-controllers-redesign.md`.
- Create this plan at `docs/superpowers/plans/2026-09-10-px4-offboard-controllers-redesign.md`.

**Interfaces:**
- Consumes: exact baseline commit `986eceb5b39ec6d1ae046e51882ccbb5df922a36`.
- Produces: clean local source snapshot; no remote mutation.

- [ ] Fetch every tracked non-submodule file from the baseline commit into the local tree and preserve `.gitmodules` entries/pins.
- [ ] Initialize local Git metadata with the baseline tree recorded as the parent-equivalent snapshot only for local diff accounting; do not push or create a remote scratch branch.
- [ ] Run the baseline host CMake/tests exactly as supported by the current environment and record any environment-only exclusions.
- [ ] Verify the local file manifest matches the baseline Git tree paths and submodule SHAs.

### Task 2: Write Structural Tests Before Renaming

**Files:**
- Create: `test/architecture/test_repository_structure.py`
- Create: `test/architecture/test_dependency_boundaries.py`

**Interfaces:**
- Consumes: baseline tree.
- Produces: executable assertions for the approved repository shape and forbidden dependency directions.

- [ ] Write a failing test that requires top-level `package.xml`, `README.md`, `include/px4_offboard_controllers`, `src/{core,controllers,px4,state_sources,ros2}`, `vehicles/f450`, `third_party`, `research/control_boundary_comparison`, and forbids the nested `simulation/ros2/package.xml` architecture.
- [ ] Run `python3 -m unittest test.architecture.test_repository_structure -v` and verify RED because the current tree still uses `control/`, `simulation/ros2/`, and has no top-level package manifest/README.
- [ ] Write a failing dependency-boundary test that rejects generic includes containing `f450`, `hardware_wrench_calibration`, direct Gazebo headers in controller/PX4 headers, `/custom/` controller topics, and sibling `.cpp` compilation paths.
- [ ] Run the dependency-boundary test and verify RED against the baseline coupling.

### Task 3: Rename Package and Build Graph Without Changing Controller Math

**Files:**
- Create/Modify: `CMakeLists.txt`, `package.xml`, `README.md`.
- Move/create headers/sources under the locked layout.
- Modify: `.gitmodules` paths from `dependencies/*` to `third_party/*` while preserving URLs and commit SHAs.
- Create: `third_party/README.md`.
- Create: `docs/dependencies.md`.

**Interfaces:**
- Consumes: existing controller/state/simulation sources.
- Produces: targets `px4_offboard_core`, `px4_offboard_controllers_lib`, `px4_offboard_px4`, `px4_offboard_state_sources`, `px4_offboard_f450`, ROS executables when dependencies are available.

- [ ] Move files into component-owned directories and update namespaces/includes to `px4_offboard` / `px4_offboard_controllers/...`.
- [ ] Split CMake targets so each `.cpp` has one owner and lower-level libraries have no ROS/Gazebo/F450 implementation dependency.
- [ ] Add top-level ament package metadata with runtime/build dependency categories from the spec, including `ros_gz_bridge` as an installed runtime dependency.
- [ ] Update `.gitmodules` path names to `third_party/*` without changing pinned URLs/revisions.
- [ ] Make the structural tests GREEN.
- [ ] Run host CMake configure/build/tests to verify behavior-preserving moves.

### Task 4: Split Math Primitives and Fix Invalid Indexing

**Files:**
- Create: `include/px4_offboard_controllers/core/vector3.hpp`
- Create: `include/px4_offboard_controllers/core/matrix3.hpp`
- Create: `include/px4_offboard_controllers/core/quaternion.hpp`
- Create/Modify: `include/px4_offboard_controllers/core/math.hpp` as a compatibility aggregation header only if needed during migration.
- Test: `test/core/test_math.cpp`

**Interfaces:**
- Produces: `px4_offboard::Vec3`, `Mat3`, `Quat` APIs used by all ROS-independent layers.

- [ ] Write a failing unit test asserting `Vec3::at(3)` throws `std::out_of_range` and valid indices 0/1/2 return x/y/z exactly.
- [ ] Run the focused test and verify RED against the baseline silent-index behavior.
- [ ] Split the primitives without changing valid arithmetic semantics; make invalid indexing explicit via checked access and prevent unchecked out-of-range use in refactored code.
- [ ] Make the focused test GREEN and run all math/frame/controller tests.

### Task 5: Controller-Specific State and Reference Contracts

**Files:**
- Modify: `include/px4_offboard_controllers/core/state.hpp`, `src/core/state.cpp`.
- Modify: `include/px4_offboard_controllers/core/trajectory.hpp`, `src/core/trajectory.cpp`.
- Test: `test/core/test_state_requirements.cpp`, `test/core/test_trajectory.cpp`.

**Interfaces:**
- Produces: timestamped canonical state fields plus `StateRequirements`/validation API; trajectory representation that does not require unused derivatives for simple modes.

- [ ] Write failing tests proving position/velocity PX4 modes require no controller state; geometric acceleration requires only position+velocity; geometric rate requires position+velocity+attitude; Lee requires position+velocity+attitude+body rate; mirror requires its exact state subset.
- [ ] Verify RED against the all-modes validator.
- [ ] Introduce explicit per-controller state requirements and validation with no source substitution.
- [ ] Write failing tests proving a position-only or velocity-only reference can be valid without jerk/snap/yaw acceleration while a full Lee reference still validates all derivatives it consumes.
- [ ] Implement minimal reference validation profiles and make tests GREEN.

### Task 6: Add Geometric Acceleration and Preserve Existing Geometric/Lee/PX4-Mirror Math

**Files:**
- Create: `include/px4_offboard_controllers/controllers/geometric_acceleration.hpp`
- Create: `src/controllers/geometric_acceleration.cpp`
- Rename/move/comment: geometric rate, Lee, PX4 mirror headers/sources.
- Test: `test/controllers/test_geometric_acceleration.cpp` plus migrated golden tests.

**Interfaces:**
- `GeometricAccelerationController::compute(state, reference) -> AccelerationCommandNed`.
- Existing controller outputs remain ROS-independent controller-domain values.

- [ ] Write failing tests for `a_cmd = a_d - Kp*(p-p_d) - Kv*(v-v_d)`, including zero-error feed-forward and axis-independent gains.
- [ ] Verify RED because the controller does not exist.
- [ ] Implement the controller with no gravity term and document that PX4 owns gravity compensation below the acceleration setpoint boundary.
- [ ] Make tests GREEN.
- [ ] Migrate existing geometric-rate, Lee, thrust-normalization, and PX4-mirror tests without altering golden/reference behavior.
- [ ] Add concise public contract comments to every controller declaration and source/equation comments to non-obvious implementation blocks.

### Task 7: Replace Comparison Handoff With Typed PX4-Bound Commands

**Files:**
- Create: `include/px4_offboard_controllers/px4/control_level.hpp`
- Create: `include/px4_offboard_controllers/px4/command_types.hpp`
- Create: `include/px4_offboard_controllers/px4/command_builder.hpp`
- Create: `src/px4/command_builder.cpp`
- Retire old public `handoff.hpp`/`controller_handoff.hpp` after migration.
- Test: `test/px4/test_command_types.cpp`, `test/px4/test_command_builder_contract.cpp`.

**Interfaces:**
- Domain commands: `PositionCommand`, `VelocityCommand`, `AccelerationCommand`, `AttitudeCommand`, `BodyRateCommand`, `NormalizedWrenchCommand`.
- ROS adapter converts these commands to generated `px4_msgs` only in ROS-enabled builds.

- [ ] Write failing host tests for typed command validation and exact `OffboardControlLevel` mapping.
- [ ] Implement typed domain commands with finite/range/timestamp validation; no project-owned duplicate PX4 message schema.
- [ ] Write ROS-bound contract tests/stubs asserting exact destination names and exact first-active `OffboardControlMode` flags for all seven modes.
- [ ] Add generated-`px4_msgs` conversion in the ROS/PX4 adapter only, using NaN semantics for uncontrolled `TrajectorySetpoint` dimensions.
- [ ] Remove obsolete `HandoffMode` and source-shaped duplicate public message structures after all new tests are GREEN.

### Task 8: Isolate F450 and Hardware Wrench Normalization

**Files:**
- Create: `include/px4_offboard_controllers/wrench/normalization.hpp`
- Move: F450 model to `vehicles/f450/include/...` + `vehicles/f450/src/...`
- Move: hardware calibration implementation to `experiment/calibration/` or a generic calibration support target that does not leak into controller/PX4 headers.
- Test: `test/vehicles/test_f450_wrench_model.cpp`, `test/px4/test_wrench_normalization.cpp`.

**Interfaces:**
- `WrenchNormalizationResult` contains `ok`, reason, normalized body-FRD torque, normalized body-FRD thrust.
- `lee_wrench` publishes only after successful normalization.

- [ ] Write failing static test proving generic controller/PX4 headers include no F450 or calibration implementation headers.
- [ ] Move the implementations behind the generic normalization result/interface while preserving exact F450 physical reconstruction and force/moment residual checks.
- [ ] Make existing F450/calibration golden tests GREEN in their new ownership.
- [ ] Verify failed physical normalization cannot produce a PX4 wrench command.

### Task 9: Flatten Gazebo Direct-Position Support and Remove Hidden F450 Defaults

**Files:**
- Move: `simulation/include/simulation/gazebo_position.hpp` -> `include/px4_offboard_controllers/state_sources/gazebo_position.hpp`
- Move: `simulation/src/gazebo_position.cpp` -> `src/state_sources/gazebo_position.cpp`
- Move/refactor ROS relay/adapter -> `src/ros2/gazebo_pose_relay*.cpp`, `src/ros2/gazebo_position_ros.cpp`
- Move: launch file -> `launch/gazebo_direct_position.launch.py`
- Test: `test/state_sources/test_gazebo_position.cpp`, `test/ros2/test_gazebo_runtime_config.py`.

**Interfaces:**
- Gazebo source preserves exact model identity and simulation timestamp, then converts ENU position to NED exactly once.

- [ ] Write/update failing launch/config test requiring explicit `world_name` and `model_name` with no `f450_0` generic default.
- [ ] Preserve the identity relay only because Pose_V-to-single-model selection is semantically required; use stock `ros_gz_bridge` for the Gazebo-to-ROS conversion.
- [ ] Use standard `geometry_msgs` representation at the ROS boundary and keep one-way bridge direction.
- [ ] Make pure and runtime-config tests GREEN; keep native Harmonic execution marked runtime-pending in this Debian environment.

### Task 10: Add the ROS 2 Offboard Runtime Around Native PX4 Topics

**Files:**
- Create: `src/ros2/offboard_controller_node.cpp`
- Create: `src/ros2/px4_state_input.cpp` and matching private/public interface if needed.
- Create: `src/ros2/px4_command_publisher.cpp` and matching interface.
- Create: `launch/offboard_controller.launch.py`
- Create: controller YAML under `config/controllers/`.
- Test: `test/ros2/test_native_px4_topics.py` plus compile/stub tests.

**Interfaces:**
- Publishes only native PX4 command topics from the spec.
- Subscribes to native PX4 state topics required by the selected controller.
- No ROS topics between trajectory generation, controller math, and command construction in the default in-process path.

- [ ] Write failing static/runtime-config tests that enumerate allowed `/fmu/in/*`, `/fmu/out/*` names and reject `/custom/` controller topics.
- [ ] Implement controller selection/orchestration with explicit required parameters and no airframe-specific defaults.
- [ ] Configure publishers/subscribers using native `px4_msgs` and sensor-data QoS where appropriate.
- [ ] Keep arming/mode-change `VehicleCommand` actions explicit and outside controller math.
- [ ] Make ROS adapter compile against audited Jazzy/PX4 stubs when native dependencies are absent locally.

### Task 11: Preserve Research Comparison and Rewrite User-Facing Documentation

**Files:**
- Create: `research/control_boundary_comparison/README.md`
- Rewrite: `README.md`, `docs/architecture.md`, `docs/controllers.md`, `docs/dependencies.md`, `docs/px4_interface.md`, `docs/gazebo_position.md`, `docs/physical_wrench_normalization.md`, `docs/validation.md`, `docs/sources.md`, `docs/engineering_guidelines.md`.

**Interfaces:**
- Produces: onboarding path from controller selection to build/run, with deeper provenance docs linked rather than required for basic use.

- [ ] Document controller table, required state, output message, PX4-owned lower loops, frames, and runtime validation status.
- [ ] Document dependency categories and exact pinned external repositories, explicitly stating `ros_gz_bridge` is installed rather than vendored.
- [ ] Preserve source provenance for Lee, PX4 mirror, F450 model, Gazebo direct-position semantics, and inaccessible SLS embedded PX4 fork caveat.
- [ ] Add research mapping for the original four boundaries.
- [ ] Run link/name/topic static checks to ensure stale `px4_control_comparison*`, generic `/custom/`, and hidden F450 defaults are gone from public runtime paths.

### Task 12: Full Local Verification, Single Commit, and Remote Verification

**Files:** all changed files.

**Interfaces:** final implementation tree.

- [ ] Run GCC configure/build/CTest with warnings-as-errors where supported.
- [ ] Run Clang configure/build/CTest.
- [ ] Run AddressSanitizer/UndefinedBehaviorSanitizer host tests.
- [ ] Run Python tests for calibration, launch/config, architecture, and dependency boundaries.
- [ ] Run `clang-format`/format verification on changed C++ files and compile with no warnings.
- [ ] Verify no generic headers include F450/calibration/Gazebo implementation dependencies, no sibling `.cpp` compilation exists, no default internal `/custom/` topic exists, and `ros_gz_bridge` is absent from `.gitmodules` but present in package dependencies.
- [ ] Verify all old source pins remain exact and the direct Gazebo position path still performs one ENU-to-NED conversion.
- [ ] Inspect the full staged diff for readability, public API comments, accidental files, generated artifacts, and stale naming.
- [ ] Create exactly one source-refactor commit with message `refactor: redesign PX4 offboard controller architecture`.
- [ ] Push exactly that implementation commit to `implementation` without force, on top of the documentation-only recovery checkpoint.
- [ ] Verify remote HEAD SHA, parent equals the documentation-checkpoint SHA, commit message, changed-file set, and compare against baseline `986eceb5b39ec6d1ae046e51882ccbb5df922a36`.
- [ ] Do not rename the GitHub repository or switch the default branch automatically unless a repository-administration capability is explicitly available and the user separately directs that final hosting migration.
