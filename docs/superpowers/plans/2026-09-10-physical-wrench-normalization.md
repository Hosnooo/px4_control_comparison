# Physical Wrench Normalization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add source-traceable F450 physical-moment normalization and a measured-hardware calibration gate while preserving the common PX4 thrust mapping.

**Architecture:** `F450WrenchModel` owns the frozen PX4-allocation-to-Gazebo forward model and bounded torque solve. `HardwareWrenchCalibration` owns strict measured-record parsing, validation, and conditional torque inversion; it has no simulation fallback. A small Python fitter produces the same strict record format from measured samples.

**Tech Stack:** C++17, CMake/CTest, Python 3 with NumPy, pinned PX4/F450 sources.

**Spec:** `docs/physical_wrench_normalization.md`

## Global Constraints

- Controller mathematics remain ROS-independent and use world NED/body FRD.
- `lee_wrench` uses the common `Px4ThrustNormalization` output unchanged.
- Simulation and hardware authority remain separate; Gazebo constants never authorize hardware.
- Raw motor commands must remain in `[0, 1]`; accepted results never require PX4 desaturation.
- Physical and PX4 allocator rotor geometry remain distinct and explicit.
- All physical/normalized quantities carry units in their identifiers.
- No hidden physical constants, generic backend/factory layer, or default hardware calibration.
- The complete milestone is one substantial commit pushed to `implementation` after all gates pass.

---

### Task 1: F450 forward model and conditional torque solve

**Files:**
- Create: `control/include/control/f450_wrench_model.hpp`
- Create: `control/src/f450_wrench_model.cpp`
- Create: `tests/test_f450_wrench_model.cpp`
- Modify: `control/CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: positive `collective_thrust_n`, desired `Vec3 body_moment_frd_nm`, and common `normalized_collective_thrust`.
- Produces: `F450WrenchResult F450WrenchModel::normalize(...) const` and `F450ForwardState F450WrenchModel::forward(...) const`.

- [x] **Step 1: Define failing golden forward-model tests**

  Add tests that instantiate `frozenF450WrenchConfig()` and assert:

  ```cpp
  const auto state = model.forward({}, 0.60);
  checkVecNear(state.motor_command, {0.60, 0.60, 0.60, 0.60}, 1e-12, "hover motors");
  checkNear(state.rotor_speed_radps[0], 660.0, 1e-12, "hover speed");
  checkNear(state.collective_thrust_n, 20.9088, 1e-9, "sourced nonlinear hover force");
  checkVecNear(state.body_moment_frd_nm, {}, 1e-12, "symmetric hover moment");
  ```

  Add independent sign cases for positive normalized roll, pitch, and yaw and assert physical moment signs in FRD.

- [x] **Step 2: Run the new test red**

  Run:

  ```bash
  g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -Icontrol/include -Itests \
    control/src/*.cpp tests/test_f450_wrench_model.cpp -o /tmp/test_f450_wrench_model
  ```

  Expected: compile failure because `control/f450_wrench_model.hpp` does not exist.

- [x] **Step 3: Implement the explicit frozen configuration and forward chain**

  Define:

  ```cpp
  struct F450WrenchConfig {
    std::array<Vec3, 4> allocator_rotor_position_frd_m;
    std::array<double, 4> allocator_yaw_moment_ratio;
    std::array<Vec3, 4> physical_rotor_position_frd_m;
    std::array<double, 4> physical_yaw_moment_ratio;
    double motor_thrust_constant_n_per_radps2;
    double esc_speed_min_radps;
    double esc_speed_max_radps;
    double moment_tolerance_nm;
    double jacobian_condition_limit;
    int max_iterations;
  };
  ```

  Construct the normalized four-motor mixer using the exact pinned PX4 pseudo-inverse normalization rules. `forward()` computes raw motor commands, ESC speed, quadratic physical rotor thrust, physical collective thrust, and FRD moment without clipping.

- [x] **Step 4: Run forward-model tests green**

  Run the direct compile command and `/tmp/test_f450_wrench_model`.

  Expected: exit 0.

- [x] **Step 5: Add failing conditional-solve tests**

  Cover zero moment, each axis, simultaneous axes, deterministic repetition, collective residual, and rejection of non-finite, out-of-range thrust, saturated motor, normalized-torque overflow, singular Jacobian, and non-convergence cases. Assert the fixed-thrust invariant:

  ```cpp
  const auto result = model.normalize(19.84081428, {0.20, -0.12, 0.03}, 0.60);
  check(result.ok, result.reason);
  checkNear(result.normalized_thrust_body_frd.z, -0.60, 1e-12, "common thrust unchanged");
  checkVecNear(result.reconstructed_body_moment_frd_nm,
               {0.20, -0.12, 0.03}, config.moment_tolerance_nm, "moment reconstruction");
  checkNear(result.collective_force_residual_n,
            result.reconstructed_collective_thrust_n - 19.84081428,
            1e-12, "reported force residual");
  ```

- [x] **Step 6: Implement bounded Newton solve with backtracking**

  Solve only the three normalized torque coordinates. At each iteration compute the analytic 3-by-3 moment Jacobian through `u = mixer * [torque, thrust_z]`, `omega = omega_min + u(omega_max-omega_min)`, and `f = k omega^2`. Accept a step only if it reduces the moment residual while all motor commands and normalized torque remain within bounds. Reject with a stable enum-backed reason when a gate fails.

- [x] **Step 7: Run the new test and complete host suite green**

  Compile and run every `tests/test_*.cpp` executable under the same strict flags. Expected: all executables exit 0.

---

### Task 2: Measured hardware calibration record and fail-closed normalizer

**Files:**
- Create: `control/include/control/hardware_wrench_calibration.hpp`
- Create: `control/src/hardware_wrench_calibration.cpp`
- Create: `tests/test_hardware_wrench_calibration.cpp`
- Modify: `control/CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: strict key/value calibration file, expected vehicle ID, current UTC date, desired physical moment, common normalized thrust.
- Produces: validated `HardwareWrenchCalibration` and `HardwareWrenchResult normalize(...) const`.

- [x] **Step 1: Add failing parser and provenance tests**

  Define one valid measured record fixture and mutations that remove each required key or supply `authority=simulation`, the wrong vehicle, invalid unit strings, malformed SHA-256, future/expired dates, inadequate samples, excessive residuals, non-finite coefficients, singular torque block, bad condition number, or invalid operating ranges.

- [x] **Step 2: Run the hardware test red**

  Use the Task 1 direct compile pattern with `tests/test_hardware_wrench_calibration.cpp`.

  Expected: compile failure because the calibration interface does not exist.

- [x] **Step 3: Implement strict record parsing and validation**

  Use a documented line-oriented `key=value` record. Reject duplicate keys, unknown keys, missing keys, whitespace-only identity fields, locale-dependent numbers, and extra matrix elements. Required coefficient rows map `[1, thrust, torque_x, torque_y, torque_z]` to collective force and three moments. Validation thresholds are supplied explicitly by `HardwareCalibrationLimits`; none have permissive runtime defaults.

- [x] **Step 4: Add failing normalization tests**

  For a synthetic valid affine record, solve the moment rows at fixed normalized thrust and check exact reconstruction, collective-force residual, torque/range bounds, and deterministic rejection reasons.

- [x] **Step 5: Implement conditional affine inversion**

  Subtract the intercept and thrust contribution from desired moment, invert the measured 3-by-3 torque block, verify its condition and residual, and reject commands outside the calibrated envelope. The class cannot be constructed from an invalid record and exposes no Gazebo fallback.

- [x] **Step 6: Run hardware and complete C++ suite green**

  Expected: all strict direct-compiled executables exit 0.

---

### Task 3: Measured-data fitting utility and round trip

**Files:**
- Create: `experiment/calibration/fit_wrench_calibration.py`
- Create: `experiment/calibration/README.md`
- Create: `tests/test_fit_wrench_calibration.py`

**Interfaces:**
- Consumes CSV columns `normalized_thrust,torque_x,torque_y,torque_z,collective_thrust_n,moment_x_nm,moment_y_nm,moment_z_nm` plus required provenance arguments.
- Produces the strict `key=value` calibration record consumed by `HardwareWrenchCalibration`.

- [x] **Step 1: Add a failing synthetic-fit test**

  Generate a full-rank deterministic grid from a known 4-by-5 affine matrix, invoke the fitter, parse the output, and compare every coefficient and quality metric. Add rejection cases for missing columns, NaN, insufficient samples, singular design, excessive condition number, and residual-limit failure.

- [x] **Step 2: Run the Python test red**

  Run:

  ```bash
  python3 -m unittest tests/test_fit_wrench_calibration.py -v
  ```

  Expected: import/file failure because the fitter does not exist.

- [x] **Step 3: Implement deterministic least-squares fitting**

  Use `numpy.linalg.lstsq`, calculate singular-value condition number, per-output RMSE and maximum absolute residual, validate the measured operating envelope, and write keys in a stable order. Require all provenance and acceptance thresholds as CLI arguments.

- [x] **Step 4: Run fitter tests and C++ loader round trip green**

  Feed the generated record to a tiny test invocation of the C++ loader in addition to the Python assertions. Expected: both suites exit 0.

---

### Task 4: Documentation, audit corrections, and milestone verification

**Files:**
- Modify: `docs/control.md`
- Modify: `docs/sources.md`
- Modify: `docs/requirements_traceability.md`
- Modify: `docs/physical_wrench_normalization.md`
- Modify: `.gitignore`
- Modify: `control/src/px4_thrust_normalization.cpp`
- Modify: `tests/test_px4_thrust_normalization.cpp`

**Interfaces:**
- Produces: truthful implemented/planned status and exact verification commands.

- [x] **Step 1: Update provenance and traceability**

  Cite the exact F450 SDF, airframe 4022, allocator effectiveness/pseudo-inverse, `FunctionMotors`, `MixingOutput`, and GZ ESC bridge files. State that the lab workspace supplied a simulation sweep and wrench injector but no measured hardware calibration record.

- [x] **Step 2: Correct source-fidelity edge coverage**

  Add a test for PX4's float-epsilon near-parallel tilt behavior and either match it or document the deliberate double-precision difference. Keep the fixed PX4 one-g value explicit in every production configuration path.

- [x] **Step 3: Run all available verification gates**

  Run strict direct C++ compilation/execution, Python unit tests, `git diff --check`, submodule SHA checks, source-constant checks with `rg`, and a clean status review. Attempt CMake/CTest only if `cmake` becomes available; otherwise record it as unavailable.

- [x] **Step 4: Review the complete milestone diff**

  Check frame/sign/unit names, physical-versus-allocator geometry, rejection paths, absence of hidden hardware defaults, comments, test readability, and generated artifacts. Confirm no ROS bags, logs, build outputs, credentials, or private paths are tracked.

- [x] **Step 5: Create and publish the single milestone commit**

  ```bash
  git add control experiment tests docs .gitignore
  git commit -m "feat: add physical wrench normalization and hardware calibration"
  git push origin implementation
  git ls-remote origin refs/heads/implementation
  ```

  Expected: the remote branch SHA exactly equals the local commit SHA.
