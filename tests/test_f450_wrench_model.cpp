#include "control/f450_wrench_model.hpp"
#include "test_support.hpp"

#include <array>
#include <limits>
#include <string>

using namespace control;

namespace {

void checkVecNear(const Vec3 &actual, const Vec3 &expected, double tolerance,
                  const std::string &message) {
  checkNear(actual.x, expected.x, tolerance, message + " x");
  checkNear(actual.y, expected.y, tolerance, message + " y");
  checkNear(actual.z, expected.z, tolerance, message + " z");
}

void checkArrayNear(const std::array<double, 4> &actual,
                    const std::array<double, 4> &expected, double tolerance,
                    const std::string &message) {
  for (std::size_t index = 0; index < actual.size(); ++index) {
    checkNear(actual[index], expected[index], tolerance,
              message + " [" + std::to_string(index) + "]");
  }
}

}  // namespace

int main() {
  const F450WrenchConfig config = frozenF450WrenchConfig();
  const F450WrenchModel model(config);

  checkNear(config.allocator_rotor_position_frd_m[0].x, 0.159, 0.0,
            "PX4 allocator arm remains source-exact");
  checkNear(config.physical_rotor_position_frd_m[0].x, 0.1626345596714, 0.0,
            "Gazebo physical arm remains distinct from allocator geometry");

  const auto hover = model.forward({}, 0.60);
  check(hover.ok, hover.reason);
  checkArrayNear(hover.motor_command, {0.60, 0.60, 0.60, 0.60}, 1e-12,
                 "symmetric hover motor command");
  checkArrayNear(hover.rotor_speed_radps, {660.0, 660.0, 660.0, 660.0}, 1e-12,
                 "sourced hover rotor speed");
  checkNear(hover.collective_thrust_n, 20.9088, 1e-9,
            "quadratic Gazebo thrust at nominal PX4 hover command");
  checkVecNear(hover.body_moment_frd_nm, {}, 1e-12, "symmetric hover moment");
  for (double thrust : {0.20, 0.80}) {
    const auto symmetric = model.forward({}, thrust);
    check(symmetric.ok, symmetric.reason);
    checkArrayNear(symmetric.motor_command, {thrust, thrust, thrust, thrust}, 1e-12,
                   "zero-moment symmetry across collective commands");
    checkVecNear(symmetric.body_moment_frd_nm, {}, 1e-12,
                 "symmetric collective has zero physical moment");
  }

  const auto positive_roll = model.forward({0.01, 0.0, 0.0}, 0.60);
  check(positive_roll.ok, positive_roll.reason);
  checkArrayNear(positive_roll.motor_command,
                 {0.592928932188, 0.607071067812, 0.607071067812, 0.592928932188},
                 1e-12, "PX4 normalized roll allocation");
  checkVecNear(positive_roll.body_moment_frd_nm, {0.061934399999, 0.0, 0.0},
               1e-10, "positive physical roll moment");

  const auto positive_pitch = model.forward({0.0, 0.01, 0.0}, 0.60);
  check(positive_pitch.ok, positive_pitch.reason);
  checkVecNear(positive_pitch.body_moment_frd_nm, {0.0, 0.061934399999, 0.0},
               1e-10, "positive physical pitch moment");

  const auto positive_yaw = model.forward({0.0, 0.0, 0.01}, 0.60);
  check(positive_yaw.ok, positive_yaw.reason);
  checkVecNear(positive_yaw.body_moment_frd_nm, {0.0, 0.0, 0.007378272},
               1e-12, "positive physical yaw moment");

  const auto zero_moment = model.normalize(hover.collective_thrust_n, {}, 0.60);
  check(zero_moment.ok, zero_moment.reason);
  checkVecNear(zero_moment.normalized_torque_frd, {}, 1e-12,
               "zero physical moment needs zero normalized torque");
  checkNear(zero_moment.normalized_thrust_body_frd.z, -0.60, 1e-12,
            "common normalized thrust remains unchanged");
  checkNear(zero_moment.collective_force_residual_n, 0.0, 1e-9,
            "zero-moment full-wrench reconstruction");

  const auto force_mismatch = model.normalize(19.84081428, {}, 0.60);
  check(!force_mismatch.ok && force_mismatch.status == F450WrenchStatus::infeasible,
        "collective-force mismatch must fail closed in simulation");
  checkNear(force_mismatch.collective_force_residual_n, 1.06798572, 1e-9,
            "rejected simulation result retains force residual diagnostics");

  const Vec3 known_torque{0.02, -0.015, 0.01};
  const auto combined_forward = model.forward(known_torque, 0.60);
  check(combined_forward.ok, combined_forward.reason);
  const Vec3 combined_moment = combined_forward.body_moment_frd_nm;
  const auto combined = model.normalize(combined_forward.collective_thrust_n,
                                        combined_moment, 0.60);
  check(combined.ok, combined.reason);
  checkVecNear(combined.normalized_torque_frd, known_torque, 1e-10,
               "conditional solve recovers the normalized torque");
  checkVecNear(combined.reconstructed_body_moment_frd_nm, combined_moment,
               config.moment_tolerance_nm, "combined physical moment reconstruction");
  checkNear(combined.reconstructed_collective_thrust_n,
            combined_forward.collective_thrust_n, 1e-9,
            "combined physical collective reconstruction");
  checkNear(combined.collective_force_residual_n, 0.0, 1e-9,
            "combined full-wrench force residual");

  const auto combined_force_mismatch = model.normalize(20.0, combined_moment, 0.60);
  check(!combined_force_mismatch.ok &&
            combined_force_mismatch.status == F450WrenchStatus::infeasible,
        "mixed-axis moment solution cannot hide a collective-force mismatch");

  const auto repeated = model.normalize(combined_forward.collective_thrust_n,
                                        combined_moment, 0.60);
  checkVecNear(repeated.normalized_torque_frd, combined.normalized_torque_frd, 0.0,
               "conditional solve is deterministic");

  const auto invalid_force = model.normalize(
      std::numeric_limits<double>::quiet_NaN(), {}, 0.60);
  check(!invalid_force.ok && invalid_force.status == F450WrenchStatus::invalid_input,
        "non-finite collective demand is rejected");

  const auto invalid_thrust = model.normalize(20.0, {}, 1.01);
  check(!invalid_thrust.ok && invalid_thrust.status == F450WrenchStatus::invalid_input,
        "out-of-range normalized thrust is rejected");
  check(!model.forward({1.01, 0.0, 0.0}, 0.60).ok,
        "forward model rejects normalized torque outside PX4 bounds");

  const auto infeasible = model.normalize(20.0, {100.0, 0.0, 0.0}, 0.60);
  check(!infeasible.ok && infeasible.status == F450WrenchStatus::infeasible,
        "moment request requiring allocator desaturation is rejected");

  F450WrenchConfig singular_config = config;
  singular_config.physical_rotor_position_frd_m.fill({});
  singular_config.physical_yaw_moment_ratio.fill(0.0);
  const auto singular =
      F450WrenchModel(singular_config).normalize(20.0, {0.1, 0.0, 0.0}, 0.60);
  check(!singular.ok && singular.status == F450WrenchStatus::singular_jacobian,
        "singular physical moment map is rejected");

  F450WrenchConfig iteration_limited_config = config;
  iteration_limited_config.max_iterations = 1;
  iteration_limited_config.moment_tolerance_nm = 1e-14;
  const auto iteration_limited = F450WrenchModel(iteration_limited_config).normalize(
      20.0, combined_moment, 0.60);
  check(!iteration_limited.ok &&
            iteration_limited.status == F450WrenchStatus::no_convergence,
        "iteration limit has a distinct deterministic rejection status");

  return 0;
}
