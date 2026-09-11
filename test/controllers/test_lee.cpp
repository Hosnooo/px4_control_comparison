#include "px4_offboard_controllers/controllers/lee.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace px4_offboard;

namespace {

void vecNear(const Vec3 &actual, const Vec3 &expected, double tolerance,
             const std::string &message) {
  checkNear(actual.x, expected.x, tolerance, message + " x");
  checkNear(actual.y, expected.y, tolerance, message + " y");
  checkNear(actual.z, expected.z, tolerance, message + " z");
}

double matMaxAbs(const Mat3 &matrix) {
  double value = 0.0;
  for (double entry : matrix.a) {
    value = std::max(value, std::abs(entry));
  }
  return value;
}

CanonicalState hoverState() {
  CanonicalState state{};
  state.position_ned = Vec3{0.0, 0.0, -1.0};
  state.velocity_ned = Vec3{};
  state.attitude_ned_frd = Quat{};
  state.body_rate_frd = Vec3{};
  return state;
}

TrajectoryReference hoverReference(Vec3 position_ned_m, double yaw_rad) {
  TrajectoryReference reference{};
  reference.position = position_ned_m;
  reference.velocity = Vec3{};
  reference.acceleration = Vec3{};
  reference.jerk = Vec3{};
  reference.snap = Vec3{};
  reference.yaw = yaw_rad;
  reference.yaw_rate = 0.0;
  reference.yaw_accel = 0.0;
  return reference;
}

TrajectoryReference figureEightReference(Vec3 center_ned_m, double x_amplitude_m,
                                         double y_amplitude_m, double angular_rate_radps,
                                         double yaw_rad, double elapsed_s) {
  const double phase_rad = angular_rate_radps * elapsed_s;
  const double double_phase_rad = 2.0 * phase_rad;
  const double rate2 = angular_rate_radps * angular_rate_radps;
  const double rate3 = rate2 * angular_rate_radps;
  const double rate4 = rate3 * angular_rate_radps;

  TrajectoryReference reference{};
  reference.position =
      center_ned_m + Vec3{x_amplitude_m * std::sin(phase_rad),
                         y_amplitude_m * std::sin(double_phase_rad), 0.0};
  reference.velocity =
      Vec3{x_amplitude_m * angular_rate_radps * std::cos(phase_rad),
           2.0 * y_amplitude_m * angular_rate_radps * std::cos(double_phase_rad), 0.0};
  reference.acceleration =
      Vec3{-x_amplitude_m * rate2 * std::sin(phase_rad),
           -4.0 * y_amplitude_m * rate2 * std::sin(double_phase_rad), 0.0};
  reference.jerk = Vec3{-x_amplitude_m * rate3 * std::cos(phase_rad),
                        -8.0 * y_amplitude_m * rate3 * std::cos(double_phase_rad), 0.0};
  reference.snap = Vec3{x_amplitude_m * rate4 * std::sin(phase_rad),
                        16.0 * y_amplitude_m * rate4 * std::sin(double_phase_rad), 0.0};
  reference.yaw = yaw_rad;
  reference.yaw_rate = 0.0;
  reference.yaw_accel = 0.0;
  return reference;
}

}  // namespace

int main() {
  bool threw = false;
  try {
    LeeController unconfigured(LeeConfig{});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "Lee controller requires explicit configuration");

  LeeConfig config{};
  config.mass_kg = 2.0;
  config.gravity_mps2 = 9.80665;
  config.k_position = {4.0, 4.0, 6.0};
  config.k_velocity = {3.0, 3.0, 4.0};
  config.k_attitude = {3.5, 3.5, 1.2};
  config.k_rate = {0.35, 0.35, 0.2};
  config.inertia_kgm2 = diagonal({0.02, 0.02, 0.04});
  LeeController controller(config);

  auto state = hoverState();
  auto reference = hoverReference({0.0, 0.0, -1.0}, 0.0);
  const auto output = controller.update(state, reference);
  vecNear(output.position_error_ned_m, {}, 1e-12, "hover position error");
  vecNear(output.velocity_error_ned_mps, {}, 1e-12, "hover velocity error");
  vecNear(output.force_ned_n, {0.0, 0.0, -config.mass_kg * config.gravity_mps2}, 1e-10,
          "hover control force");
  vecNear(output.desired_body_z_ned, {0.0, 0.0, 1.0}, 1e-12, "hover body z");
  checkNear(output.collective_thrust_n, config.mass_kg * config.gravity_mps2, 1e-10,
            "hover thrust");
  check(Quat::fromMat3(output.desired_rotation_ned_frd).rotationDistance({1.0, 0.0, 0.0, 0.0}) <
            1e-10,
        "hover desired attitude is level");
  vecNear(output.desired_body_rate_frd_radps, {}, 1e-10, "hover desired rate");
  vecNear(output.attitude_error, {}, 1e-10, "hover attitude error");
  vecNear(output.body_moment_frd_nm, {}, 1e-10, "hover moment");

  state.attitude_ned_frd = Quat::fromAxisAngle({1.0, 0.0, 0.0}, 0.12);
  const auto tilted = controller.update(state, reference);
  check(tilted.attitude_error.x > 0.0, "positive roll attitude error sign");
  check(tilted.body_moment_frd_nm.x < 0.0, "moment opposes positive roll error");

  reference = hoverReference({0.0, 0.0, -1.0}, 0.3);
  reference.yaw_rate = 0.7;
  reference.yaw_accel = 0.4;
  state = hoverState();
  const auto desired = controller.update(state, reference);
  state.attitude_ned_frd = Quat::fromMat3(desired.desired_rotation_ned_frd);
  state.body_rate_frd = desired.desired_body_rate_frd_radps;
  const auto feedforward = controller.update(state, reference);
  vecNear(feedforward.attitude_error, {}, 1e-9, "feedforward zero attitude error");
  vecNear(feedforward.rate_error_frd_radps, {}, 1e-9, "feedforward zero rate error");
  checkNear(feedforward.body_moment_frd_nm.z, config.inertia_kgm2(2, 2) * 0.4, 2e-8,
            "yaw angular acceleration feedforward moment");

  state = hoverState();
  state.position_ned = Vec3{0.15, -0.08, -1.1};
  state.velocity_ned = Vec3{0.12, 0.03, -0.04};
  state.attitude_ned_frd = Quat::fromAxisAngle({0.2, -0.3, 0.5}, 0.18);
  state.body_rate_frd = Vec3{0.1, -0.08, 0.05};
  const double elapsed_s = 0.73;
  const double step_s = 1e-4;
  auto dynamic = figureEightReference({0.0, 0.0, -1.0}, 1.2, 0.7, 0.6, 0.2, elapsed_s);
  dynamic.yaw_rate = 0.15;
  dynamic.yaw_accel = -0.03;
  const auto center = controller.update(state, dynamic);
  const Mat3 rotation_dot_relation =
      center.desired_rotation_ned_frd * hat(center.desired_body_rate_frd_radps);
  check(matMaxAbs(center.desired_rotation_dot - rotation_dot_relation) < 2e-10,
        "Rdot equals R hat(Omega_d)");
  check(center.force_ned_n.finite() && center.body_moment_frd_nm.finite(),
        "dynamic Lee output finite");

  const Mat3 predicted = center.desired_rotation_ned_frd + center.desired_rotation_dot * step_s +
                         center.desired_rotation_ddot * (0.5 * step_s * step_s);
  const Mat3 orthogonal = predicted.transpose() * predicted;
  check(std::abs(orthogonal(0, 0) - 1.0) < 2e-6 &&
            std::abs(orthogonal(1, 1) - 1.0) < 2e-6 &&
            std::abs(orthogonal(2, 2) - 1.0) < 2e-6,
        "analytic desired attitude derivatives preserve SO3 locally");

  threw = false;
  try {
    LeeConfig invalid = config;
    invalid.mass_kg = 0.0;
    LeeController rejected(invalid);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "invalid mass rejected");

  threw = false;
  try {
    LeeConfig invalid = config;
    invalid.inertia_kgm2 = Mat3::identity();
    invalid.inertia_kgm2(0, 1) = 10.0;
    LeeController rejected(invalid);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "non-symmetric inertia rejected");

  threw = false;
  try {
    LeeConfig invalid = config;
    invalid.k_attitude.x = -1.0;
    LeeController rejected(invalid);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "Lee gains must be positive");
  return 0;
}
