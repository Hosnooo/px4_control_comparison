#include "px4_offboard_controllers/controllers/lee.hpp"

#include <cmath>
#include <stdexcept>

namespace px4_offboard {
namespace {

struct UnitKinematics {
  Vec3 value;
  Vec3 first;
  Vec3 second;
};

UnitKinematics normalizeWithDerivatives(const Vec3 &value, const Vec3 &first,
                                        const Vec3 &second) {
  const double norm = value.norm();
  if (!(norm > kEps) || !std::isfinite(norm)) {
    throw std::invalid_argument("desired direction is singular");
  }

  const Vec3 unit = value / norm;
  const double norm_dot = dot(unit, first);
  const Vec3 unit_dot = (first - unit * norm_dot) / norm;
  const double norm_ddot = dot(unit_dot, first) + dot(unit, second);
  const Vec3 unit_ddot =
      (second - 2.0 * unit_dot * norm_dot - unit * norm_ddot) / norm;
  return {unit, unit_dot, unit_ddot};
}

struct DesiredKinematics {
  Mat3 rotation;
  Mat3 rotation_dot;
  Mat3 rotation_ddot;
  Vec3 body_rate;
  Vec3 body_angular_accel;
};

DesiredKinematics desiredKinematics(const Vec3 &force, const Vec3 &force_dot,
                                    const Vec3 &force_ddot, double yaw, double yaw_rate,
                                    double yaw_accel) {
  const auto body_z = normalizeWithDerivatives(-force, -force_dot, -force_ddot);

  const Vec3 heading{std::cos(yaw), std::sin(yaw), 0.0};
  const Vec3 heading_dot{-std::sin(yaw) * yaw_rate, std::cos(yaw) * yaw_rate, 0.0};
  const Vec3 heading_ddot{-std::cos(yaw) * yaw_rate * yaw_rate -
                              std::sin(yaw) * yaw_accel,
                          -std::sin(yaw) * yaw_rate * yaw_rate +
                              std::cos(yaw) * yaw_accel,
                          0.0};

  const Vec3 cross_value = cross(body_z.value, heading);
  const Vec3 cross_dot =
      cross(body_z.first, heading) + cross(body_z.value, heading_dot);
  const Vec3 cross_ddot = cross(body_z.second, heading) +
                          2.0 * cross(body_z.first, heading_dot) +
                          cross(body_z.value, heading_ddot);
  const auto body_y = normalizeWithDerivatives(cross_value, cross_dot, cross_ddot);

  const Vec3 body_x = cross(body_y.value, body_z.value);
  const Vec3 body_x_dot =
      cross(body_y.first, body_z.value) + cross(body_y.value, body_z.first);
  const Vec3 body_x_ddot = cross(body_y.second, body_z.value) +
                           2.0 * cross(body_y.first, body_z.first) +
                           cross(body_y.value, body_z.second);

  DesiredKinematics output{};
  output.rotation = rotationFromColumns(body_x, body_y.value, body_z.value);
  output.rotation_dot = rotationFromColumns(body_x_dot, body_y.first, body_z.first);
  output.rotation_ddot = rotationFromColumns(body_x_ddot, body_y.second, body_z.second);
  output.body_rate = vee(skew(output.rotation.transpose() * output.rotation_dot));
  output.body_angular_accel =
      vee(skew(output.rotation.transpose() * output.rotation_ddot));
  return output;
}

bool positive(const Vec3 &value) {
  return value.finite() && value.x > 0.0 && value.y > 0.0 && value.z > 0.0;
}

bool validInertia(const Mat3 &inertia) {
  if (!inertia.finite()) {
    return false;
  }

  constexpr double symmetry_tolerance = 1e-12;
  if (std::abs(inertia(0, 1) - inertia(1, 0)) > symmetry_tolerance ||
      std::abs(inertia(0, 2) - inertia(2, 0)) > symmetry_tolerance ||
      std::abs(inertia(1, 2) - inertia(2, 1)) > symmetry_tolerance) {
    return false;
  }

  const double minor_1 = inertia(0, 0);
  const double minor_2 =
      inertia(0, 0) * inertia(1, 1) - inertia(0, 1) * inertia(1, 0);
  const double minor_3 = determinant(inertia);
  return minor_1 > kEps && minor_2 > kEps && minor_3 > kEps;
}

}  // namespace

LeeController::LeeController(LeeConfig config) : config_(config) {
  const bool valid = config.mass_kg > 0.0 && std::isfinite(config.mass_kg) &&
                     config.gravity_mps2 > 0.0 && std::isfinite(config.gravity_mps2) &&
                     positive(config.k_position) && positive(config.k_velocity) &&
                     positive(config.k_attitude) && positive(config.k_rate) &&
                     validInertia(config.inertia_kgm2);
  if (!valid) {
    throw std::invalid_argument("invalid Lee controller configuration");
  }
}

LeeOutput LeeController::update(const CanonicalState &state,
                                const TrajectoryReference &reference) const {
  if (!validateState(state, requirementsFor(ControllerKind::LeeWrench)) ||
      !validateReference(reference, ReferenceProfile::LeeFull)) {
    throw std::invalid_argument("invalid Lee controller state/reference");
  }

  const Vec3 position = state.position_ned->value;
  const Vec3 velocity = state.velocity_ned->value;
  const Vec3 body_rate = state.body_rate_frd->value;
  const Mat3 rotation = state.attitude_ned_frd->value.normalized().toMat3();

  LeeOutput output{};
  output.position_error_ned_m = position - *reference.position;
  output.velocity_error_ned_mps = velocity - *reference.velocity;

  // Lee et al.: A = -Kx e_x - Kv e_v - m g e3 + m a_d in the repository's NED frame.
  const Vec3 e3{0.0, 0.0, 1.0};
  output.force_ned_n =
      -hadamard(config_.k_position, output.position_error_ned_m) -
      hadamard(config_.k_velocity, output.velocity_error_ned_mps) -
      config_.mass_kg * config_.gravity_mps2 * e3 + config_.mass_kg * *reference.acceleration;

  const Vec3 body_z = rotation * e3;
  output.collective_thrust_n = -dot(output.force_ned_n, body_z);

  // Analytically differentiate the desired-force direction. No measured-state finite difference is
  // introduced here; jerk and snap come directly from the full Lee reference profile.
  const Vec3 acceleration = config_.gravity_mps2 * e3 -
                            (output.collective_thrust_n / config_.mass_kg) * body_z;
  const Vec3 velocity_error_dot = acceleration - *reference.acceleration;
  const Vec3 body_z_dot = rotation * cross(body_rate, e3);
  const Vec3 force_dot =
      -hadamard(config_.k_position, output.velocity_error_ned_mps) -
      hadamard(config_.k_velocity, velocity_error_dot) + config_.mass_kg * *reference.jerk;
  const double thrust_dot =
      -(dot(force_dot, body_z) + dot(output.force_ned_n, body_z_dot));
  const Vec3 jerk = -(thrust_dot / config_.mass_kg) * body_z -
                    (output.collective_thrust_n / config_.mass_kg) * body_z_dot;
  const Vec3 velocity_error_ddot = jerk - *reference.jerk;
  const Vec3 force_ddot =
      -hadamard(config_.k_position, velocity_error_dot) -
      hadamard(config_.k_velocity, velocity_error_ddot) + config_.mass_kg * *reference.snap;

  const auto desired = desiredKinematics(output.force_ned_n, force_dot, force_ddot,
                                         *reference.yaw, *reference.yaw_rate,
                                         *reference.yaw_accel);
  output.desired_rotation_ned_frd = desired.rotation;
  output.desired_rotation_dot = desired.rotation_dot;
  output.desired_rotation_ddot = desired.rotation_ddot;
  output.desired_body_z_ned = desired.rotation.column(2);
  output.desired_body_rate_frd_radps = desired.body_rate;
  output.desired_body_angular_accel_frd_radps2 = desired.body_angular_accel;

  // e_R and e_Omega are the physical Lee attitude/rate errors, not the PX4 rate-handoff law.
  output.attitude_error =
      0.5 * vee(desired.rotation.transpose() * rotation -
                rotation.transpose() * desired.rotation);
  const Vec3 desired_rate_in_body =
      rotation.transpose() * desired.rotation * desired.body_rate;
  output.rate_error_frd_radps = body_rate - desired_rate_in_body;

  const Vec3 angular_momentum = config_.inertia_kgm2 * body_rate;
  const Vec3 feed_forward_inside =
      cross(body_rate, desired_rate_in_body) -
      rotation.transpose() * desired.rotation * desired.body_angular_accel;
  output.body_moment_frd_nm =
      -hadamard(config_.k_attitude, output.attitude_error) -
      hadamard(config_.k_rate, output.rate_error_frd_radps) +
      cross(body_rate, angular_momentum) - config_.inertia_kgm2 * feed_forward_inside;
  return output;
}

}  // namespace px4_offboard
