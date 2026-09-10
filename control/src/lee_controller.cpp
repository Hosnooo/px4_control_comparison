#include "control/lee_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace control {
namespace {

struct UnitKinematics {
  Vec3 value;
  Vec3 first;
  Vec3 second;
};

UnitKinematics normalizeWithDerivatives(const Vec3 &v, const Vec3 &v_dot,
                                        const Vec3 &v_ddot) {
  const double r = v.norm();
  if (!(r > kEps) || !std::isfinite(r)) {
    throw std::invalid_argument("desired direction is singular");
  }
  const Vec3 b = v / r;
  const double r_dot = dot(b, v_dot);
  const Vec3 b_dot = (v_dot - b * r_dot) / r;
  const double r_ddot = dot(b_dot, v_dot) + dot(b, v_ddot);
  const Vec3 b_ddot = (v_ddot - 2.0 * b_dot * r_dot - b * r_ddot) / r;
  return {b, b_dot, b_ddot};
}

struct DesiredKinematics {
  Mat3 rotation;
  Mat3 rotation_dot;
  Mat3 rotation_ddot;
  Vec3 body_rate;
  Vec3 body_angular_accel;
};

DesiredKinematics makeDesiredKinematics(const Vec3 &force_ned_n, const Vec3 &force_dot,
                                        const Vec3 &force_ddot, double yaw, double yaw_rate,
                                        double yaw_accel) {
  // Lee et al. Eq. (14): b3_d = -A / ||A||. Derivatives are analytic.
  const auto b3 = normalizeWithDerivatives(-force_ned_n, -force_dot, -force_ddot);

  const Vec3 b1c{std::cos(yaw), std::sin(yaw), 0.0};
  const Vec3 b1c_dot{-std::sin(yaw) * yaw_rate, std::cos(yaw) * yaw_rate, 0.0};
  const Vec3 b1c_ddot{-std::cos(yaw) * yaw_rate * yaw_rate - std::sin(yaw) * yaw_accel,
                      -std::sin(yaw) * yaw_rate * yaw_rate + std::cos(yaw) * yaw_accel,
                      0.0};

  const Vec3 c = cross(b3.value, b1c);
  const Vec3 c_dot = cross(b3.first, b1c) + cross(b3.value, b1c_dot);
  const Vec3 c_ddot = cross(b3.second, b1c) + 2.0 * cross(b3.first, b1c_dot) +
                      cross(b3.value, b1c_ddot);
  const auto b2 = normalizeWithDerivatives(c, c_dot, c_ddot);

  const Vec3 b1 = cross(b2.value, b3.value);
  const Vec3 b1_dot = cross(b2.first, b3.value) + cross(b2.value, b3.first);
  const Vec3 b1_ddot = cross(b2.second, b3.value) + 2.0 * cross(b2.first, b3.first) +
                       cross(b2.value, b3.second);

  DesiredKinematics out{};
  out.rotation = rotationFromColumns(b1, b2.value, b3.value);
  out.rotation_dot = rotationFromColumns(b1_dot, b2.first, b3.first);
  out.rotation_ddot = rotationFromColumns(b1_ddot, b2.second, b3.second);
  out.body_rate = vee(skew(out.rotation.transpose() * out.rotation_dot));
  out.body_angular_accel = vee(skew(out.rotation.transpose() * out.rotation_ddot));
  return out;
}

bool strictlyPositive(const Vec3 &value) {
  return value.finite() && value.x > 0.0 && value.y > 0.0 && value.z > 0.0;
}

bool validInertiaMatrix(const Mat3 &inertia) {
  if (!inertia.finite()) {
    return false;
  }

  constexpr double kSymmetryTolerance = 1e-12;
  if (std::abs(inertia(0, 1) - inertia(1, 0)) > kSymmetryTolerance ||
      std::abs(inertia(0, 2) - inertia(2, 0)) > kSymmetryTolerance ||
      std::abs(inertia(1, 2) - inertia(2, 1)) > kSymmetryTolerance) {
    return false;
  }

  const double leading_minor_1 = inertia(0, 0);
  const double leading_minor_2 =
      inertia(0, 0) * inertia(1, 1) - inertia(0, 1) * inertia(1, 0);
  const double leading_minor_3 = determinant(inertia);
  return leading_minor_1 > kEps && leading_minor_2 > kEps && leading_minor_3 > kEps;
}

}  // namespace

LeeController::LeeController(LeeConfig config) : config_(config) {
  if (!(config_.mass_kg > 0.0) || !std::isfinite(config_.mass_kg) ||
      !(config_.gravity_mps2 > 0.0) || !std::isfinite(config_.gravity_mps2) ||
      !strictlyPositive(config_.k_position) || !strictlyPositive(config_.k_velocity) ||
      !strictlyPositive(config_.k_attitude) || !strictlyPositive(config_.k_rate) ||
      !validInertiaMatrix(config_.inertia_kgm2)) {
    throw std::invalid_argument("invalid Lee controller configuration");
  }
}

LeeOutput LeeController::update(const CanonicalState &state,
                                const TrajectoryReference &reference) const {
  if (!reference.finite()) throw std::invalid_argument("invalid trajectory reference");
  const Vec3 x = state.external_position_ned_m.value;
  const Vec3 v = state.ekf_velocity_ned_mps.value;
  const Quat q = state.attitude_ned_frd.value.normalized();
  const Mat3 r = q.toMat3();
  const Vec3 omega = state.body_rate_frd_radps.value;
  if (!x.finite() || !v.finite() || !omega.finite()) {
    throw std::invalid_argument("invalid Lee controller state");
  }

  LeeOutput out{};
  out.position_error_ned_m = x - reference.position_ned_m;
  out.velocity_error_ned_mps = v - reference.velocity_ned_mps;
  const Vec3 e3{0.0, 0.0, 1.0};

  // Lee et al. Eq. (12) bracketed vector A, expressed directly in NED.
  out.force_ned_n = -hadamard(config_.k_position, out.position_error_ned_m) -
                    hadamard(config_.k_velocity, out.velocity_error_ned_mps) -
                    config_.mass_kg * config_.gravity_mps2 * e3 +
                    config_.mass_kg * reference.acceleration_ned_mps2;

  const Vec3 body_z_ned = r * e3;
  out.collective_thrust_n = -dot(out.force_ned_n, body_z_ned);  // Lee Eq. (12).

  // Analytic derivative of A. The measured velocity is not differentiated: acceleration and jerk
  // follow the nominal rigid-body translational dynamics used by the Lee model.
  const Vec3 acceleration_ned = config_.gravity_mps2 * e3 -
                                (out.collective_thrust_n / config_.mass_kg) * body_z_ned;
  const Vec3 velocity_error_dot = acceleration_ned - reference.acceleration_ned_mps2;
  const Vec3 body_z_dot_ned = r * cross(omega, e3);
  const Vec3 force_dot = -hadamard(config_.k_position, out.velocity_error_ned_mps) -
                         hadamard(config_.k_velocity, velocity_error_dot) +
                         config_.mass_kg * reference.jerk_ned_mps3;
  const double thrust_dot = -(dot(force_dot, body_z_ned) +
                              dot(out.force_ned_n, body_z_dot_ned));
  const Vec3 jerk_ned = -(thrust_dot / config_.mass_kg) * body_z_ned -
                         (out.collective_thrust_n / config_.mass_kg) * body_z_dot_ned;
  const Vec3 velocity_error_ddot = jerk_ned - reference.jerk_ned_mps3;
  const Vec3 force_ddot = -hadamard(config_.k_position, velocity_error_dot) -
                          hadamard(config_.k_velocity, velocity_error_ddot) +
                          config_.mass_kg * reference.snap_ned_mps4;

  const auto desired = makeDesiredKinematics(out.force_ned_n, force_dot, force_ddot,
                                             reference.yaw_rad, reference.yaw_rate_radps,
                                             reference.yaw_accel_radps2);
  out.desired_rotation_ned_frd = desired.rotation;
  out.desired_rotation_dot = desired.rotation_dot;
  out.desired_rotation_ddot = desired.rotation_ddot;
  out.desired_body_z_ned = desired.rotation.column(2);
  out.desired_body_rate_frd_radps = desired.body_rate;
  out.desired_body_angular_accel_frd_radps2 = desired.body_angular_accel;

  const Mat3 rdtr = desired.rotation.transpose() * r;
  const Mat3 rtrd = r.transpose() * desired.rotation;
  out.attitude_error = 0.5 * vee(rdtr - rtrd);  // Lee Eq. (10).
  const Vec3 desired_rate_in_current_body = r.transpose() * desired.rotation * desired.body_rate;
  out.rate_error_frd_radps = omega - desired_rate_in_current_body;  // Lee Eq. (11).

  const Vec3 j_omega = config_.inertia_kgm2 * omega;
  const Vec3 feedforward_inside = cross(omega, desired_rate_in_current_body) -
                                  r.transpose() * desired.rotation * desired.body_angular_accel;
  // Lee et al. Eq. (13), in FRD body coordinates.
  out.body_moment_frd_nm = -hadamard(config_.k_attitude, out.attitude_error) -
                           hadamard(config_.k_rate, out.rate_error_frd_radps) +
                           cross(omega, j_omega) - config_.inertia_kgm2 * feedforward_inside;
  return out;
}

}  // namespace control
