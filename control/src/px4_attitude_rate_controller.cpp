#include "control/px4_attitude_rate_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control {
namespace {

// MulticopterRateControl constrains gyro-sample dt to this interval before RateControl.
constexpr double kRateDtMinS = 0.000125;
constexpr double kRateDtMaxS = 0.02;

// RateControl reduces integral gain as rate error approaches 400 deg/s.
constexpr double kIntegratorReductionRateRadps = 400.0 * kPi / 180.0;

double clampUnit(double value) {
  return std::clamp(value, -1.0, 1.0);
}

bool nonNegative(const Vec3 &value) {
  return value.finite() && value.x >= 0.0 && value.y >= 0.0 && value.z >= 0.0;
}

Quat px4QuaternionFromTwoVectors(const Vec3 &from, const Vec3 &to) {
  if (from.squaredNorm() <= kEps || to.squaredNorm() <= kEps || !from.finite() ||
      !to.finite()) {
    throw std::invalid_argument("cannot construct rotation from zero/invalid vector");
  }

  Vec3 cross_product = cross(from, to);
  const double dot_product = dot(from, to);
  constexpr double kParallelTolerance = 1e-5;

  if (cross_product.norm() < kParallelTolerance && dot_product < 0.0) {
    // Exact 180-degree ambiguity handling from PX4 matrix::Quaternion(src, dst).
    const Vec3 abs_from{std::abs(from.x), std::abs(from.y), std::abs(from.z)};
    Vec3 basis{};

    if (abs_from.x < abs_from.y) {
      basis = abs_from.x < abs_from.z ? Vec3{1.0, 0.0, 0.0}
                                      : Vec3{0.0, 0.0, 1.0};
    } else {
      basis = abs_from.y < abs_from.z ? Vec3{0.0, 1.0, 0.0}
                                      : Vec3{0.0, 0.0, 1.0};
    }

    cross_product = cross(from, basis);
    return Quat{0.0, cross_product.x, cross_product.y, cross_product.z}.normalized();
  }

  const double real = dot_product + std::sqrt(from.squaredNorm() * to.squaredNorm());
  return Quat{real, cross_product.x, cross_product.y, cross_product.z}.normalized();
}

}  // namespace

Px4AttitudeRateController::Px4AttitudeRateController(Px4AttitudeConfig attitude,
                                                     Px4RateConfig rate)
    : attitude_(attitude),
      rate_(rate),
      attitude_gain_(attitude.proportional_gain),
      effective_rate_p_(hadamard(rate.k, rate.p)),
      effective_rate_i_(hadamard(rate.k, rate.i)),
      effective_rate_d_(hadamard(rate.k, rate.d)) {
  const bool valid_attitude =
      nonNegative(attitude_.proportional_gain) && attitude_.rate_limit_radps.finite() &&
      std::isfinite(attitude_.yaw_weight) && attitude_.rate_limit_radps.x > 0.0 &&
      attitude_.rate_limit_radps.y > 0.0 && attitude_.rate_limit_radps.z > 0.0;
  const bool valid_rate =
      nonNegative(rate_.k) && nonNegative(rate_.p) && nonNegative(rate_.i) &&
      nonNegative(rate_.d) && nonNegative(rate_.ff) && nonNegative(rate_.integrator_limit) &&
      std::isfinite(rate_.yaw_torque_cutoff_hz) && rate_.yaw_torque_cutoff_hz >= 0.0;

  if (!valid_attitude || !valid_rate) {
    throw std::invalid_argument("invalid PX4 attitude/rate mirror configuration");
  }

  attitude_.yaw_weight = std::clamp(attitude_.yaw_weight, 0.0, 1.0);
  if (attitude_.yaw_weight > 1e-4) {
    attitude_gain_.z /= attitude_.yaw_weight;
  }
}

Vec3 Px4AttitudeRateController::attitudeUpdate(const Quat &current_ned_frd,
                                               const Quat &desired_ned_frd,
                                               double yaw_speed_setpoint_radps) const {
  const Quat current = current_ned_frd.normalized();
  Quat desired = desired_ned_frd.normalized();

  const Vec3 current_body_z_ned = current.dcmZ();
  const Vec3 desired_body_z_ned = desired.dcmZ();
  Quat reduced_desired =
      px4QuaternionFromTwoVectors(current_body_z_ned, desired_body_z_ned);

  if (std::abs(reduced_desired.x) > 1.0 - 1e-5 ||
      std::abs(reduced_desired.y) > 1.0 - 1e-5) {
    reduced_desired = desired;
  } else {
    reduced_desired = (reduced_desired * current).normalized();
  }

  Quat delta_yaw = (reduced_desired.inverse() * desired).canonical();
  delta_yaw.w = clampUnit(delta_yaw.w);
  delta_yaw.z = clampUnit(delta_yaw.z);

  const double yaw_weight = attitude_.yaw_weight;
  const Quat weighted_yaw{std::cos(yaw_weight * std::acos(delta_yaw.w)),
                          0.0,
                          0.0,
                          std::sin(yaw_weight * std::asin(delta_yaw.z))};
  desired = (reduced_desired * weighted_yaw).normalized();

  const Quat attitude_error_quaternion = (current.inverse() * desired).canonical();
  const Vec3 attitude_error{2.0 * attitude_error_quaternion.x,
                            2.0 * attitude_error_quaternion.y,
                            2.0 * attitude_error_quaternion.z};
  Vec3 rate_setpoint = hadamard(attitude_error, attitude_gain_);

  if (std::isfinite(yaw_speed_setpoint_radps)) {
    // PX4 expresses world-Z yaw feed-forward in the current FRD body frame.
    rate_setpoint += current.inverse().dcmZ() * yaw_speed_setpoint_radps;
  }

  return clamped(rate_setpoint, attitude_.rate_limit_radps);
}

Px4RateOutput Px4AttitudeRateController::rateUpdate(const RateControlInput &input) {
  if (!input.body_rate_frd_radps.finite() ||
      !input.body_rate_setpoint_frd_radps.finite() ||
      !input.body_angular_accel_frd_radps2.finite() ||
      !input.normalized_thrust_body_frd.finite() || !std::isfinite(input.dt_s) ||
      input.dt_s < 0.0) {
    throw std::invalid_argument("invalid PX4 rate mirror input");
  }

  last_dt_s_ = std::clamp(input.dt_s, kRateDtMinS, kRateDtMaxS);
  if (!input.armed) {
    rate_integral_ = {};
  }

  Vec3 rate_error = input.body_rate_setpoint_frd_radps - input.body_rate_frd_radps;

  // PX4 multiplies MC_*RATE_{P,I,D} by MC_*RATE_K before entering RateControl.
  Vec3 normalized_torque =
      hadamard(effective_rate_p_, rate_error) + rate_integral_ -
      hadamard(effective_rate_d_, input.body_angular_accel_frd_radps2) +
      hadamard(rate_.ff, input.body_rate_setpoint_frd_radps);

  if (!input.landed_or_maybe_landed) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      double error_for_integrator = rate_error[axis];
      if (input.saturation_positive[axis]) {
        error_for_integrator = std::min(error_for_integrator, 0.0);
      }
      if (input.saturation_negative[axis]) {
        error_for_integrator = std::max(error_for_integrator, 0.0);
      }

      double integral_factor = error_for_integrator / kIntegratorReductionRateRadps;
      integral_factor = std::max(0.0, 1.0 - integral_factor * integral_factor);
      const double candidate =
          rate_integral_[axis] + integral_factor * effective_rate_i_[axis] *
                                     error_for_integrator * last_dt_s_;

      if (std::isfinite(candidate)) {
        rate_integral_[axis] = std::clamp(candidate, -rate_.integrator_limit[axis],
                                          rate_.integrator_limit[axis]);
      }
    }
  }

  // PX4 AlphaFilter: tau = 1/(2*pi*f), alpha = dt/(tau+dt); f=0 is pass-through.
  const double yaw_filter_time_constant_s =
      rate_.yaw_torque_cutoff_hz > kEps
          ? 1.0 / (2.0 * kPi * rate_.yaw_torque_cutoff_hz)
          : 0.0;
  const double yaw_filter_alpha =
      last_dt_s_ / (yaw_filter_time_constant_s + last_dt_s_);
  yaw_filter_state_ += yaw_filter_alpha * (normalized_torque.z - yaw_filter_state_);
  normalized_torque.z = yaw_filter_state_;

  Vec3 normalized_thrust_body_frd = input.normalized_thrust_body_frd;
  if (rate_.battery_scaling_enabled && std::isfinite(input.battery_scale) &&
      input.battery_scale > 0.0) {
    normalized_torque = clamped(normalized_torque * input.battery_scale, {1.0, 1.0, 1.0});
    normalized_thrust_body_frd =
        clamped(normalized_thrust_body_frd * input.battery_scale, {1.0, 1.0, 1.0});
  }

  return {normalized_torque, normalized_thrust_body_frd, rate_integral_};
}

void Px4AttitudeRateController::reset() {
  rate_integral_ = {};
  yaw_filter_state_ = 0.0;
  last_dt_s_ = 0.0;
}

}  // namespace control
