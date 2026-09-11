#include "px4_offboard_controllers/controllers/px4_attitude_rate_mirror.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard {
namespace {

constexpr double kRateDtMinS = 0.000125;
constexpr double kRateDtMaxS = 0.02;
constexpr double kIntegratorReductionRateRadps = 400.0 * kPi / 180.0;

double clampUnit(double value) { return std::clamp(value, -1.0, 1.0); }

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
  if (cross_product.norm() < 1e-5 && dot_product < 0.0) {
    const Vec3 absolute{std::abs(from.x), std::abs(from.y), std::abs(from.z)};
    Vec3 basis{};
    if (absolute.x < absolute.y) {
      basis = absolute.x < absolute.z ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 0.0, 1.0};
    } else {
      basis = absolute.y < absolute.z ? Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0};
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
  const bool valid_attitude = nonNegative(attitude.proportional_gain) &&
                              attitude.rate_limit_radps.finite() &&
                              std::isfinite(attitude.yaw_weight) &&
                              attitude.rate_limit_radps.x > 0.0 &&
                              attitude.rate_limit_radps.y > 0.0 &&
                              attitude.rate_limit_radps.z > 0.0;
  const bool valid_rate = nonNegative(rate.k) && nonNegative(rate.p) && nonNegative(rate.i) &&
                          nonNegative(rate.d) && nonNegative(rate.ff) &&
                          nonNegative(rate.integrator_limit) &&
                          std::isfinite(rate.yaw_torque_cutoff_hz) &&
                          rate.yaw_torque_cutoff_hz >= 0.0;
  if (!valid_attitude || !valid_rate) {
    throw std::invalid_argument("invalid PX4 attitude/rate mirror configuration");
  }

  attitude_.yaw_weight = std::clamp(attitude_.yaw_weight, 0.0, 1.0);
  if (attitude_.yaw_weight > 1e-4) {
    // PX4 compensates the yaw gain so yaw_weight changes authority without changing the tuned
    // small-angle yaw gain.
    attitude_gain_.z /= attitude_.yaw_weight;
  }
}

Vec3 Px4AttitudeRateController::attitudeUpdate(const Quat &current_ned_frd,
                                               const Quat &desired_ned_frd,
                                               double yaw_speed_setpoint_radps) const {
  const Quat current = current_ned_frd.normalized();
  Quat desired = desired_ned_frd.normalized();

  // Preserve PX4's reduced-attitude construction: align body-Z first, then mix in yaw.
  Quat reduced = px4QuaternionFromTwoVectors(current.dcmZ(), desired.dcmZ());
  if (std::abs(reduced.x) > 1.0 - 1e-5 || std::abs(reduced.y) > 1.0 - 1e-5) {
    reduced = desired;
  } else {
    reduced = (reduced * current).normalized();
  }

  Quat delta_yaw = (reduced.inverse() * desired).canonical();
  delta_yaw.w = clampUnit(delta_yaw.w);
  delta_yaw.z = clampUnit(delta_yaw.z);
  const double yaw_weight = attitude_.yaw_weight;
  const Quat weighted_yaw{std::cos(yaw_weight * std::acos(delta_yaw.w)), 0.0, 0.0,
                          std::sin(yaw_weight * std::asin(delta_yaw.z))};
  desired = (reduced * weighted_yaw).normalized();

  const Quat error_quaternion = (current.inverse() * desired).canonical();
  const Vec3 attitude_error{2.0 * error_quaternion.x, 2.0 * error_quaternion.y,
                            2.0 * error_quaternion.z};
  Vec3 rate_setpoint = hadamard(attitude_error, attitude_gain_);
  if (std::isfinite(yaw_speed_setpoint_radps)) {
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

  const Vec3 rate_error = input.body_rate_setpoint_frd_radps - input.body_rate_frd_radps;
  Vec3 torque = hadamard(effective_rate_p_, rate_error) + rate_integral_ -
                hadamard(effective_rate_d_, input.body_angular_accel_frd_radps2) +
                hadamard(rate_.ff, input.body_rate_setpoint_frd_radps);

  if (!input.landed_or_maybe_landed) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      double error = rate_error[axis];
      if (input.saturation_positive[axis]) {
        error = std::min(error, 0.0);
      }
      if (input.saturation_negative[axis]) {
        error = std::max(error, 0.0);
      }

      // This is PX4's error-dependent integrator reduction, not a generic anti-windup rule.
      const double normalized_error = error / kIntegratorReductionRateRadps;
      const double reduction = std::max(0.0, 1.0 - normalized_error * normalized_error);
      const double candidate = rate_integral_[axis] +
                               reduction * effective_rate_i_[axis] * error * last_dt_s_;
      if (std::isfinite(candidate)) {
        rate_integral_[axis] = std::clamp(candidate, -rate_.integrator_limit[axis],
                                          rate_.integrator_limit[axis]);
      }
    }
  }

  const double time_constant = rate_.yaw_torque_cutoff_hz > kEps
                                   ? 1.0 / (2.0 * kPi * rate_.yaw_torque_cutoff_hz)
                                   : 0.0;
  const double alpha = last_dt_s_ / (time_constant + last_dt_s_);
  yaw_filter_state_ += alpha * (torque.z - yaw_filter_state_);
  torque.z = yaw_filter_state_;

  Vec3 thrust = input.normalized_thrust_body_frd;
  if (rate_.battery_scaling_enabled && std::isfinite(input.battery_scale) &&
      input.battery_scale > 0.0) {
    torque = clamped(torque * input.battery_scale, {1.0, 1.0, 1.0});
    thrust = clamped(thrust * input.battery_scale, {1.0, 1.0, 1.0});
  }

  return {torque, thrust, rate_integral_};
}

void Px4AttitudeRateController::reset() {
  rate_integral_ = {};
  yaw_filter_state_ = 0.0;
  last_dt_s_ = 0.0;
}

}  // namespace px4_offboard
