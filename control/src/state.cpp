#include "control/state.hpp"

#include <cmath>

namespace control {
namespace {
constexpr double kAgeToleranceS = 1e-12;

bool ageValid(double now_s, double timestamp_s, double max_age_s, double &age_s) {
  if (!std::isfinite(now_s) || !std::isfinite(timestamp_s) || !std::isfinite(max_age_s) ||
      max_age_s < 0.0) {
    return false;
  }
  age_s = now_s - timestamp_s;
  return age_s >= -kAgeToleranceS && age_s <= max_age_s + kAgeToleranceS;
}
}  // namespace

bool TrajectoryReference::finite() const {
  return position_ned_m.finite() && velocity_ned_mps.finite() && acceleration_ned_mps2.finite() &&
         jerk_ned_mps3.finite() && snap_ned_mps4.finite() && std::isfinite(yaw_rad) &&
         std::isfinite(yaw_rate_radps) && std::isfinite(yaw_accel_radps2) &&
         std::isfinite(timestamp_s);
}

StateValidity validateState(const CanonicalState &state, double now_s,
                            const StateValidityLimits &limits,
                            bool require_angular_acceleration) {
  StateValidity out{};
  const bool limits_valid =
      std::isfinite(limits.max_external_position_age_s) &&
      std::isfinite(limits.max_ekf_velocity_age_s) &&
      std::isfinite(limits.max_attitude_age_s) &&
      std::isfinite(limits.max_body_rate_age_s) &&
      std::isfinite(limits.max_angular_accel_age_s) &&
      std::isfinite(limits.quaternion_norm_tolerance) &&
      limits.max_external_position_age_s >= 0.0 &&
      limits.max_ekf_velocity_age_s >= 0.0 && limits.max_attitude_age_s >= 0.0 &&
      limits.max_body_rate_age_s >= 0.0 && limits.max_angular_accel_age_s >= 0.0 &&
      limits.quaternion_norm_tolerance >= 0.0;
  if (!limits_valid) {
    out.reason = "invalid state-validity limits";
    return out;
  }
  if (!state.external_position_ned_m.value.finite()) {
    out.reason = "external position is not finite";
    return out;
  }
  if (!ageValid(now_s, state.external_position_ned_m.timestamp_s,
                limits.max_external_position_age_s, out.external_position_age_s)) {
    out.reason = "external position is stale";
    return out;
  }
  if (!state.ekf_velocity_ned_mps.value.finite()) {
    out.reason = "PX4 EKF velocity is not finite";
    return out;
  }
  if (!ageValid(now_s, state.ekf_velocity_ned_mps.timestamp_s, limits.max_ekf_velocity_age_s,
                out.ekf_velocity_age_s)) {
    out.reason = "PX4 EKF velocity is stale";
    return out;
  }
  if (!state.attitude_ned_frd.value.finite()) {
    out.reason = "PX4 attitude is not finite";
    return out;
  }
  const double q_norm = std::sqrt(state.attitude_ned_frd.value.squaredNorm());
  if (!std::isfinite(q_norm) || std::abs(q_norm - 1.0) > limits.quaternion_norm_tolerance) {
    out.reason = "PX4 attitude quaternion is not normalized";
    return out;
  }
  if (!ageValid(now_s, state.attitude_ned_frd.timestamp_s, limits.max_attitude_age_s,
                out.attitude_age_s)) {
    out.reason = "PX4 attitude is stale";
    return out;
  }
  if (!state.body_rate_frd_radps.value.finite()) {
    out.reason = "PX4 body rate is not finite";
    return out;
  }
  if (!ageValid(now_s, state.body_rate_frd_radps.timestamp_s, limits.max_body_rate_age_s,
                out.body_rate_age_s)) {
    out.reason = "PX4 body rate is stale";
    return out;
  }
  if (require_angular_acceleration) {
    if (!state.body_angular_accel_frd_radps2.value.finite()) {
      out.reason = "PX4 angular acceleration is required but invalid";
      return out;
    }
    if (!ageValid(now_s, state.body_angular_accel_frd_radps2.timestamp_s,
                  limits.max_angular_accel_age_s, out.angular_accel_age_s)) {
      out.reason = "PX4 angular acceleration is stale";
      return out;
    }
  }
  out.ok = true;
  return out;
}

}  // namespace control
