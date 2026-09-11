#include "px4_offboard_controllers/controllers/geometric_rate.hpp"

#include <stdexcept>

namespace px4_offboard {

GeometricRateController::GeometricRateController(GeometricRateConfig config) : config_(config) {
  const bool valid_gain = config.k_attitude.finite() && config.k_attitude.x >= 0.0 &&
                          config.k_attitude.y >= 0.0 && config.k_attitude.z >= 0.0;
  const bool valid_limit = config.rate_limit_radps.finite() &&
                           config.rate_limit_radps.x > 0.0 &&
                           config.rate_limit_radps.y > 0.0 &&
                           config.rate_limit_radps.z > 0.0;
  if (!valid_gain || !valid_limit) {
    throw std::invalid_argument("invalid geometric-rate configuration");
  }
}

GeometricRateOutput GeometricRateController::update(
    const Mat3 &current_rotation_ned_frd,
    const Mat3 &desired_rotation_ned_frd,
    const Vec3 &desired_body_rate_frd_radps) const {
  if (!current_rotation_ned_frd.finite() || !desired_rotation_ned_frd.finite() ||
      !desired_body_rate_frd_radps.finite()) {
    throw std::invalid_argument("invalid geometric-rate input");
  }

  GeometricRateOutput output{};
  // e_R = 1/2 vee(R_d^T R - R^T R_d), preserving the audited geometric-rate handoff.
  output.attitude_error =
      0.5 * vee(desired_rotation_ned_frd.transpose() * current_rotation_ned_frd -
                current_rotation_ned_frd.transpose() * desired_rotation_ned_frd);
  const Vec3 feed_forward = current_rotation_ned_frd.transpose() * desired_rotation_ned_frd *
                            desired_body_rate_frd_radps;
  output.body_rate_setpoint_frd_radps =
      clamped(feed_forward - hadamard(config_.k_attitude, output.attitude_error),
              config_.rate_limit_radps);
  return output;
}

}  // namespace px4_offboard
