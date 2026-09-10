#include "control/geometric_rate_controller.hpp"

#include <stdexcept>

namespace control {

GeometricRateController::GeometricRateController(GeometricRateConfig config) : config_(config) {
  if (!config_.k_attitude.finite() || !config_.rate_limit_radps.finite() ||
      config_.k_attitude.x < 0.0 || config_.k_attitude.y < 0.0 ||
      config_.k_attitude.z < 0.0 || config_.rate_limit_radps.x <= 0.0 ||
      config_.rate_limit_radps.y <= 0.0 || config_.rate_limit_radps.z <= 0.0) {
    throw std::invalid_argument("invalid geometric-rate configuration");
  }
}

GeometricRateOutput GeometricRateController::update(
    const Mat3 &current_rotation_ned_frd, const Mat3 &desired_rotation_ned_frd,
    const Vec3 &desired_body_rate_frd_radps) const {
  if (!current_rotation_ned_frd.finite() || !desired_rotation_ned_frd.finite() ||
      !desired_body_rate_frd_radps.finite()) {
    throw std::invalid_argument("invalid geometric-rate input");
  }

  GeometricRateOutput output{};
  output.attitude_error =
      0.5 * vee(desired_rotation_ned_frd.transpose() * current_rotation_ned_frd -
                current_rotation_ned_frd.transpose() * desired_rotation_ned_frd);

  const Vec3 feedforward_body_rate = current_rotation_ned_frd.transpose() *
                                     desired_rotation_ned_frd *
                                     desired_body_rate_frd_radps;
  output.body_rate_setpoint_frd_radps =
      clamped(feedforward_body_rate - hadamard(config_.k_attitude, output.attitude_error),
              config_.rate_limit_radps);
  return output;
}

}  // namespace control
