#include "px4_offboard_controllers/controllers/px4_thrust_normalization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace px4_offboard {
namespace {

Vec3 limitTilt(Vec3 body_z_ned, double max_tilt_rad, bool &limited) {
  body_z_ned = normalized(body_z_ned);
  const Vec3 down_ned{0.0, 0.0, 1.0};
  const double cosine = std::clamp(dot(body_z_ned, down_ned), -1.0, 1.0);
  const double angle = std::acos(cosine);
  const double limited_angle = std::min(angle, max_tilt_rad);

  Vec3 horizontal = body_z_ned - cosine * down_ned;
  if (horizontal.squaredNorm() <
      static_cast<double>(std::numeric_limits<float>::epsilon())) {
    horizontal = {1.0, 0.0, 0.0};
  }
  if (angle > max_tilt_rad + 1e-12) {
    limited = true;
  }

  return std::cos(limited_angle) * down_ned +
         std::sin(limited_angle) * normalized(horizontal);
}

}  // namespace

Px4ThrustNormalization::Px4ThrustNormalization(Px4ThrustConfig config)
    : config_(config), effective_min_thrust_(std::max(config.min_thrust, 0.001)) {
  const bool valid = config.hover_thrust > 0.0 && std::isfinite(config.hover_thrust) &&
                     config.gravity_mps2 == kPx4OneGmps2 && config.tilt_limit_rad >= 0.0 &&
                     config.tilt_limit_rad < 0.5 * kPi && config.min_thrust >= 0.0 &&
                     config.max_thrust > effective_min_thrust_ && config.max_thrust <= 1.0 &&
                     config.horizontal_thrust_margin >= 0.0 &&
                     config.horizontal_thrust_margin <= config.max_thrust;
  if (!valid) {
    throw std::invalid_argument("invalid PX4 thrust-normalization configuration");
  }
}

Px4ThrustOutput Px4ThrustNormalization::fromAccelerationSetpoint(
    const Vec3 &acceleration_ned_mps2) const {
  if (!acceleration_ned_mps2.finite()) {
    throw std::invalid_argument("acceleration setpoint is not finite");
  }

  Px4ThrustOutput output{};
  output.acceleration_setpoint_ned_mps2 = acceleration_ned_mps2;

  // Mirror PX4 PositionControl::_accelerationControl: construct body-Z first, then apply the
  // configured tilt limit before scaling the collective thrust by body_z.z.
  double z_specific_force = -config_.gravity_mps2;
  if (!config_.decouple_horizontal_and_vertical_acceleration) {
    z_specific_force += acceleration_ned_mps2.z;
  }
  Vec3 body_z_ned = normalized(
      {-acceleration_ned_mps2.x, -acceleration_ned_mps2.y, -z_specific_force});
  body_z_ned = limitTilt(body_z_ned, config_.tilt_limit_rad, output.saturated);

  const double thrust_z = acceleration_ned_mps2.z *
                              (config_.hover_thrust / config_.gravity_mps2) -
                          config_.hover_thrust;
  const double raw_collective = thrust_z / body_z_ned.z;
  const double collective = std::min(raw_collective, -effective_min_thrust_);
  if (std::abs(collective - raw_collective) > 1e-12) {
    output.saturated = true;
  }

  Vec3 thrust_ned = body_z_ned * collective;
  const double horizontal_norm = std::hypot(thrust_ned.x, thrust_ned.y);
  const double max_thrust_squared = config_.max_thrust * config_.max_thrust;
  const double allocated_horizontal =
      std::min(horizontal_norm, config_.horizontal_thrust_margin);
  const double max_vertical_squared =
      std::max(0.0, max_thrust_squared - allocated_horizontal * allocated_horizontal);

  const double unclamped_z = thrust_ned.z;
  thrust_ned.z = std::max(thrust_ned.z, -std::sqrt(max_vertical_squared));
  if (std::abs(thrust_ned.z - unclamped_z) > 1e-12) {
    output.saturated = true;
  }

  const double max_horizontal =
      std::sqrt(std::max(0.0, max_thrust_squared - thrust_ned.z * thrust_ned.z));
  if (horizontal_norm > max_horizontal && horizontal_norm > kEps) {
    const double scale = max_horizontal / horizontal_norm;
    thrust_ned.x *= scale;
    thrust_ned.y *= scale;
    output.saturated = true;
  }

  output.normalized_thrust_ned = thrust_ned;
  output.desired_body_z_ned = normalized(-thrust_ned);
  output.normalized_collective_thrust_magnitude = thrust_ned.norm();
  output.normalized_thrust_body_frd =
      {0.0, 0.0, -output.normalized_collective_thrust_magnitude};
  return output;
}

Px4ThrustOutput Px4ThrustNormalization::fromPhysicalRotorForce(
    const Vec3 &rotor_force_ned_n, double mass_kg) const {
  if (!rotor_force_ned_n.finite() || !(mass_kg > 0.0) || !std::isfinite(mass_kg)) {
    throw std::invalid_argument("invalid physical rotor force or mass");
  }

  return fromAccelerationSetpoint(rotor_force_ned_n / mass_kg +
                                  Vec3{0.0, 0.0, config_.gravity_mps2});
}

}  // namespace px4_offboard
