#include "control/px4_thrust_normalization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace control {
namespace {

Vec3 limitTilt(Vec3 body_z_ned, double max_angle_rad, bool &limited) {
  body_z_ned = normalized(body_z_ned);
  const Vec3 world_down_ned{0.0, 0.0, 1.0};
  const double cosine_tilt = std::clamp(dot(body_z_ned, world_down_ned), -1.0, 1.0);
  const double original_angle = std::acos(cosine_tilt);
  const double limited_angle = std::min(original_angle, max_angle_rad);

  Vec3 horizontal_rejection = body_z_ned - cosine_tilt * world_down_ned;
  // ControlMath::limitTilt() uses FLT_EPSILON because PX4 performs this in float.
  if (horizontal_rejection.squaredNorm() <
      static_cast<double>(std::numeric_limits<float>::epsilon())) {
    horizontal_rejection = {1.0, 0.0, 0.0};
  }

  if (original_angle > max_angle_rad + 1e-12) {
    limited = true;
  }

  return std::cos(limited_angle) * world_down_ned +
         std::sin(limited_angle) * normalized(horizontal_rejection);
}

}  // namespace

Px4ThrustNormalization::Px4ThrustNormalization(Px4ThrustConfig config)
    : config_(config), effective_min_thrust_(std::max(config.min_thrust, 0.001)) {
  const bool valid = config_.hover_thrust > 0.0 && std::isfinite(config_.hover_thrust) &&
                     config_.gravity_mps2 == kPx4OneGmps2 &&
                     config_.tilt_limit_rad >= 0.0 && config_.tilt_limit_rad < 0.5 * kPi &&
                     config_.min_thrust >= 0.0 &&
                     config_.max_thrust > effective_min_thrust_ &&
                     config_.max_thrust <= 1.0 &&
                     config_.horizontal_thrust_margin >= 0.0 &&
                     config_.horizontal_thrust_margin <= config_.max_thrust;
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

  // Mirrors PositionControl::_accelerationControl() at the frozen PX4 revision.
  double z_specific_force = -config_.gravity_mps2;
  if (!config_.decouple_horizontal_and_vertical_acceleration) {
    z_specific_force += acceleration_ned_mps2.z;
  }

  Vec3 body_z_ned = normalized({-acceleration_ned_mps2.x,
                                -acceleration_ned_mps2.y,
                                -z_specific_force});
  body_z_ned = limitTilt(body_z_ned, config_.tilt_limit_rad, output.saturated);

  const double thrust_ned_z =
      acceleration_ned_mps2.z * (config_.hover_thrust / config_.gravity_mps2) -
      config_.hover_thrust;
  const double raw_collective = thrust_ned_z / body_z_ned.z;
  // PositionControl::setThrustLimits() enforces at least 10e-4 normalized thrust.
  const double collective = std::min(raw_collective, -effective_min_thrust_);
  if (std::abs(collective - raw_collective) > 1e-12) {
    output.saturated = true;
  }

  Vec3 normalized_thrust_ned = body_z_ned * collective;

  // Mirrors the thrust-vector saturation in PositionControl::_velocityControl().
  const double horizontal_norm =
      std::hypot(normalized_thrust_ned.x, normalized_thrust_ned.y);
  const double max_thrust_squared = config_.max_thrust * config_.max_thrust;
  const double allocated_horizontal =
      std::min(horizontal_norm, config_.horizontal_thrust_margin);
  const double max_vertical_squared =
      std::max(0.0, max_thrust_squared - allocated_horizontal * allocated_horizontal);

  const double unsaturated_z = normalized_thrust_ned.z;
  normalized_thrust_ned.z =
      std::max(normalized_thrust_ned.z, -std::sqrt(max_vertical_squared));
  if (std::abs(normalized_thrust_ned.z - unsaturated_z) > 1e-12) {
    output.saturated = true;
  }

  const double max_horizontal_squared =
      std::max(0.0, max_thrust_squared - normalized_thrust_ned.z * normalized_thrust_ned.z);
  const double max_horizontal = std::sqrt(max_horizontal_squared);
  if (horizontal_norm > max_horizontal && horizontal_norm > kEps) {
    const double scale = max_horizontal / horizontal_norm;
    normalized_thrust_ned.x *= scale;
    normalized_thrust_ned.y *= scale;
    output.saturated = true;
  }

  output.normalized_thrust_ned = normalized_thrust_ned;
  output.desired_body_z_ned = normalized(-normalized_thrust_ned);
  output.normalized_collective_thrust_magnitude = normalized_thrust_ned.norm();

  // PX4 multicopter attitude/rate setpoints carry collective thrust on negative body-Z in FRD.
  output.normalized_thrust_body_frd =
      {0.0, 0.0, -output.normalized_collective_thrust_magnitude};
  return output;
}

Px4ThrustOutput Px4ThrustNormalization::fromPhysicalRotorForce(
    const Vec3 &rotor_force_ned_n, double mass_kg) const {
  if (!rotor_force_ned_n.finite() || !(mass_kg > 0.0) || !std::isfinite(mass_kg)) {
    throw std::invalid_argument("invalid physical rotor force or mass");
  }

  // NED rigid-body translation: m*a = m*g*e3 + F_rotor.
  const Vec3 acceleration_ned_mps2 =
      rotor_force_ned_n / mass_kg + Vec3{0.0, 0.0, config_.gravity_mps2};
  return fromAccelerationSetpoint(acceleration_ned_mps2);
}

}  // namespace control
