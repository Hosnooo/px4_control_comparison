#pragma once

#include "control/math.hpp"

namespace control {

struct Px4ThrustConfig {
  double hover_thrust{0.0};
  double gravity_mps2{0.0};
  double tilt_limit_rad{0.0};
  double min_thrust{0.0};
  double max_thrust{0.0};
  double horizontal_thrust_margin{0.0};
  bool decouple_horizontal_and_vertical_acceleration{false};
};

struct Px4ThrustOutput {
  Vec3 acceleration_setpoint_ned_mps2{};
  Vec3 desired_body_z_ned{};
  Vec3 normalized_thrust_ned{};
  Vec3 normalized_thrust_body_frd{};
  double normalized_collective_thrust_magnitude{0.0};
  bool saturated{false};
};

class Px4ThrustNormalization {
 public:
  explicit Px4ThrustNormalization(Px4ThrustConfig config);
  Px4ThrustOutput fromAccelerationSetpoint(const Vec3 &acceleration_ned_mps2) const;
  Px4ThrustOutput fromPhysicalRotorForce(const Vec3 &rotor_force_ned_n, double mass_kg) const;
  const Px4ThrustConfig &config() const { return config_; }

 private:
  Px4ThrustConfig config_;
  double effective_min_thrust_{0.0};
};

}  // namespace control
