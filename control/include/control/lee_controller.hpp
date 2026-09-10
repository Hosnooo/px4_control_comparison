#pragma once

#include "control/state.hpp"

namespace control {

struct LeeConfig {
  double mass_kg{0.0};
  double gravity_mps2{0.0};
  Vec3 k_position{};
  Vec3 k_velocity{};
  Vec3 k_attitude{};
  Vec3 k_rate{};
  Mat3 inertia_kgm2{Mat3::zero()};
};

struct LeeOutput {
  Vec3 position_error_ned_m{};
  Vec3 velocity_error_ned_mps{};
  Vec3 force_ned_n{};
  Vec3 desired_body_z_ned{};
  Mat3 desired_rotation_ned_frd{};
  Mat3 desired_rotation_dot{};
  Mat3 desired_rotation_ddot{};
  Vec3 desired_body_rate_frd_radps{};
  Vec3 desired_body_angular_accel_frd_radps2{};
  Vec3 attitude_error{};
  Vec3 rate_error_frd_radps{};
  double collective_thrust_n{0.0};
  Vec3 body_moment_frd_nm{};
};

class LeeController {
 public:
  explicit LeeController(LeeConfig config);
  LeeOutput update(const CanonicalState &state, const TrajectoryReference &reference) const;
  const LeeConfig &config() const { return config_; }

 private:
  LeeConfig config_;
};

}  // namespace control
