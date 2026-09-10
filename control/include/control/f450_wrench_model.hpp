#pragma once

#include "control/math.hpp"

#include <array>
#include <string>

namespace control {

struct F450WrenchConfig {
  std::array<Vec3, 4> allocator_rotor_position_frd_m{};
  std::array<double, 4> allocator_thrust_coefficient{};
  std::array<double, 4> allocator_yaw_moment_ratio{};
  std::array<Vec3, 4> physical_rotor_position_frd_m{};
  std::array<double, 4> physical_yaw_moment_ratio{};
  double motor_thrust_constant_n_per_radps2{0.0};
  double esc_speed_min_radps{0.0};
  double esc_speed_max_radps{0.0};
  double collective_force_tolerance_n{0.0};
  double moment_tolerance_nm{0.0};
  double jacobian_condition_limit{0.0};
  int max_iterations{0};
};

struct F450ForwardState {
  bool ok{false};
  std::string reason;
  Vec3 normalized_torque_frd{};
  double normalized_collective_thrust{0.0};
  std::array<double, 4> motor_command{};
  std::array<double, 4> rotor_speed_radps{};
  std::array<double, 4> rotor_thrust_n{};
  double collective_thrust_n{0.0};
  Vec3 body_moment_frd_nm{};
};

enum class F450WrenchStatus {
  success,
  invalid_input,
  singular_jacobian,
  infeasible,
  no_convergence,
};

struct F450WrenchResult {
  bool ok{false};
  F450WrenchStatus status{F450WrenchStatus::invalid_input};
  std::string reason;
  Vec3 normalized_torque_frd{};
  Vec3 normalized_thrust_body_frd{};
  std::array<double, 4> motor_command{};
  std::array<double, 4> rotor_speed_radps{};
  std::array<double, 4> rotor_thrust_n{};
  double reconstructed_collective_thrust_n{0.0};
  Vec3 reconstructed_body_moment_frd_nm{};
  double collective_force_residual_n{0.0};
  int iterations{0};
};

F450WrenchConfig frozenF450WrenchConfig();

class F450WrenchModel {
 public:
  explicit F450WrenchModel(F450WrenchConfig config);

  F450ForwardState forward(const Vec3 &normalized_torque_frd,
                           double normalized_collective_thrust) const;

  F450WrenchResult normalize(double desired_collective_thrust_n,
                             const Vec3 &desired_body_moment_frd_nm,
                             double normalized_collective_thrust) const;

 private:
  F450WrenchConfig config_;
  std::array<std::array<double, 4>, 4> normalized_mixer_{};
};

}  // namespace control
