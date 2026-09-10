#include "control/f450_wrench_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace control {
namespace {

using Matrix4 = std::array<std::array<double, 4>, 4>;

Matrix4 invert(Matrix4 matrix) {
  Matrix4 inverse{};
  for (std::size_t row = 0; row < 4; ++row) {
    inverse[row][row] = 1.0;
  }

  for (std::size_t column = 0; column < 4; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1; row < 4; ++row) {
      if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][column]) <= kEps) {
      throw std::invalid_argument("singular F450 allocator effectiveness matrix");
    }
    std::swap(matrix[column], matrix[pivot]);
    std::swap(inverse[column], inverse[pivot]);

    const double divisor = matrix[column][column];
    for (std::size_t entry = 0; entry < 4; ++entry) {
      matrix[column][entry] /= divisor;
      inverse[column][entry] /= divisor;
    }

    for (std::size_t row = 0; row < 4; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = matrix[row][column];
      for (std::size_t entry = 0; entry < 4; ++entry) {
        matrix[row][entry] -= factor * matrix[column][entry];
        inverse[row][entry] -= factor * inverse[column][entry];
      }
    }
  }
  return inverse;
}

Matrix4 makeNormalizedMixer(const F450WrenchConfig &config) {
  Matrix4 effectiveness{};
  for (std::size_t motor = 0; motor < 4; ++motor) {
    const double thrust_coefficient = config.allocator_thrust_coefficient[motor];
    const Vec3 &position = config.allocator_rotor_position_frd_m[motor];
    effectiveness[0][motor] = -thrust_coefficient * position.y;
    effectiveness[1][motor] = thrust_coefficient * position.x;
    effectiveness[2][motor] =
        thrust_coefficient * config.allocator_yaw_moment_ratio[motor];
    effectiveness[3][motor] = -thrust_coefficient;
  }

  Matrix4 mixer = invert(effectiveness);

  auto rpyScale = [&mixer](std::size_t axis) {
    double squared_norm = 0.0;
    int nonzero = 0;
    for (const auto &motor : mixer) {
      squared_norm += motor[axis] * motor[axis];
      if (std::abs(motor[axis]) > 1e-3) {
        ++nonzero;
      }
    }
    return nonzero > 0 ? std::sqrt(squared_norm / (static_cast<double>(nonzero) / 2.0))
                       : 0.0;
  };

  const double roll_pitch_scale = std::max(rpyScale(0), rpyScale(1));
  double yaw_scale = -std::numeric_limits<double>::infinity();
  double thrust_scale = 0.0;
  int thrust_nonzero = 0;
  for (const auto &motor : mixer) {
    yaw_scale = std::max(yaw_scale, motor[2]);
    if (std::abs(motor[3]) > std::numeric_limits<float>::epsilon()) {
      thrust_scale += std::abs(motor[3]);
      ++thrust_nonzero;
    }
  }
  thrust_scale = thrust_nonzero > 0 ? thrust_scale / thrust_nonzero : 0.0;

  if (!(roll_pitch_scale > kEps) || !(yaw_scale > kEps) || !(thrust_scale > kEps)) {
    throw std::invalid_argument("invalid F450 allocator normalization scale");
  }
  for (auto &motor : mixer) {
    motor[0] /= roll_pitch_scale;
    motor[1] /= roll_pitch_scale;
    motor[2] /= yaw_scale;
    motor[3] /= thrust_scale;
    for (double &coefficient : motor) {
      if (std::abs(coefficient) < 1e-3) {
        coefficient = 0.0;
      }
    }
  }
  return mixer;
}

bool finiteConfig(const F450WrenchConfig &config) {
  for (std::size_t motor = 0; motor < 4; ++motor) {
    if (!config.allocator_rotor_position_frd_m[motor].finite() ||
        !config.physical_rotor_position_frd_m[motor].finite() ||
        !std::isfinite(config.allocator_thrust_coefficient[motor]) ||
        !std::isfinite(config.allocator_yaw_moment_ratio[motor]) ||
        !std::isfinite(config.physical_yaw_moment_ratio[motor]) ||
        !(config.allocator_thrust_coefficient[motor] > 0.0)) {
      return false;
    }
  }
  return std::isfinite(config.motor_thrust_constant_n_per_radps2) &&
         config.motor_thrust_constant_n_per_radps2 > 0.0 &&
         std::isfinite(config.esc_speed_min_radps) && config.esc_speed_min_radps >= 0.0 &&
         std::isfinite(config.esc_speed_max_radps) &&
         config.esc_speed_max_radps > config.esc_speed_min_radps &&
         std::isfinite(config.collective_force_tolerance_n) &&
         config.collective_force_tolerance_n > 0.0 &&
         std::isfinite(config.moment_tolerance_nm) && config.moment_tolerance_nm > 0.0 &&
         std::isfinite(config.jacobian_condition_limit) &&
         config.jacobian_condition_limit > 1.0 && config.max_iterations > 0;
}

double infinityNorm(const Mat3 &matrix) {
  double norm = 0.0;
  for (std::size_t row = 0; row < 3; ++row) {
    double row_sum = 0.0;
    for (std::size_t column = 0; column < 3; ++column) {
      row_sum += std::abs(matrix(row, column));
    }
    norm = std::max(norm, row_sum);
  }
  return norm;
}

bool withinNormalizedTorqueBounds(const Vec3 &torque) {
  return std::abs(torque.x) <= 1.0 && std::abs(torque.y) <= 1.0 &&
         std::abs(torque.z) <= 1.0;
}

}  // namespace

F450WrenchConfig frozenF450WrenchConfig() {
  F450WrenchConfig config{};
  config.allocator_rotor_position_frd_m = {
      Vec3{0.159, 0.159, 0.0}, Vec3{-0.159, -0.159, 0.0},
      Vec3{0.159, -0.159, 0.0}, Vec3{-0.159, 0.159, 0.0}};
  config.allocator_thrust_coefficient = {6.5, 6.5, 6.5, 6.5};
  config.allocator_yaw_moment_ratio = {0.014, 0.014, -0.014, -0.014};

  // The SDF positions are FLU. This is the single conversion to body FRD.
  constexpr double physical_arm_m = 0.1626345596714;
  config.physical_rotor_position_frd_m = {
      Vec3{physical_arm_m, physical_arm_m, 0.0},
      Vec3{-physical_arm_m, -physical_arm_m, 0.0},
      Vec3{physical_arm_m, -physical_arm_m, 0.0},
      Vec3{-physical_arm_m, physical_arm_m, 0.0}};
  config.physical_yaw_moment_ratio = {0.0137, 0.0137, -0.0137, -0.0137};
  config.motor_thrust_constant_n_per_radps2 = 1.2e-5;
  config.esc_speed_min_radps = 150.0;
  config.esc_speed_max_radps = 1000.0;
  config.collective_force_tolerance_n = 1e-6;
  config.moment_tolerance_nm = 1e-8;
  config.jacobian_condition_limit = 1e4;
  config.max_iterations = 20;
  return config;
}

F450WrenchModel::F450WrenchModel(F450WrenchConfig config) : config_(config) {
  if (!finiteConfig(config_)) {
    throw std::invalid_argument("invalid F450 wrench configuration");
  }
  normalized_mixer_ = makeNormalizedMixer(config_);
}

F450ForwardState F450WrenchModel::forward(const Vec3 &normalized_torque_frd,
                                          double normalized_collective_thrust) const {
  F450ForwardState state{};
  state.normalized_torque_frd = normalized_torque_frd;
  state.normalized_collective_thrust = normalized_collective_thrust;
  if (!normalized_torque_frd.finite() || !std::isfinite(normalized_collective_thrust) ||
      normalized_collective_thrust < 0.0 || normalized_collective_thrust > 1.0 ||
      !withinNormalizedTorqueBounds(normalized_torque_frd)) {
    state.reason = "normalized wrench is outside PX4 bounds";
    return state;
  }

  const std::array<double, 4> control{normalized_torque_frd.x, normalized_torque_frd.y,
                                      normalized_torque_frd.z,
                                      -normalized_collective_thrust};
  const double speed_span_radps = config_.esc_speed_max_radps - config_.esc_speed_min_radps;
  for (std::size_t motor = 0; motor < 4; ++motor) {
    for (std::size_t axis = 0; axis < 4; ++axis) {
      state.motor_command[motor] += normalized_mixer_[motor][axis] * control[axis];
    }
    if (state.motor_command[motor] < 0.0 || state.motor_command[motor] > 1.0) {
      state.reason = "raw PX4 allocation requires desaturation";
      return state;
    }

    state.rotor_speed_radps[motor] =
        config_.esc_speed_min_radps + speed_span_radps * state.motor_command[motor];
    state.rotor_thrust_n[motor] = config_.motor_thrust_constant_n_per_radps2 *
                                  state.rotor_speed_radps[motor] *
                                  state.rotor_speed_radps[motor];
    state.collective_thrust_n += state.rotor_thrust_n[motor];

    const Vec3 &position = config_.physical_rotor_position_frd_m[motor];
    const double thrust_n = state.rotor_thrust_n[motor];
    state.body_moment_frd_nm.x += -position.y * thrust_n;
    state.body_moment_frd_nm.y += position.x * thrust_n;
    state.body_moment_frd_nm.z += config_.physical_yaw_moment_ratio[motor] * thrust_n;
  }

  state.ok = true;
  return state;
}

F450WrenchResult F450WrenchModel::normalize(
    double desired_collective_thrust_n, const Vec3 &desired_body_moment_frd_nm,
    double normalized_collective_thrust) const {
  F450WrenchResult result{};
  if (!std::isfinite(desired_collective_thrust_n) || desired_collective_thrust_n < 0.0 ||
      !desired_body_moment_frd_nm.finite() ||
      !std::isfinite(normalized_collective_thrust) ||
      normalized_collective_thrust < 0.0 || normalized_collective_thrust > 1.0) {
    result.reason = "invalid physical wrench or normalized thrust";
    return result;
  }
  result.normalized_thrust_body_frd = {0.0, 0.0, -normalized_collective_thrust};

  auto capture = [&](const F450ForwardState &state, int iterations) {
    result.normalized_torque_frd = state.normalized_torque_frd;
    result.motor_command = state.motor_command;
    result.rotor_speed_radps = state.rotor_speed_radps;
    result.rotor_thrust_n = state.rotor_thrust_n;
    result.reconstructed_collective_thrust_n = state.collective_thrust_n;
    result.reconstructed_body_moment_frd_nm = state.body_moment_frd_nm;
    result.collective_force_residual_n =
        state.collective_thrust_n - desired_collective_thrust_n;
    result.iterations = iterations;
  };

  Vec3 torque{};
  for (int iteration = 0; iteration <= config_.max_iterations; ++iteration) {
    const F450ForwardState state = forward(torque, normalized_collective_thrust);
    if (!state.ok) {
      result.status = F450WrenchStatus::infeasible;
      result.reason = state.reason;
      return result;
    }

    const Vec3 moment_residual = desired_body_moment_frd_nm - state.body_moment_frd_nm;
    if (moment_residual.norm() <= config_.moment_tolerance_nm) {
      capture(state, iteration);
      if (std::abs(result.collective_force_residual_n) >
          config_.collective_force_tolerance_n) {
        result.status = F450WrenchStatus::infeasible;
        result.reason =
            "physical collective force does not match the common PX4 thrust command";
        return result;
      }
      result.ok = true;
      result.status = F450WrenchStatus::success;
      result.reason.clear();
      return result;
    }
    if (iteration == config_.max_iterations) {
      result.status = F450WrenchStatus::no_convergence;
      result.reason = "physical moment solve did not converge";
      return result;
    }

    Mat3 jacobian = Mat3::zero();
    const double speed_span_radps =
        config_.esc_speed_max_radps - config_.esc_speed_min_radps;
    for (std::size_t motor = 0; motor < 4; ++motor) {
      const Vec3 moment_per_thrust{-config_.physical_rotor_position_frd_m[motor].y,
                                   config_.physical_rotor_position_frd_m[motor].x,
                                   config_.physical_yaw_moment_ratio[motor]};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double thrust_derivative =
            2.0 * config_.motor_thrust_constant_n_per_radps2 *
            state.rotor_speed_radps[motor] * speed_span_radps *
            normalized_mixer_[motor][axis];
        jacobian(0, axis) += moment_per_thrust.x * thrust_derivative;
        jacobian(1, axis) += moment_per_thrust.y * thrust_derivative;
        jacobian(2, axis) += moment_per_thrust.z * thrust_derivative;
      }
    }

    Mat3 inverse_jacobian;
    try {
      inverse_jacobian = inverse(jacobian);
    } catch (const std::invalid_argument &) {
      result.status = F450WrenchStatus::singular_jacobian;
      result.reason = "singular physical moment Jacobian";
      return result;
    }
    const double condition = infinityNorm(jacobian) * infinityNorm(inverse_jacobian);
    if (!std::isfinite(condition) || condition > config_.jacobian_condition_limit) {
      result.status = F450WrenchStatus::singular_jacobian;
      result.reason = "poorly conditioned physical moment Jacobian";
      return result;
    }

    const Vec3 newton_step = inverse_jacobian * moment_residual;
    bool step_accepted = false;
    double step_scale = 1.0;
    for (int backtrack = 0; backtrack < 30; ++backtrack) {
      const Vec3 candidate_torque = torque + step_scale * newton_step;
      if (withinNormalizedTorqueBounds(candidate_torque)) {
        const F450ForwardState candidate =
            forward(candidate_torque, normalized_collective_thrust);
        if (candidate.ok &&
            (desired_body_moment_frd_nm - candidate.body_moment_frd_nm).norm() <
                moment_residual.norm()) {
          torque = candidate_torque;
          step_accepted = true;
          break;
        }
      }
      step_scale *= 0.5;
    }
    if (!step_accepted) {
      result.status = F450WrenchStatus::infeasible;
      result.reason = "physical moment request exceeds raw allocator bounds";
      return result;
    }
  }

  result.status = F450WrenchStatus::no_convergence;
  result.reason = "physical moment solve did not converge";
  return result;
}

}  // namespace control
