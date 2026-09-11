#include "px4_offboard_controllers/vehicles/f450/wrench_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace px4_offboard::vehicles::f450 {
namespace {

using Matrix4 = std::array<std::array<double, 4>, 4>;

Matrix4 invert4(Matrix4 matrix) {
  Matrix4 inverse_matrix{};
  for (std::size_t row = 0; row < 4; ++row) {
    inverse_matrix[row][row] = 1.0;
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
    std::swap(inverse_matrix[column], inverse_matrix[pivot]);
    const double diagonal = matrix[column][column];
    for (std::size_t element = 0; element < 4; ++element) {
      matrix[column][element] /= diagonal;
      inverse_matrix[column][element] /= diagonal;
    }

    for (std::size_t row = 0; row < 4; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = matrix[row][column];
      for (std::size_t element = 0; element < 4; ++element) {
        matrix[row][element] -= factor * matrix[column][element];
        inverse_matrix[row][element] -= factor * inverse_matrix[column][element];
      }
    }
  }

  return inverse_matrix;
}

Matrix4 makeMixer(const F450WrenchConfig &config) {
  Matrix4 effectiveness{};
  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    const double thrust_coefficient = config.allocator_thrust_coefficient[rotor];
    const auto &position = config.allocator_rotor_position_frd_m[rotor];
    effectiveness[0][rotor] = -thrust_coefficient * position.y;
    effectiveness[1][rotor] = thrust_coefficient * position.x;
    effectiveness[2][rotor] =
        thrust_coefficient * config.allocator_yaw_moment_ratio[rotor];
    effectiveness[3][rotor] = -thrust_coefficient;
  }

  auto mixer = invert4(effectiveness);
  const auto axisScale = [&](std::size_t axis) {
    double squared_norm = 0.0;
    int non_zero = 0;
    for (const auto &row : mixer) {
      squared_norm += row[axis] * row[axis];
      if (std::abs(row[axis]) > 1e-3) {
        ++non_zero;
      }
    }
    return non_zero ? std::sqrt(squared_norm / (static_cast<double>(non_zero) / 2.0)) : 0.0;
  };

  const double roll_pitch_scale = std::max(axisScale(0), axisScale(1));
  double yaw_scale = -std::numeric_limits<double>::infinity();
  double thrust_scale = 0.0;
  int thrust_non_zero = 0;
  for (const auto &row : mixer) {
    yaw_scale = std::max(yaw_scale, row[2]);
    if (std::abs(row[3]) > std::numeric_limits<float>::epsilon()) {
      thrust_scale += std::abs(row[3]);
      ++thrust_non_zero;
    }
  }
  thrust_scale = thrust_non_zero ? thrust_scale / thrust_non_zero : 0.0;

  if (!(roll_pitch_scale > kEps && yaw_scale > kEps && thrust_scale > kEps)) {
    throw std::invalid_argument("invalid F450 allocator normalization scale");
  }

  // Preserve the frozen PX4 pseudo-inverse normalization used by the comparison baseline.
  for (auto &row : mixer) {
    row[0] /= roll_pitch_scale;
    row[1] /= roll_pitch_scale;
    row[2] /= yaw_scale;
    row[3] /= thrust_scale;
    for (double &value : row) {
      if (std::abs(value) < 1e-3) {
        value = 0.0;
      }
    }
  }
  return mixer;
}

bool finiteConfig(const F450WrenchConfig &config) {
  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    if (!config.allocator_rotor_position_frd_m[rotor].finite() ||
        !config.physical_rotor_position_frd_m[rotor].finite() ||
        !std::isfinite(config.allocator_thrust_coefficient[rotor]) ||
        !std::isfinite(config.allocator_yaw_moment_ratio[rotor]) ||
        !std::isfinite(config.physical_yaw_moment_ratio[rotor]) ||
        !(config.allocator_thrust_coefficient[rotor] > 0.0)) {
      return false;
    }
  }

  return std::isfinite(config.motor_thrust_constant_n_per_radps2) &&
         config.motor_thrust_constant_n_per_radps2 > 0.0 &&
         std::isfinite(config.esc_speed_min_radps) && config.esc_speed_min_radps >= 0.0 &&
         std::isfinite(config.esc_speed_max_radps) &&
         config.esc_speed_max_radps > config.esc_speed_min_radps &&
         std::isfinite(config.collective_force_tolerance_n) &&
         config.collective_force_tolerance_n > 0.0 && std::isfinite(config.moment_tolerance_nm) &&
         config.moment_tolerance_nm > 0.0 &&
         std::isfinite(config.jacobian_condition_limit) && config.jacobian_condition_limit > 1.0 &&
         config.max_iterations > 0;
}

double infinityNorm(const Mat3 &matrix) {
  double norm = 0.0;
  for (std::size_t row = 0; row < 3; ++row) {
    double sum = 0.0;
    for (std::size_t column = 0; column < 3; ++column) {
      sum += std::abs(matrix(row, column));
    }
    norm = std::max(norm, sum);
  }
  return norm;
}

bool inBounds(const Vec3 &normalized_torque_frd) {
  return std::abs(normalized_torque_frd.x) <= 1.0 &&
         std::abs(normalized_torque_frd.y) <= 1.0 &&
         std::abs(normalized_torque_frd.z) <= 1.0;
}

}  // namespace

F450WrenchConfig frozenF450WrenchConfig() {
  F450WrenchConfig config{};
  config.allocator_rotor_position_frd_m =
      {Vec3{0.159, 0.159, 0.0}, Vec3{-0.159, -0.159, 0.0},
       Vec3{0.159, -0.159, 0.0}, Vec3{-0.159, 0.159, 0.0}};
  config.allocator_thrust_coefficient = {6.5, 6.5, 6.5, 6.5};
  config.allocator_yaw_moment_ratio = {0.014, 0.014, -0.014, -0.014};

  constexpr double physical_arm_m = 0.1626345596714;
  config.physical_rotor_position_frd_m =
      {Vec3{physical_arm_m, physical_arm_m, 0.0},
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
  normalized_mixer_ = makeMixer(config_);
}

F450ForwardState F450WrenchModel::forward(const Vec3 &normalized_torque_frd,
                                          double normalized_collective_thrust) const {
  F450ForwardState state{};
  state.normalized_torque_frd = normalized_torque_frd;
  state.normalized_collective_thrust = normalized_collective_thrust;
  if (!normalized_torque_frd.finite() || !std::isfinite(normalized_collective_thrust) ||
      normalized_collective_thrust < 0.0 || normalized_collective_thrust > 1.0 ||
      !inBounds(normalized_torque_frd)) {
    state.reason = "normalized wrench is outside PX4 bounds";
    return state;
  }

  const std::array<double, 4> control{normalized_torque_frd.x, normalized_torque_frd.y,
                                      normalized_torque_frd.z,
                                      -normalized_collective_thrust};
  const double speed_span = config_.esc_speed_max_radps - config_.esc_speed_min_radps;
  for (std::size_t motor = 0; motor < 4; ++motor) {
    for (std::size_t axis = 0; axis < 4; ++axis) {
      state.motor_command[motor] += normalized_mixer_[motor][axis] * control[axis];
    }
    if (state.motor_command[motor] < 0.0 || state.motor_command[motor] > 1.0) {
      state.reason = "raw PX4 allocation requires desaturation";
      return state;
    }

    state.rotor_speed_radps[motor] =
        config_.esc_speed_min_radps + speed_span * state.motor_command[motor];
    state.rotor_thrust_n[motor] =
        config_.motor_thrust_constant_n_per_radps2 * state.rotor_speed_radps[motor] *
        state.rotor_speed_radps[motor];
    state.collective_thrust_n += state.rotor_thrust_n[motor];

    const auto &position = config_.physical_rotor_position_frd_m[motor];
    const double thrust = state.rotor_thrust_n[motor];
    state.body_moment_frd_nm.x += -position.y * thrust;
    state.body_moment_frd_nm.y += position.x * thrust;
    state.body_moment_frd_nm.z += config_.physical_yaw_moment_ratio[motor] * thrust;
  }

  state.ok = true;
  return state;
}

F450WrenchResult F450WrenchModel::normalize(double desired_collective_force_n,
                                            const Vec3 &desired_body_moment_frd_nm,
                                            double normalized_collective_thrust) const {
  F450WrenchResult result{};
  if (!std::isfinite(desired_collective_force_n) || desired_collective_force_n < 0.0 ||
      !desired_body_moment_frd_nm.finite() || !std::isfinite(normalized_collective_thrust) ||
      normalized_collective_thrust < 0.0 || normalized_collective_thrust > 1.0) {
    result.reason = "invalid physical wrench or normalized thrust";
    return result;
  }

  result.normalized_thrust_body_frd = {0.0, 0.0, -normalized_collective_thrust};
  const auto capture = [&](const F450ForwardState &state, int iterations) {
    result.normalized_torque_frd = state.normalized_torque_frd;
    result.motor_command = state.motor_command;
    result.rotor_speed_radps = state.rotor_speed_radps;
    result.rotor_thrust_n = state.rotor_thrust_n;
    result.reconstructed_collective_thrust_n = state.collective_thrust_n;
    result.reconstructed_body_moment_frd_nm = state.body_moment_frd_nm;
    result.collective_force_residual_n =
        state.collective_thrust_n - desired_collective_force_n;
    result.iterations = iterations;
  };

  Vec3 normalized_torque{};
  for (int iteration = 0; iteration <= config_.max_iterations; ++iteration) {
    const auto state = forward(normalized_torque, normalized_collective_thrust);
    if (!state.ok) {
      result.status = F450WrenchStatus::infeasible;
      result.reason = state.reason;
      return result;
    }

    const Vec3 residual = desired_body_moment_frd_nm - state.body_moment_frd_nm;
    if (residual.norm() <= config_.moment_tolerance_nm) {
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

    // Differentiate the physical rotor wrench through the frozen normalized mixer so Newton's
    // step is solved in PX4 normalized torque coordinates.
    Mat3 jacobian = Mat3::zero();
    const double speed_span = config_.esc_speed_max_radps - config_.esc_speed_min_radps;
    for (std::size_t motor = 0; motor < 4; ++motor) {
      const Vec3 moment_per_force{-config_.physical_rotor_position_frd_m[motor].y,
                                  config_.physical_rotor_position_frd_m[motor].x,
                                  config_.physical_yaw_moment_ratio[motor]};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double thrust_derivative =
            2.0 * config_.motor_thrust_constant_n_per_radps2 *
            state.rotor_speed_radps[motor] * speed_span * normalized_mixer_[motor][axis];
        jacobian(0, axis) += moment_per_force.x * thrust_derivative;
        jacobian(1, axis) += moment_per_force.y * thrust_derivative;
        jacobian(2, axis) += moment_per_force.z * thrust_derivative;
      }
    }

    Mat3 jacobian_inverse;
    try {
      jacobian_inverse = inverse(jacobian);
    } catch (const std::invalid_argument &) {
      result.status = F450WrenchStatus::singular_jacobian;
      result.reason = "singular physical moment Jacobian";
      return result;
    }

    const double condition = infinityNorm(jacobian) * infinityNorm(jacobian_inverse);
    if (!std::isfinite(condition) || condition > config_.jacobian_condition_limit) {
      result.status = F450WrenchStatus::singular_jacobian;
      result.reason = "poorly conditioned physical moment Jacobian";
      return result;
    }

    const Vec3 step = jacobian_inverse * residual;
    bool accepted = false;
    double scale = 1.0;
    for (int backtrack = 0; backtrack < 30; ++backtrack) {
      const Vec3 candidate = normalized_torque + scale * step;
      if (inBounds(candidate)) {
        const auto candidate_state = forward(candidate, normalized_collective_thrust);
        if (candidate_state.ok &&
            (desired_body_moment_frd_nm - candidate_state.body_moment_frd_nm).norm() <
                residual.norm()) {
          normalized_torque = candidate;
          accepted = true;
          break;
        }
      }
      scale *= 0.5;
    }

    if (!accepted) {
      result.status = F450WrenchStatus::infeasible;
      result.reason = "physical moment request exceeds raw allocator bounds";
      return result;
    }
  }

  result.status = F450WrenchStatus::no_convergence;
  result.reason = "physical moment solve did not converge";
  return result;
}

}  // namespace px4_offboard::vehicles::f450
