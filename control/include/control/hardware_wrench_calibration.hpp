#pragma once

#include "control/math.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace control {

struct HardwareCalibrationLimits {
  std::size_t minimum_sample_count;
  double maximum_condition_number;
  double maximum_collective_rmse_n;
  Vec3 maximum_moment_rmse_nm;
  double maximum_collective_residual_n;
  Vec3 maximum_moment_residual_nm;
  int maximum_age_days;
  Vec3 maximum_reconstruction_error_nm;
};

enum class HardwareWrenchStatus {
  success,
  invalid_input,
  outside_calibrated_range,
  reconstruction_failure,
};

struct HardwareWrenchResult {
  bool ok{false};
  HardwareWrenchStatus status{HardwareWrenchStatus::invalid_input};
  std::string reason;
  Vec3 normalized_torque_frd{};
  Vec3 normalized_thrust_body_frd{};
  double reconstructed_collective_thrust_n{0.0};
  Vec3 reconstructed_body_moment_frd_nm{};
  double collective_force_residual_n{0.0};
};

class HardwareWrenchCalibration {
 public:
  static HardwareWrenchCalibration loadFromText(
      const std::string &record, const std::string &expected_vehicle_id,
      const std::string &current_date_utc, const HardwareCalibrationLimits &limits);
  static HardwareWrenchCalibration loadFromFile(
      const std::string &path, const std::string &expected_vehicle_id,
      const std::string &current_date_utc, const HardwareCalibrationLimits &limits);

  HardwareWrenchResult normalize(double desired_collective_thrust_n,
                                 const Vec3 &desired_body_moment_frd_nm,
                                 double normalized_collective_thrust) const;

 private:
  std::array<std::array<double, 5>, 4> coefficients_{};
  double thrust_min_{0.0};
  double thrust_max_{0.0};
  Vec3 torque_min_{};
  Vec3 torque_max_{};
  Mat3 inverse_moment_torque_block_{};
  Vec3 maximum_reconstruction_error_nm_{};
};

}  // namespace control
