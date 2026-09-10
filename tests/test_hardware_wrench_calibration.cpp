#include "control/hardware_wrench_calibration.hpp"
#include "test_support.hpp"

#include <sstream>
#include <stdexcept>
#include <string>

using namespace control;

namespace {

const char *kValidRecord = R"(schema_version=1
authority=measured_hardware
vehicle_id=f450_lab_01
calibration_date_utc=2026-09-01
method=static_wrench_stand
source_data_sha256=0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef
input_units=px4_normalized_thrust_torque
output_units=N_Nm
accept_minimum_sample_count=50
accept_maximum_condition_number=100
accept_maximum_collective_rmse_n=0.1
accept_maximum_moment_rmse_nm=0.01
accept_maximum_collective_residual_n=0.2
accept_maximum_moment_residual_nm=0.02
thrust_min=0.2
thrust_max=0.8
torque_x_min=-0.3
torque_x_max=0.3
torque_y_min=-0.3
torque_y_max=0.3
torque_z_min=-0.3
torque_z_max=0.3
sample_count=120
design_condition_number=10
rmse_collective_n=0.02
rmse_moment_x_nm=0.001
rmse_moment_y_nm=0.001
rmse_moment_z_nm=0.001
max_residual_collective_n=0.05
max_residual_moment_x_nm=0.004
max_residual_moment_y_nm=0.004
max_residual_moment_z_nm=0.004
coefficient_collective=1,30,0.2,0.1,0.05
coefficient_moment_x=0.01,0.02,5,0.1,0.2
coefficient_moment_y=-0.02,0.01,0.1,4,0.3
coefficient_moment_z=0,0.005,0.2,0.3,2
)";

HardwareCalibrationLimits limits() {
  return {50, 100.0, 0.1, {0.01, 0.01, 0.01},
          0.2, {0.02, 0.02, 0.02}, 30, {1e-10, 1e-10, 1e-10}};
}

std::string replaced(std::string text, const std::string &from,
                     const std::string &to) {
  const std::size_t position = text.find(from);
  check(position != std::string::npos, "fixture mutation target exists");
  text.replace(position, from.size(), to);
  return text;
}

void expectReject(const std::string &record, const std::string &message) {
  bool rejected = false;
  try {
    (void)HardwareWrenchCalibration::loadFromText(
        record, "f450_lab_01", "2026-09-10", limits());
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, message);
}

void checkVecNear(const Vec3 &actual, const Vec3 &expected, double tolerance,
                  const std::string &message) {
  checkNear(actual.x, expected.x, tolerance, message + " x");
  checkNear(actual.y, expected.y, tolerance, message + " y");
  checkNear(actual.z, expected.z, tolerance, message + " z");
}

}  // namespace

int main() {
  const auto calibration = HardwareWrenchCalibration::loadFromText(
      kValidRecord, "f450_lab_01", "2026-09-10", limits());
  const auto result = calibration.normalize(19.0, {0.121, -0.129, 0.018}, 0.60);
  check(result.ok, result.reason);
  checkVecNear(result.normalized_torque_frd, {0.02, -0.03, 0.01}, 1e-12,
               "measured affine moment inversion");
  checkNear(result.normalized_thrust_body_frd.z, -0.60, 0.0,
            "common thrust remains unchanged");
  checkVecNear(result.reconstructed_body_moment_frd_nm,
               {0.121, -0.129, 0.018}, 1e-12,
               "measured moment reconstruction");
  checkNear(result.reconstructed_collective_thrust_n, 19.0015, 1e-12,
            "measured collective reconstruction");
  checkNear(result.collective_force_residual_n, 0.0015, 1e-12,
            "measured collective residual");

  bool missing_file_rejected = false;
  try {
    (void)HardwareWrenchCalibration::loadFromFile(
        "/file/that/does/not/exist", "f450_lab_01", "2026-09-10", limits());
  } catch (const std::invalid_argument &) {
    missing_file_rejected = true;
  }
  check(missing_file_rejected, "missing hardware record fails closed");

  const auto repeated = calibration.normalize(19.0, {0.121, -0.129, 0.018}, 0.60);
  checkVecNear(repeated.normalized_torque_frd, result.normalized_torque_frd, 0.0,
               "hardware inversion is deterministic");

  const auto outside_thrust = calibration.normalize(19.0, {}, 0.81);
  check(!outside_thrust.ok &&
            outside_thrust.status == HardwareWrenchStatus::outside_calibrated_range,
        "thrust outside the measured range is rejected");
  const auto outside_torque = calibration.normalize(19.0, {10.0, 0.0, 0.0}, 0.60);
  check(!outside_torque.ok &&
            outside_torque.status == HardwareWrenchStatus::outside_calibrated_range,
        "solved torque outside the measured range is rejected");

  std::istringstream valid_lines(kValidRecord);
  std::string required_line;
  while (std::getline(valid_lines, required_line)) {
    expectReject(replaced(kValidRecord, required_line + "\n", ""),
                 "every record field is required: " + required_line);
  }
  expectReject(replaced(kValidRecord, "authority=measured_hardware",
                        "authority=simulation"),
               "simulation authority");
  expectReject(replaced(kValidRecord, "vehicle_id=f450_lab_01",
                        "vehicle_id=another_vehicle"),
               "wrong vehicle");
  expectReject(replaced(kValidRecord, "output_units=N_Nm", "output_units=SI"),
               "wrong units");
  expectReject(replaced(kValidRecord,
                        "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
                        "not-a-sha"),
               "malformed source hash");
  expectReject(replaced(kValidRecord, "calibration_date_utc=2026-09-01",
                        "calibration_date_utc=2026-09-11"),
               "future calibration");
  expectReject(replaced(kValidRecord, "calibration_date_utc=2026-09-01",
                        "calibration_date_utc=2026-01-01"),
               "stale calibration");
  expectReject(replaced(kValidRecord, "sample_count=120", "sample_count=20"),
               "insufficient samples");
  expectReject(replaced(kValidRecord, "accept_minimum_sample_count=50",
                        "accept_minimum_sample_count=20"),
               "permissive fitting sample threshold");
  expectReject(replaced(kValidRecord, "accept_maximum_condition_number=100",
                        "accept_maximum_condition_number=1000"),
               "permissive fitting condition threshold");
  expectReject(replaced(kValidRecord, "design_condition_number=10",
                        "design_condition_number=1000"),
               "poor design condition");
  expectReject(replaced(kValidRecord, "rmse_collective_n=0.02",
                        "rmse_collective_n=0.2"),
               "excessive fit RMSE");
  expectReject(replaced(kValidRecord, "rmse_collective_n=0.02",
                        "rmse_collective_n=-0.02"),
               "negative fit metric");
  expectReject(replaced(kValidRecord, "max_residual_moment_x_nm=0.004",
                        "max_residual_moment_x_nm=0.04"),
               "excessive fit residual");
  expectReject(replaced(kValidRecord, "coefficient_moment_x=0.01,0.02,5,0.1,0.2",
                        "coefficient_moment_x=0.01,0.02,nan,0.1,0.2"),
               "non-finite coefficient");
  expectReject(replaced(kValidRecord, "coefficient_moment_y=-0.02,0.01,0.1,4,0.3",
                        "coefficient_moment_y=-0.02,0.01,10,0.2,0.4"),
               "singular torque block");
  expectReject(std::string(kValidRecord) + "authority=measured_hardware\n",
               "duplicate key");
  expectReject(std::string(kValidRecord) + "unreviewed_field=1\n", "unknown key");
  expectReject(replaced(kValidRecord, "thrust_min=0.2", "thrust_min=0.9"),
               "invalid operating range");
  return 0;
}
