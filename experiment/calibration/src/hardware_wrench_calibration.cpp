#include "px4_offboard_controllers/calibration/hardware_wrench_calibration.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace px4_offboard::calibration {
namespace {

const std::vector<std::string> kRequiredKeys{
    "schema_version", "authority", "vehicle_id", "calibration_date_utc",
    "method", "source_data_sha256", "input_units", "output_units",
    "accept_minimum_sample_count", "accept_maximum_condition_number",
    "accept_maximum_collective_rmse_n", "accept_maximum_moment_rmse_nm",
    "accept_maximum_collective_residual_n", "accept_maximum_moment_residual_nm",
    "thrust_min", "thrust_max", "torque_x_min", "torque_x_max",
    "torque_y_min", "torque_y_max", "torque_z_min", "torque_z_max",
    "sample_count", "design_condition_number", "rmse_collective_n",
    "rmse_moment_x_nm", "rmse_moment_y_nm", "rmse_moment_z_nm",
    "max_residual_collective_n", "max_residual_moment_x_nm",
    "max_residual_moment_y_nm", "max_residual_moment_z_nm",
    "coefficient_collective", "coefficient_moment_x", "coefficient_moment_y",
    "coefficient_moment_z"};

bool isKnownKey(const std::string &key) {
  return std::find(kRequiredKeys.begin(), kRequiredKeys.end(), key) != kRequiredKeys.end();
}

std::map<std::string, std::string> parseRecord(const std::string &record) {
  std::map<std::string, std::string> fields;
  std::istringstream lines(record);
  std::string line;
  while (std::getline(lines, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      throw std::invalid_argument("empty calibration record line");
    }
    const std::size_t separator = line.find('=');
    if (separator == std::string::npos || separator == 0 || separator + 1 == line.size() ||
        line.find('=', separator + 1) != std::string::npos) {
      throw std::invalid_argument("malformed calibration record line");
    }
    const std::string key = line.substr(0, separator);
    const std::string value = line.substr(separator + 1);
    if (!isKnownKey(key)) {
      throw std::invalid_argument("unknown calibration field: " + key);
    }
    if (!fields.emplace(key, value).second) {
      throw std::invalid_argument("duplicate calibration field: " + key);
    }
  }
  for (const std::string &key : kRequiredKeys) {
    if (fields.count(key) == 0) {
      throw std::invalid_argument("missing calibration field: " + key);
    }
  }
  return fields;
}

double parseDouble(const std::string &text, const std::string &field) {
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  double value = 0.0;
  stream >> std::noskipws >> value;
  if (!stream || !stream.eof() || !std::isfinite(value)) {
    throw std::invalid_argument("invalid finite number in " + field);
  }
  return value;
}

int parseInt(const std::string &text, const std::string &field) {
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  int value = 0;
  stream >> std::noskipws >> value;
  if (!stream || !stream.eof()) {
    throw std::invalid_argument("invalid integer in " + field);
  }
  return value;
}

std::array<double, 5> parseCoefficients(const std::string &text, const std::string &field) {
  std::array<double, 5> coefficients{};
  std::istringstream entries(text);
  std::string entry;
  std::size_t index = 0;
  while (std::getline(entries, entry, ',')) {
    if (index >= coefficients.size()) {
      throw std::invalid_argument("too many coefficients in " + field);
    }
    coefficients[index++] = parseDouble(entry, field);
  }
  if (index != coefficients.size() || (!text.empty() && text.back() == ',')) {
    throw std::invalid_argument("wrong coefficient count in " + field);
  }
  return coefficients;
}

bool leapYear(int year) {
  return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

long long daysFromCivil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era = static_cast<unsigned>(year - era * 400);
  const unsigned shifted_month = month > 2 ? month - 3 : month + 9;
  const unsigned day_of_year = (153 * shifted_month + 2) / 5 + day - 1;
  const unsigned day_of_era =
      year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
  return static_cast<long long>(era) * 146097 + static_cast<long long>(day_of_era);
}

long long parseDate(const std::string &text, const std::string &field) {
  if (text.size() != 10 || text[4] != '-' || text[7] != '-') {
    throw std::invalid_argument("invalid UTC date in " + field);
  }
  const int year = parseInt(text.substr(0, 4), field);
  const int month = parseInt(text.substr(5, 2), field);
  const int day = parseInt(text.substr(8, 2), field);
  constexpr std::array<int, 12> month_days{31, 28, 31, 30, 31, 30,
                                            31, 31, 30, 31, 30, 31};
  if (year < 1970 || month < 1 || month > 12) {
    throw std::invalid_argument("invalid UTC date in " + field);
  }
  int maximum_day = month_days[static_cast<std::size_t>(month - 1)];
  if (month == 2 && leapYear(year)) {
    ++maximum_day;
  }
  if (day < 1 || day > maximum_day) {
    throw std::invalid_argument("invalid UTC date in " + field);
  }
  return daysFromCivil(year, static_cast<unsigned>(month), static_cast<unsigned>(day));
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

void validateIdentity(const std::string &value, const std::string &field) {
  if (value.empty() || std::isspace(static_cast<unsigned char>(value.front())) != 0 ||
      std::isspace(static_cast<unsigned char>(value.back())) != 0) {
    throw std::invalid_argument("invalid identity field: " + field);
  }
}

}  // namespace

HardwareWrenchCalibration HardwareWrenchCalibration::loadFromText(
    const std::string &record, const std::string &expected_vehicle_id,
    const std::string &current_date_utc, const HardwareCalibrationLimits &limits) {
  if (limits.minimum_sample_count == 0 || !(limits.maximum_condition_number > 1.0) ||
      !(limits.maximum_collective_rmse_n >= 0.0) || !limits.maximum_moment_rmse_nm.finite() ||
      limits.maximum_moment_rmse_nm.x < 0.0 || limits.maximum_moment_rmse_nm.y < 0.0 ||
      limits.maximum_moment_rmse_nm.z < 0.0 || !(limits.maximum_collective_residual_n >= 0.0) ||
      !limits.maximum_moment_residual_nm.finite() || limits.maximum_moment_residual_nm.x < 0.0 ||
      limits.maximum_moment_residual_nm.y < 0.0 || limits.maximum_moment_residual_nm.z < 0.0 ||
      limits.maximum_age_days < 0 || !limits.maximum_reconstruction_error_nm.finite() ||
      !(limits.maximum_reconstruction_error_nm.x > 0.0) ||
      !(limits.maximum_reconstruction_error_nm.y > 0.0) ||
      !(limits.maximum_reconstruction_error_nm.z > 0.0)) {
    throw std::invalid_argument("invalid hardware calibration acceptance limits");
  }
  const auto fields = parseRecord(record);
  if (fields.at("schema_version") != "1" || fields.at("authority") != "measured_hardware") {
    throw std::invalid_argument("unsupported hardware calibration provenance");
  }
  validateIdentity(fields.at("vehicle_id"), "vehicle_id");
  validateIdentity(fields.at("method"), "method");
  if (expected_vehicle_id.empty() || fields.at("vehicle_id") != expected_vehicle_id) {
    throw std::invalid_argument("hardware calibration vehicle mismatch");
  }
  if (fields.at("input_units") != "px4_normalized_thrust_torque" ||
      fields.at("output_units") != "N_Nm") {
    throw std::invalid_argument("hardware calibration unit mismatch");
  }
  const std::string &sha = fields.at("source_data_sha256");
  if (sha.size() != 64 ||
      !std::all_of(sha.begin(), sha.end(),
                   [](unsigned char character) { return std::isxdigit(character) != 0; })) {
    throw std::invalid_argument("invalid source-data SHA-256");
  }
  const long long calibration_day =
      parseDate(fields.at("calibration_date_utc"), "calibration_date_utc");
  const long long current_day = parseDate(current_date_utc, "current_date_utc");
  const long long age_days = current_day - calibration_day;
  if (age_days < 0 || age_days > limits.maximum_age_days) {
    throw std::invalid_argument("hardware calibration date is future or stale");
  }

  const int recorded_minimum_samples =
      parseInt(fields.at("accept_minimum_sample_count"), "accept_minimum_sample_count");
  const double recorded_maximum_condition =
      parseDouble(fields.at("accept_maximum_condition_number"), "accept_maximum_condition_number");
  const double recorded_maximum_collective_rmse = parseDouble(
      fields.at("accept_maximum_collective_rmse_n"), "accept_maximum_collective_rmse_n");
  const double recorded_maximum_moment_rmse = parseDouble(
      fields.at("accept_maximum_moment_rmse_nm"), "accept_maximum_moment_rmse_nm");
  const double recorded_maximum_collective_residual = parseDouble(
      fields.at("accept_maximum_collective_residual_n"), "accept_maximum_collective_residual_n");
  const double recorded_maximum_moment_residual = parseDouble(
      fields.at("accept_maximum_moment_residual_nm"), "accept_maximum_moment_residual_nm");
  if (recorded_minimum_samples < 0 ||
      static_cast<std::size_t>(recorded_minimum_samples) < limits.minimum_sample_count ||
      !(recorded_maximum_condition > 1.0) ||
      recorded_maximum_condition > limits.maximum_condition_number ||
      recorded_maximum_collective_rmse < 0.0 ||
      recorded_maximum_collective_rmse > limits.maximum_collective_rmse_n ||
      recorded_maximum_moment_rmse < 0.0 ||
      recorded_maximum_moment_rmse > limits.maximum_moment_rmse_nm.x ||
      recorded_maximum_moment_rmse > limits.maximum_moment_rmse_nm.y ||
      recorded_maximum_moment_rmse > limits.maximum_moment_rmse_nm.z ||
      recorded_maximum_collective_residual < 0.0 ||
      recorded_maximum_collective_residual > limits.maximum_collective_residual_n ||
      recorded_maximum_moment_residual < 0.0 ||
      recorded_maximum_moment_residual > limits.maximum_moment_residual_nm.x ||
      recorded_maximum_moment_residual > limits.maximum_moment_residual_nm.y ||
      recorded_maximum_moment_residual > limits.maximum_moment_residual_nm.z) {
    throw std::invalid_argument("recorded fitting limits are too permissive");
  }

  const int sample_count = parseInt(fields.at("sample_count"), "sample_count");
  if (sample_count < recorded_minimum_samples || sample_count < 0 ||
      static_cast<std::size_t>(sample_count) < limits.minimum_sample_count) {
    throw std::invalid_argument("insufficient hardware calibration samples");
  }
  const double design_condition =
      parseDouble(fields.at("design_condition_number"), "design_condition_number");
  if (design_condition < 1.0 || design_condition > recorded_maximum_condition ||
      design_condition > limits.maximum_condition_number) {
    throw std::invalid_argument("hardware calibration design is poorly conditioned");
  }
  const double collective_rmse = parseDouble(fields.at("rmse_collective_n"), "rmse_collective_n");
  const Vec3 moment_rmse{parseDouble(fields.at("rmse_moment_x_nm"), "rmse_moment_x_nm"),
                         parseDouble(fields.at("rmse_moment_y_nm"), "rmse_moment_y_nm"),
                         parseDouble(fields.at("rmse_moment_z_nm"), "rmse_moment_z_nm")};
  const double collective_residual = parseDouble(fields.at("max_residual_collective_n"),
                                                  "max_residual_collective_n");
  const Vec3 moment_residual{
      parseDouble(fields.at("max_residual_moment_x_nm"), "max_residual_moment_x_nm"),
      parseDouble(fields.at("max_residual_moment_y_nm"), "max_residual_moment_y_nm"),
      parseDouble(fields.at("max_residual_moment_z_nm"), "max_residual_moment_z_nm")};
  if (collective_rmse < 0.0 || moment_rmse.x < 0.0 || moment_rmse.y < 0.0 ||
      moment_rmse.z < 0.0 || collective_residual < 0.0 || moment_residual.x < 0.0 ||
      moment_residual.y < 0.0 || moment_residual.z < 0.0 ||
      collective_rmse > std::min(recorded_maximum_collective_rmse,
                                 limits.maximum_collective_rmse_n) ||
      moment_rmse.x > std::min(recorded_maximum_moment_rmse, limits.maximum_moment_rmse_nm.x) ||
      moment_rmse.y > std::min(recorded_maximum_moment_rmse, limits.maximum_moment_rmse_nm.y) ||
      moment_rmse.z > std::min(recorded_maximum_moment_rmse, limits.maximum_moment_rmse_nm.z) ||
      collective_residual > std::min(recorded_maximum_collective_residual,
                                     limits.maximum_collective_residual_n) ||
      moment_residual.x > std::min(recorded_maximum_moment_residual,
                                   limits.maximum_moment_residual_nm.x) ||
      moment_residual.y > std::min(recorded_maximum_moment_residual,
                                   limits.maximum_moment_residual_nm.y) ||
      moment_residual.z > std::min(recorded_maximum_moment_residual,
                                   limits.maximum_moment_residual_nm.z)) {
    throw std::invalid_argument("hardware calibration fit quality exceeds limits");
  }

  HardwareWrenchCalibration calibration;
  calibration.maximum_collective_reconstruction_error_n_ =
      std::min(recorded_maximum_collective_residual, limits.maximum_collective_residual_n);
  calibration.maximum_reconstruction_error_nm_ = limits.maximum_reconstruction_error_nm;
  calibration.thrust_min_ = parseDouble(fields.at("thrust_min"), "thrust_min");
  calibration.thrust_max_ = parseDouble(fields.at("thrust_max"), "thrust_max");
  calibration.torque_min_ = {parseDouble(fields.at("torque_x_min"), "torque_x_min"),
                             parseDouble(fields.at("torque_y_min"), "torque_y_min"),
                             parseDouble(fields.at("torque_z_min"), "torque_z_min")};
  calibration.torque_max_ = {parseDouble(fields.at("torque_x_max"), "torque_x_max"),
                             parseDouble(fields.at("torque_y_max"), "torque_y_max"),
                             parseDouble(fields.at("torque_z_max"), "torque_z_max")};
  if (!(calibration.thrust_min_ >= 0.0) || !(calibration.thrust_min_ < calibration.thrust_max_) ||
      calibration.thrust_max_ > 1.0 || !(calibration.torque_min_.x < calibration.torque_max_.x) ||
      !(calibration.torque_min_.y < calibration.torque_max_.y) ||
      !(calibration.torque_min_.z < calibration.torque_max_.z) ||
      calibration.torque_min_.x < -1.0 || calibration.torque_min_.y < -1.0 ||
      calibration.torque_min_.z < -1.0 || calibration.torque_max_.x > 1.0 ||
      calibration.torque_max_.y > 1.0 || calibration.torque_max_.z > 1.0) {
    throw std::invalid_argument("invalid hardware calibration operating range");
  }

  calibration.coefficients_[0] =
      parseCoefficients(fields.at("coefficient_collective"), "coefficient_collective");
  calibration.coefficients_[1] =
      parseCoefficients(fields.at("coefficient_moment_x"), "coefficient_moment_x");
  calibration.coefficients_[2] =
      parseCoefficients(fields.at("coefficient_moment_y"), "coefficient_moment_y");
  calibration.coefficients_[3] =
      parseCoefficients(fields.at("coefficient_moment_z"), "coefficient_moment_z");

  Mat3 torque_block = Mat3::zero();
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      torque_block(row, column) = calibration.coefficients_[row + 1][column + 2];
    }
  }
  try {
    calibration.inverse_moment_torque_block_ = inverse(torque_block);
  } catch (const std::invalid_argument &) {
    throw std::invalid_argument("singular hardware moment calibration");
  }
  const double torque_condition =
      infinityNorm(torque_block) * infinityNorm(calibration.inverse_moment_torque_block_);
  if (!std::isfinite(torque_condition) || torque_condition > limits.maximum_condition_number) {
    throw std::invalid_argument("hardware moment calibration is poorly conditioned");
  }
  return calibration;
}

HardwareWrenchCalibration HardwareWrenchCalibration::loadFromFile(
    const std::string &path, const std::string &expected_vehicle_id,
    const std::string &current_date_utc, const HardwareCalibrationLimits &limits) {
  std::ifstream input(path);
  if (!input) {
    throw std::invalid_argument("hardware calibration file is unavailable");
  }
  std::ostringstream contents;
  contents << input.rdbuf();
  if (!input.good() && !input.eof()) {
    throw std::invalid_argument("failed to read hardware calibration file");
  }
  return loadFromText(contents.str(), expected_vehicle_id, current_date_utc, limits);
}

HardwareWrenchResult HardwareWrenchCalibration::normalize(
    double desired_collective_thrust_n, const Vec3 &desired_body_moment_frd_nm,
    double normalized_collective_thrust) const {
  HardwareWrenchResult result{};
  if (!std::isfinite(desired_collective_thrust_n) || desired_collective_thrust_n < 0.0 ||
      !desired_body_moment_frd_nm.finite() || !std::isfinite(normalized_collective_thrust)) {
    result.reason = "invalid physical wrench or normalized thrust";
    return result;
  }
  result.normalized_thrust_body_frd = {0.0, 0.0, -normalized_collective_thrust};
  if (normalized_collective_thrust < thrust_min_ || normalized_collective_thrust > thrust_max_) {
    result.status = HardwareWrenchStatus::outside_calibrated_range;
    result.reason = "normalized thrust is outside the measured range";
    return result;
  }

  Vec3 moment_without_bias = desired_body_moment_frd_nm;
  for (std::size_t row = 0; row < 3; ++row) {
    moment_without_bias[row] -=
        coefficients_[row + 1][0] + coefficients_[row + 1][1] * normalized_collective_thrust;
  }
  const Vec3 torque = inverse_moment_torque_block_ * moment_without_bias;
  if (!torque.finite() || torque.x < torque_min_.x || torque.x > torque_max_.x ||
      torque.y < torque_min_.y || torque.y > torque_max_.y || torque.z < torque_min_.z ||
      torque.z > torque_max_.z) {
    result.status = HardwareWrenchStatus::outside_calibrated_range;
    result.reason = "normalized torque is outside the measured range";
    return result;
  }

  const std::array<double, 5> input{1.0, normalized_collective_thrust, torque.x, torque.y,
                                    torque.z};
  std::array<double, 4> reconstructed{};
  for (std::size_t output = 0; output < 4; ++output) {
    for (std::size_t column = 0; column < input.size(); ++column) {
      reconstructed[output] += coefficients_[output][column] * input[column];
    }
  }
  result.normalized_torque_frd = torque;
  result.reconstructed_collective_thrust_n = reconstructed[0];
  result.reconstructed_body_moment_frd_nm =
      {reconstructed[1], reconstructed[2], reconstructed[3]};
  result.collective_force_residual_n = reconstructed[0] - desired_collective_thrust_n;
  const Vec3 reconstruction_error =
      result.reconstructed_body_moment_frd_nm - desired_body_moment_frd_nm;
  if (std::abs(result.collective_force_residual_n) >
          maximum_collective_reconstruction_error_n_ ||
      std::abs(reconstruction_error.x) > maximum_reconstruction_error_nm_.x ||
      std::abs(reconstruction_error.y) > maximum_reconstruction_error_nm_.y ||
      std::abs(reconstruction_error.z) > maximum_reconstruction_error_nm_.z) {
    result.status = HardwareWrenchStatus::reconstruction_failure;
    result.reason = "hardware affine inversion failed full-wrench reconstruction tolerance";
    return result;
  }
  result.ok = true;
  result.status = HardwareWrenchStatus::success;
  return result;
}

}  // namespace px4_offboard::calibration
