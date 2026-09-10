#include "control/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control {
namespace {

struct ScalarDerivatives {
  double position;
  double velocity;
  double acceleration;
  double jerk;
  double snap;
};

ScalarDerivatives quinticScalar(double normalized_time, double duration_s) {
  const double u = std::clamp(normalized_time, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double duration2 = duration_s * duration_s;
  const double duration3 = duration2 * duration_s;
  const double duration4 = duration3 * duration_s;

  return {
      10.0 * u3 - 15.0 * u4 + 6.0 * u5,
      (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration_s,
      (60.0 * u - 180.0 * u2 + 120.0 * u3) / duration2,
      (60.0 - 360.0 * u + 360.0 * u2) / duration3,
      (-360.0 + 720.0 * u) / duration4,
  };
}

}  // namespace

TrajectoryReference hoverReference(const Vec3 &position_ned_m, double yaw_rad,
                                   double timestamp_s) {
  if (!position_ned_m.finite() || !std::isfinite(yaw_rad) || !std::isfinite(timestamp_s)) {
    throw std::invalid_argument("invalid hover trajectory input");
  }

  TrajectoryReference reference{};
  reference.position_ned_m = position_ned_m;
  reference.yaw_rad = yaw_rad;
  reference.timestamp_s = timestamp_s;
  return reference;
}

TrajectoryReference quinticReference(const Vec3 &start_ned_m, const Vec3 &end_ned_m,
                                     double start_yaw_rad, double end_yaw_rad,
                                     double duration_s, double elapsed_s,
                                     double timestamp_s) {
  if (!start_ned_m.finite() || !end_ned_m.finite() || !std::isfinite(start_yaw_rad) ||
      !std::isfinite(end_yaw_rad) || !(duration_s > 0.0) || !std::isfinite(duration_s) ||
      !std::isfinite(elapsed_s) || !std::isfinite(timestamp_s)) {
    throw std::invalid_argument("invalid quintic trajectory input");
  }

  const double clamped_elapsed_s = std::clamp(elapsed_s, 0.0, duration_s);
  ScalarDerivatives scalar = quinticScalar(clamped_elapsed_s / duration_s, duration_s);

  if (elapsed_s <= 0.0 || elapsed_s >= duration_s) {
    scalar.velocity = 0.0;
    scalar.acceleration = 0.0;
    scalar.jerk = 0.0;
    scalar.snap = 0.0;
  }

  const Vec3 displacement_ned_m = end_ned_m - start_ned_m;
  TrajectoryReference reference{};
  reference.position_ned_m = start_ned_m + displacement_ned_m * scalar.position;
  reference.velocity_ned_mps = displacement_ned_m * scalar.velocity;
  reference.acceleration_ned_mps2 = displacement_ned_m * scalar.acceleration;
  reference.jerk_ned_mps3 = displacement_ned_m * scalar.jerk;
  reference.snap_ned_mps4 = displacement_ned_m * scalar.snap;

  const double yaw_displacement_rad = end_yaw_rad - start_yaw_rad;
  reference.yaw_rad = start_yaw_rad + yaw_displacement_rad * scalar.position;
  reference.yaw_rate_radps = yaw_displacement_rad * scalar.velocity;
  reference.yaw_accel_radps2 = yaw_displacement_rad * scalar.acceleration;
  reference.timestamp_s = timestamp_s;
  return reference;
}

TrajectoryReference circleReference(const Vec3 &center_ned_m, double radius_m,
                                    double angular_rate_radps, double yaw_rad,
                                    double elapsed_s, double timestamp_s) {
  if (!center_ned_m.finite() || !(radius_m >= 0.0) || !std::isfinite(radius_m) ||
      !std::isfinite(angular_rate_radps) || !std::isfinite(yaw_rad) ||
      !std::isfinite(elapsed_s) || !std::isfinite(timestamp_s)) {
    throw std::invalid_argument("invalid circle trajectory input");
  }

  const double phase_rad = angular_rate_radps * elapsed_s;
  const double cosine = std::cos(phase_rad);
  const double sine = std::sin(phase_rad);
  const double rate2 = angular_rate_radps * angular_rate_radps;
  const double rate3 = rate2 * angular_rate_radps;
  const double rate4 = rate3 * angular_rate_radps;

  TrajectoryReference reference{};
  reference.position_ned_m =
      center_ned_m + Vec3{radius_m * cosine, radius_m * sine, 0.0};
  reference.velocity_ned_mps =
      {-radius_m * angular_rate_radps * sine, radius_m * angular_rate_radps * cosine, 0.0};
  reference.acceleration_ned_mps2 =
      {-radius_m * rate2 * cosine, -radius_m * rate2 * sine, 0.0};
  reference.jerk_ned_mps3 = {radius_m * rate3 * sine, -radius_m * rate3 * cosine, 0.0};
  reference.snap_ned_mps4 = {radius_m * rate4 * cosine, radius_m * rate4 * sine, 0.0};
  reference.yaw_rad = yaw_rad;
  reference.timestamp_s = timestamp_s;
  return reference;
}

TrajectoryReference figureEightReference(const Vec3 &center_ned_m, double x_amplitude_m,
                                         double y_amplitude_m, double angular_rate_radps,
                                         double yaw_rad, double elapsed_s,
                                         double timestamp_s) {
  if (!center_ned_m.finite() || !std::isfinite(x_amplitude_m) ||
      !std::isfinite(y_amplitude_m) || !std::isfinite(angular_rate_radps) ||
      !std::isfinite(yaw_rad) || !std::isfinite(elapsed_s) ||
      !std::isfinite(timestamp_s)) {
    throw std::invalid_argument("invalid figure-eight trajectory input");
  }

  const double phase_rad = angular_rate_radps * elapsed_s;
  const double double_phase_rad = 2.0 * phase_rad;
  const double rate2 = angular_rate_radps * angular_rate_radps;
  const double rate3 = rate2 * angular_rate_radps;
  const double rate4 = rate3 * angular_rate_radps;

  TrajectoryReference reference{};
  reference.position_ned_m =
      center_ned_m + Vec3{x_amplitude_m * std::sin(phase_rad),
                         y_amplitude_m * std::sin(double_phase_rad), 0.0};
  reference.velocity_ned_mps =
      {x_amplitude_m * angular_rate_radps * std::cos(phase_rad),
       2.0 * y_amplitude_m * angular_rate_radps * std::cos(double_phase_rad), 0.0};
  reference.acceleration_ned_mps2 =
      {-x_amplitude_m * rate2 * std::sin(phase_rad),
       -4.0 * y_amplitude_m * rate2 * std::sin(double_phase_rad), 0.0};
  reference.jerk_ned_mps3 =
      {-x_amplitude_m * rate3 * std::cos(phase_rad),
       -8.0 * y_amplitude_m * rate3 * std::cos(double_phase_rad), 0.0};
  reference.snap_ned_mps4 =
      {x_amplitude_m * rate4 * std::sin(phase_rad),
       16.0 * y_amplitude_m * rate4 * std::sin(double_phase_rad), 0.0};
  reference.yaw_rad = yaw_rad;
  reference.timestamp_s = timestamp_s;
  return reference;
}

}  // namespace control
