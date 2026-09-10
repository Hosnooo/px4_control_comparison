#pragma once

#include "control/math.hpp"

#include <limits>
#include <string>

namespace control {

template <typename T>
struct TimedValue {
  T value{};
  double timestamp_s{std::numeric_limits<double>::quiet_NaN()};
};

struct CanonicalState {
  TimedValue<Vec3> external_position_ned_m;
  TimedValue<Vec3> ekf_position_ned_m;
  TimedValue<Vec3> ekf_velocity_ned_mps;
  TimedValue<Quat> attitude_ned_frd;
  TimedValue<Vec3> body_rate_frd_radps;
  TimedValue<Vec3> body_angular_accel_frd_radps2;
};

struct TrajectoryReference {
  Vec3 position_ned_m{};
  Vec3 velocity_ned_mps{};
  Vec3 acceleration_ned_mps2{};
  Vec3 jerk_ned_mps3{};
  Vec3 snap_ned_mps4{};
  double yaw_rad{0.0};
  double yaw_rate_radps{0.0};
  double yaw_accel_radps2{0.0};
  double timestamp_s{std::numeric_limits<double>::quiet_NaN()};

  bool finite() const;
};

struct StateValidityLimits {
  double max_external_position_age_s{0.0};
  double max_ekf_velocity_age_s{0.0};
  double max_attitude_age_s{0.0};
  double max_body_rate_age_s{0.0};
  double max_angular_accel_age_s{0.0};
  double quaternion_norm_tolerance{0.0};
};

struct StateValidity {
  bool ok{false};
  std::string reason;
  double external_position_age_s{0.0};
  double ekf_velocity_age_s{0.0};
  double attitude_age_s{0.0};
  double body_rate_age_s{0.0};
  double angular_accel_age_s{0.0};
};

StateValidity validateState(const CanonicalState &state, double now_s,
                            const StateValidityLimits &limits,
                            bool require_angular_acceleration);

}  // namespace control
