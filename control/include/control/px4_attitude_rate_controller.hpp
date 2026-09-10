#pragma once

#include "control/math.hpp"

#include <array>
#include <limits>

namespace control {

struct Px4AttitudeConfig {
  Vec3 proportional_gain{};
  double yaw_weight{0.0};
  Vec3 rate_limit_radps{};
};

struct Px4RateConfig {
  Vec3 k{};
  Vec3 p{};
  Vec3 i{};
  Vec3 d{};
  Vec3 ff{};
  Vec3 integrator_limit{};
  double yaw_torque_cutoff_hz{0.0};
  bool battery_scaling_enabled{false};
};

struct RateControlInput {
  Vec3 body_rate_frd_radps{};
  Vec3 body_rate_setpoint_frd_radps{};
  Vec3 body_angular_accel_frd_radps2{};
  Vec3 normalized_thrust_body_frd{};
  double dt_s{std::numeric_limits<double>::quiet_NaN()};
  bool armed{false};
  bool landed_or_maybe_landed{false};
  std::array<bool, 3> saturation_positive{false, false, false};
  std::array<bool, 3> saturation_negative{false, false, false};
  double battery_scale{0.0};
};

struct Px4RateOutput {
  Vec3 normalized_torque_frd{};
  Vec3 normalized_thrust_body_frd{};
  Vec3 integrator{};
};

class Px4AttitudeRateController {
 public:
  Px4AttitudeRateController(Px4AttitudeConfig attitude, Px4RateConfig rate);
  Vec3 attitudeUpdate(const Quat &current_ned_frd, const Quat &desired_ned_frd,
                      double yaw_speed_setpoint_radps) const;
  Px4RateOutput rateUpdate(const RateControlInput &input);
  void reset();
  double lastDtS() const { return last_dt_s_; }
  Vec3 integrator() const { return rate_integral_; }

 private:
  Px4AttitudeConfig attitude_;
  Px4RateConfig rate_;
  Vec3 attitude_gain_{};
  Vec3 effective_rate_p_{};
  Vec3 effective_rate_i_{};
  Vec3 effective_rate_d_{};
  Vec3 rate_integral_{};
  double yaw_filter_state_{0.0};
  double last_dt_s_{0.0};
};

}  // namespace control
