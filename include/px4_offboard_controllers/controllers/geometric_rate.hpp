#pragma once

#include "px4_offboard_controllers/core/math.hpp"

namespace px4_offboard {

struct GeometricRateConfig {
  Vec3 k_attitude{};
  Vec3 rate_limit_radps{};
};

struct GeometricRateOutput {
  Vec3 attitude_error{};
  Vec3 body_rate_setpoint_frd_radps{};
};

/**
 * Lee-style SO(3) attitude error to FRD body-rate command.
 *
 * Inputs are current and desired NED<-FRD rotations plus desired FRD body rate. Output is rad/s
 * FRD with per-axis rate limiting. PX4 retains the inner body-rate loop and allocator.
 */
class GeometricRateController {
 public:
  explicit GeometricRateController(GeometricRateConfig config);

  GeometricRateOutput update(const Mat3 &current_rotation_ned_frd,
                             const Mat3 &desired_rotation_ned_frd,
                             const Vec3 &desired_body_rate_frd_radps) const;

 private:
  GeometricRateConfig config_;
};

}  // namespace px4_offboard
