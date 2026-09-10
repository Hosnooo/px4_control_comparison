#pragma once

#include "control/math.hpp"

namespace control {

struct GeometricRateConfig {
  Vec3 k_attitude{};
  Vec3 rate_limit_radps{};
};

struct GeometricRateOutput {
  Vec3 attitude_error{};
  Vec3 body_rate_setpoint_frd_radps{};
};

class GeometricRateController {
 public:
  explicit GeometricRateController(GeometricRateConfig config);
  GeometricRateOutput update(const Mat3 &current_rotation_ned_frd,
                             const Mat3 &desired_rotation_ned_frd,
                             const Vec3 &desired_body_rate_frd_radps) const;
 private:
  GeometricRateConfig config_;
};

}  // namespace control
