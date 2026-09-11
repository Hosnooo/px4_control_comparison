#pragma once

#include "px4_offboard_controllers/core/state.hpp"
#include "px4_offboard_controllers/core/trajectory.hpp"

namespace px4_offboard {

struct AccelerationCommandNed {
  Vec3 acceleration_ned;
};

/**
 * Position/velocity feedback in canonical world NED.
 *
 * Requires position and velocity state and desired position, velocity and acceleration. Output is
 * an acceleration setpoint in m/s^2 NED. PX4 owns gravity compensation, thrust normalization and
 * every lower loop below the acceleration-setpoint boundary.
 */
class GeometricAccelerationController {
 public:
  GeometricAccelerationController(Vec3 position_gain, Vec3 velocity_gain);

  AccelerationCommandNed compute(const CanonicalState &state,
                                 const TrajectoryReference &reference) const;

 private:
  Vec3 position_gain_{};
  Vec3 velocity_gain_{};
};

}  // namespace px4_offboard
