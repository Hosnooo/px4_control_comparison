#include "px4_offboard_controllers/controllers/geometric_acceleration.hpp"

#include <stdexcept>

namespace px4_offboard {

GeometricAccelerationController::GeometricAccelerationController(Vec3 position_gain,
                                                                 Vec3 velocity_gain)
    : position_gain_(position_gain), velocity_gain_(velocity_gain) {
  if (!position_gain_.finite() || !velocity_gain_.finite() || position_gain_.x < 0.0 ||
      position_gain_.y < 0.0 || position_gain_.z < 0.0 || velocity_gain_.x < 0.0 ||
      velocity_gain_.y < 0.0 || velocity_gain_.z < 0.0) {
    throw std::invalid_argument("invalid geometric-acceleration gains");
  }
}

AccelerationCommandNed GeometricAccelerationController::compute(
    const CanonicalState &state, const TrajectoryReference &reference) const {
  if (!validateState(state, requirementsFor(ControllerKind::GeometricAcceleration)) ||
      !validateReference(reference, ReferenceProfile::GeometricAcceleration)) {
    throw std::invalid_argument("missing geometric-acceleration inputs");
  }

  const Vec3 position_error = state.position_ned->value - *reference.position;
  const Vec3 velocity_error = state.velocity_ned->value - *reference.velocity;

  // Gravity is intentionally absent: PX4 owns gravity compensation below this setpoint boundary.
  return {*reference.acceleration - hadamard(position_gain_, position_error) -
          hadamard(velocity_gain_, velocity_error)};
}

}  // namespace px4_offboard
