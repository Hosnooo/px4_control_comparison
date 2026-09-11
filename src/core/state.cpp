#include "px4_offboard_controllers/core/state.hpp"

#include <cmath>

namespace px4_offboard {
namespace {

bool validTimestamp(double timestamp_s) {
  return std::isfinite(timestamp_s) && timestamp_s >= 0.0;
}

bool validValue(const Vec3 &value) {
  return value.finite();
}

bool validValue(const Quat &value) {
  return value.finite() && value.squaredNorm() > kEps;
}

template <class T>
bool validRequiredField(const std::optional<TimedValue<T>> &field, bool required) {
  return !required ||
         (field.has_value() && validTimestamp(field->timestamp_s) && validValue(field->value));
}

}  // namespace

StateRequirements requirementsFor(ControllerKind kind) {
  switch (kind) {
    case ControllerKind::Px4Position:
    case ControllerKind::Px4Velocity:
      return {};
    case ControllerKind::GeometricAcceleration:
      return {true, true, false, false, false};
    case ControllerKind::GeometricAttitude:
    case ControllerKind::GeometricRate:
      return {true, true, true, false, false};
    case ControllerKind::Px4AttitudeRateMirror:
      return {false, false, true, true, true};
    case ControllerKind::LeeWrench:
      return {true, true, true, true, false};
  }
  return {};
}

bool validateState(const CanonicalState &state, StateRequirements requirements) {
  return validRequiredField(state.position_ned, requirements.position) &&
         validRequiredField(state.velocity_ned, requirements.velocity) &&
         validRequiredField(state.attitude_ned_frd, requirements.attitude) &&
         validRequiredField(state.body_rate_frd, requirements.body_rate) &&
         validRequiredField(state.body_angular_accel_frd,
                            requirements.body_angular_acceleration);
}

}  // namespace px4_offboard
