#include "px4_offboard_controllers/core/state.hpp"

namespace px4_offboard {

StateRequirements requirementsFor(ControllerKind kind) {
  switch (kind) {
    case ControllerKind::Px4Position:
    case ControllerKind::Px4Velocity:
      return {};
    case ControllerKind::GeometricAcceleration:
      return {true, true, false, false};
    case ControllerKind::GeometricAttitude:
    case ControllerKind::GeometricRate:
      return {true, true, true, false};
    case ControllerKind::Px4AttitudeRateMirror:
      return {false, false, true, true};
    case ControllerKind::LeeWrench:
      return {true, true, true, true};
  }
  return {};
}

bool validateState(const CanonicalState &state, StateRequirements requirements) {
  return (!requirements.position || state.position_ned.has_value()) &&
         (!requirements.velocity || state.velocity_ned.has_value()) &&
         (!requirements.attitude || state.attitude_ned_frd.has_value()) &&
         (!requirements.body_rate || state.body_rate_frd.has_value());
}

}  // namespace px4_offboard
