#include "px4_offboard_controllers/core/state.hpp"
#include "test_support.hpp"

#include <limits>

using namespace px4_offboard;

namespace {

template <class T>
TimedValue<T> sample(T value, double timestamp_s = 1.0) {
  return {value, timestamp_s};
}

}  // namespace

int main() {
  {
    const auto requirements = requirementsFor(ControllerKind::Px4Position);
    check(!requirements.position && !requirements.velocity && !requirements.attitude &&
              !requirements.body_rate && !requirements.body_angular_acceleration,
          "PX4 position requires no controller state");
    check(validateState({}, requirements), "PX4 position accepts empty controller state");
  }
  {
    const auto requirements = requirementsFor(ControllerKind::Px4Velocity);
    check(!requirements.position && !requirements.velocity && !requirements.attitude &&
              !requirements.body_rate && !requirements.body_angular_acceleration,
          "PX4 velocity requires no controller state");
  }
  {
    const auto requirements = requirementsFor(ControllerKind::GeometricAcceleration);
    check(requirements.position && requirements.velocity && !requirements.attitude &&
              !requirements.body_rate && !requirements.body_angular_acceleration,
          "geometric acceleration requires position and velocity only");
  }
  {
    const auto requirements = requirementsFor(ControllerKind::GeometricRate);
    check(requirements.position && requirements.velocity && requirements.attitude &&
              !requirements.body_rate && !requirements.body_angular_acceleration,
          "geometric rate requires position velocity and attitude");
  }
  {
    const auto requirements = requirementsFor(ControllerKind::LeeWrench);
    check(requirements.position && requirements.velocity && requirements.attitude &&
              requirements.body_rate && !requirements.body_angular_acceleration,
          "Lee requires position velocity attitude and body rate");
  }
  {
    const auto requirements = requirementsFor(ControllerKind::Px4AttitudeRateMirror);
    check(!requirements.position && !requirements.velocity && requirements.attitude &&
              requirements.body_rate && requirements.body_angular_acceleration,
          "PX4 mirror requires attitude rate and angular acceleration");
  }

  CanonicalState state{};
  state.position_ned = sample(Vec3{1.0, 2.0, 3.0}, 0.25);
  state.velocity_ned = sample(Vec3{0.1, 0.2, 0.3}, 0.30);
  state.attitude_ned_frd = sample(Quat{}, 0.35);
  state.body_rate_frd = sample(Vec3{0.4, 0.5, 0.6}, 0.40);
  state.body_angular_accel_frd = sample(Vec3{0.7, 0.8, 0.9}, 0.45);

  checkNear(state.position_ned->timestamp_s, 0.25, 0.0,
            "position timestamp remains source-owned");
  checkNear(state.body_angular_accel_frd->timestamp_s, 0.45, 0.0,
            "angular acceleration timestamp remains source-owned");
  check(validateState(state, requirementsFor(ControllerKind::LeeWrench)),
        "complete Lee state validates");
  check(validateState(state, requirementsFor(ControllerKind::Px4AttitudeRateMirror)),
        "complete mirror state validates");

  CanonicalState missing_rate = state;
  missing_rate.body_rate_frd.reset();
  check(!validateState(missing_rate, requirementsFor(ControllerKind::LeeWrench)),
        "required state is never substituted");

  CanonicalState bad_timestamp = state;
  bad_timestamp.attitude_ned_frd->timestamp_s =
      std::numeric_limits<double>::quiet_NaN();
  check(!validateState(bad_timestamp, requirementsFor(ControllerKind::GeometricRate)),
        "required state timestamp must be finite");

  CanonicalState bad_value = state;
  bad_value.velocity_ned->value.x = std::numeric_limits<double>::infinity();
  check(!validateState(bad_value, requirementsFor(ControllerKind::GeometricAcceleration)),
        "required state value must be finite");

  return 0;
}
