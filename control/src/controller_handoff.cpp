#include "control/controller_handoff.hpp"

namespace control {
namespace {

std::array<double, 3> asArray(const Vec3 &vector) {
  return {vector.x, vector.y, vector.z};
}

std::array<double, 4> asArray(const Quat &quaternion) {
  return {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
}

}  // namespace

HandoffInputs makeAttitudeHandoffInputs(const LeeOutput &lee,
                                        const TrajectoryReference &reference,
                                        const Px4ThrustOutput &thrust,
                                        bool reset_integral,
                                        const HandoffTiming &timing) {
  HandoffInputs input{};
  input.timing = timing;
  input.desired_attitude_q_ned_frd =
      asArray(Quat::fromMat3(lee.desired_rotation_ned_frd));
  input.yaw_sp_move_rate_radps = reference.yaw_rate_radps;
  input.normalized_thrust_body_frd = asArray(thrust.normalized_thrust_body_frd);
  input.reset_integral = reset_integral;
  return input;
}

HandoffInputs makeRateHandoffInputs(const GeometricRateOutput &rate,
                                    const Px4ThrustOutput &thrust,
                                    bool reset_integral,
                                    const HandoffTiming &timing) {
  HandoffInputs input{};
  input.timing = timing;
  input.body_rate_setpoint_frd_radps = asArray(rate.body_rate_setpoint_frd_radps);
  input.normalized_thrust_body_frd = asArray(thrust.normalized_thrust_body_frd);
  input.reset_integral = reset_integral;
  return input;
}

HandoffInputs makePx4MirrorHandoffInputs(const Px4RateOutput &mirror,
                                         const HandoffTiming &timing) {
  HandoffInputs input{};
  input.timing = timing;
  input.normalized_torque_frd = asArray(mirror.normalized_torque_frd);
  input.normalized_thrust_body_frd = asArray(mirror.normalized_thrust_body_frd);
  return input;
}

HandoffInputs makeLeeWrenchHandoffInputs(const F450WrenchResult &wrench,
                                         const HandoffTiming &timing) {
  HandoffInputs input{};
  input.timing = timing;
  input.normalized_torque_frd = asArray(wrench.normalized_torque_frd);
  input.normalized_thrust_body_frd = asArray(wrench.normalized_thrust_body_frd);
  input.physical_wrench_normalization_ok = wrench.ok;
  return input;
}

HandoffInputs makeLeeWrenchHandoffInputs(const HardwareWrenchResult &wrench,
                                         const HandoffTiming &timing) {
  HandoffInputs input{};
  input.timing = timing;
  input.normalized_torque_frd = asArray(wrench.normalized_torque_frd);
  input.normalized_thrust_body_frd = asArray(wrench.normalized_thrust_body_frd);
  input.physical_wrench_normalization_ok = wrench.ok;
  return input;
}

}  // namespace control
