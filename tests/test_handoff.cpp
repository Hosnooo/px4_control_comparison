#include "control/handoff.hpp"
#include "test_support.hpp"

#include <cstdint>
#include <type_traits>

using namespace control;

namespace {

HandoffInputs validInputs() {
  HandoffInputs input{};
  input.timing.publish_timestamp_us = 2'000'000;
  input.timing.sample_timestamp_us = 1'999'000;
  input.desired_attitude_q_ned_frd = {0.9238795325, 0.0, 0.3826834324, 0.0};
  input.yaw_sp_move_rate_radps = 0.3;
  input.body_rate_setpoint_frd_radps = {0.2, -0.1, 0.05};
  input.normalized_torque_frd = {0.1, -0.2, 0.03};
  input.normalized_thrust_body_frd = {0.0, 0.0, -0.6};
  input.reset_integral = false;
  input.physical_wrench_normalization_ok = true;
  return input;
}

void checkOnlyAttitude(const OffboardControlModeCommand &mode) {
  check(!mode.position && !mode.velocity && !mode.acceleration && mode.attitude &&
            !mode.body_rate && !mode.thrust_and_torque && !mode.direct_actuator,
        "attitude mode flags are one-hot at the attitude boundary");
}

void checkOnlyBodyRate(const OffboardControlModeCommand &mode) {
  check(!mode.position && !mode.velocity && !mode.acceleration && !mode.attitude &&
            mode.body_rate && !mode.thrust_and_torque && !mode.direct_actuator,
        "rate mode flags are one-hot at the body-rate boundary");
}

void checkOnlyWrench(const OffboardControlModeCommand &mode) {
  check(!mode.position && !mode.velocity && !mode.acceleration && !mode.attitude &&
            !mode.body_rate && mode.thrust_and_torque && !mode.direct_actuator,
        "wrench mode flags are one-hot at the thrust-and-torque boundary");
}

void checkInactive(const OffboardControlModeCommand &mode) {
  check(!mode.position && !mode.velocity && !mode.acceleration && !mode.attitude &&
            !mode.body_rate && !mode.thrust_and_torque && !mode.direct_actuator,
        "rejected handoff carries no active offboard boundary");
}

}  // namespace

int main() {
  static_assert(std::is_same_v<decltype(VehicleAttitudeSetpointCommand{}.q_d)::value_type,
                               float>);
  static_assert(std::is_same_v<decltype(VehicleTorqueSetpointCommand{}.timestamp),
                               std::uint64_t>);
  HandoffValidationLimits limits{};
  limits.quaternion_norm_tolerance = 1e-6;
  const HandoffInputs input = validInputs();

  const auto attitude = buildPx4Handoff(HandoffMode::attitude_handoff, input, limits);
  check(attitude.ok, attitude.reason);
  checkOnlyAttitude(attitude.commands.offboard_control_mode);
  check(attitude.commands.attitude_setpoint.has_value(), "attitude setpoint published");
  check(!attitude.commands.rates_setpoint && !attitude.commands.torque_setpoint &&
            !attitude.commands.thrust_setpoint,
        "attitude mode publishes no bypassed-loop setpoints");
  const auto &attitude_sp = *attitude.commands.attitude_setpoint;
  check(attitude_sp.timestamp == *input.timing.publish_timestamp_us,
        "attitude timestamp propagated exactly");
  checkNear(attitude_sp.q_d[0], static_cast<float>(input.desired_attitude_q_ned_frd[0]), 0.0,
            "attitude quaternion w ordering");
  checkNear(attitude_sp.q_d[2], static_cast<float>(input.desired_attitude_q_ned_frd[2]), 0.0,
            "attitude quaternion y ordering");
  checkNear(attitude_sp.thrust_body[2], static_cast<float>(-0.6), 0.0, "attitude FRD thrust sign");
  checkNear(attitude_sp.yaw_sp_move_rate, static_cast<float>(0.3), 0.0, "yaw move rate units");
  check(!attitude_sp.fw_control_yaw_wheel, "multicopter never enables FW yaw wheel");

  const auto rate = buildPx4Handoff(HandoffMode::rate_handoff, input, limits);
  check(rate.ok, rate.reason);
  checkOnlyBodyRate(rate.commands.offboard_control_mode);
  check(rate.commands.rates_setpoint.has_value(), "rate setpoint published");
  check(!rate.commands.attitude_setpoint && !rate.commands.torque_setpoint &&
            !rate.commands.thrust_setpoint,
        "rate mode publishes no bypassed-loop setpoints");
  const auto &rate_sp = *rate.commands.rates_setpoint;
  checkNear(rate_sp.roll, static_cast<float>(0.2), 0.0, "FRD roll rate maps to x");
  checkNear(rate_sp.pitch, static_cast<float>(-0.1), 0.0, "FRD pitch rate maps to y");
  checkNear(rate_sp.yaw, static_cast<float>(0.05), 0.0, "FRD yaw rate maps to z");
  checkNear(rate_sp.thrust_body[2], static_cast<float>(-0.6), 0.0, "rate FRD thrust sign");

  for (const HandoffMode mode : {HandoffMode::px4_mirror, HandoffMode::lee_wrench}) {
    HandoffInputs wrench_input = input;
    if (mode == HandoffMode::px4_mirror) {
      wrench_input.physical_wrench_normalization_ok = false;
    }
    const auto wrench = buildPx4Handoff(mode, wrench_input, limits);
    check(wrench.ok, wrench.reason);
    checkOnlyWrench(wrench.commands.offboard_control_mode);
    check(!wrench.commands.attitude_setpoint && !wrench.commands.rates_setpoint,
          "wrench modes bypass PX4 attitude and rate loops");
    check(wrench.commands.torque_setpoint.has_value() &&
              wrench.commands.thrust_setpoint.has_value(),
          "wrench modes publish normalized torque and thrust together");
    const auto &torque = *wrench.commands.torque_setpoint;
    const auto &thrust = *wrench.commands.thrust_setpoint;
    check(torque.timestamp == *input.timing.publish_timestamp_us &&
              thrust.timestamp == *input.timing.publish_timestamp_us,
          "wrench publish timestamp propagated");
    check(torque.timestamp_sample == *input.timing.sample_timestamp_us &&
              thrust.timestamp_sample == *input.timing.sample_timestamp_us,
          "wrench sample timestamp propagated");
    checkNear(torque.xyz[0], static_cast<float>(0.1), 0.0, "normalized torque x");
    checkNear(torque.xyz[1], static_cast<float>(-0.2), 0.0, "normalized torque y");
    checkNear(torque.xyz[2], static_cast<float>(0.03), 0.0, "normalized torque z");
    checkNear(thrust.xyz[2], static_cast<float>(-0.6), 0.0, "normalized thrust z");
  }

  HandoffInputs invalid = input;
  invalid.normalized_thrust_body_frd = {0.01, 0.0, -0.6};
  const auto lateral_thrust =
      buildPx4Handoff(HandoffMode::attitude_handoff, invalid, limits);
  check(!lateral_thrust.ok, "multicopter lateral body thrust is rejected");
  checkInactive(lateral_thrust.commands.offboard_control_mode);
  invalid = input;
  invalid.normalized_thrust_body_frd[2] = 0.1;
  check(!buildPx4Handoff(HandoffMode::rate_handoff, invalid, limits).ok,
        "positive FRD body-z thrust is rejected");
  invalid = input;
  invalid.normalized_torque_frd[0] = 1.01;
  check(!buildPx4Handoff(HandoffMode::px4_mirror, invalid, limits).ok,
        "out-of-range normalized torque is rejected");
  invalid = input;
  invalid.timing.sample_timestamp_us = *invalid.timing.publish_timestamp_us + 1;
  check(!buildPx4Handoff(HandoffMode::lee_wrench, invalid, limits).ok,
        "future sample timestamp is rejected");
  invalid = input;
  invalid.physical_wrench_normalization_ok = false;
  const auto failed_physical =
      buildPx4Handoff(HandoffMode::lee_wrench, invalid, limits);
  check(!failed_physical.ok,
        "failed physical-wrench normalization cannot publish lee_wrench");
  checkInactive(failed_physical.commands.offboard_control_mode);
  invalid = input;
  invalid.timing.publish_timestamp_us.reset();
  check(!buildPx4Handoff(HandoffMode::rate_handoff, invalid, limits).ok,
        "publish timestamp must be explicit");
  invalid = input;
  invalid.timing.sample_timestamp_us.reset();
  check(!buildPx4Handoff(HandoffMode::px4_mirror, invalid, limits).ok,
        "wrench sample timestamp must be explicit");
  invalid = input;
  invalid.reset_integral.reset();
  check(!buildPx4Handoff(HandoffMode::attitude_handoff, invalid, limits).ok,
        "attitude reset-integral choice must be explicit");
  check(!buildPx4Handoff(HandoffMode::rate_handoff, invalid, limits).ok,
        "rate reset-integral choice must be explicit");
  invalid = input;
  invalid.body_rate_setpoint_frd_radps[0] = 1e300;
  check(!buildPx4Handoff(HandoffMode::rate_handoff, invalid, limits).ok,
        "finite doubles that overflow PX4 float32 are rejected");
  invalid = input;
  invalid.desired_attitude_q_ned_frd = {2.0, 0.0, 0.0, 0.0};
  check(!buildPx4Handoff(HandoffMode::attitude_handoff, invalid, limits).ok,
        "non-unit attitude quaternion is rejected rather than normalized silently");

  HandoffValidationLimits invalid_limits{};
  check(!buildPx4Handoff(HandoffMode::attitude_handoff, input, invalid_limits).ok,
        "validation tolerance must be explicit for attitude serialization");
  check(buildPx4Handoff(HandoffMode::rate_handoff, input, invalid_limits).ok,
        "unrelated quaternion tolerance does not gate rate serialization");

  HandoffInputs defaults{};
  check(!buildPx4Handoff(HandoffMode::attitude_handoff, defaults, limits).ok,
        "default aggregate cannot become an executable command");
  return 0;
}
