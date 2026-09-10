#include "control/controller_handoff.hpp"
#include "test_support.hpp"

using namespace control;

int main() {
  HandoffTiming timing{};
  timing.publish_timestamp_us = 2'000'000;
  timing.sample_timestamp_us = 1'999'000;

  LeeOutput lee{};
  lee.desired_rotation_ned_frd = Mat3::identity();
  TrajectoryReference reference{};
  reference.yaw_rate_radps = 0.25;
  Px4ThrustOutput thrust{};
  thrust.normalized_thrust_body_frd = {0.0, 0.0, -0.55};

  const auto attitude = makeAttitudeHandoffInputs(lee, reference, thrust, true, timing);
  check(attitude.desired_attitude_q_ned_frd[0] == 1.0,
        "Lee desired rotation becomes PX4 quaternion wxyz");
  check(attitude.yaw_sp_move_rate_radps == 0.25,
        "trajectory yaw rate becomes PX4 attitude yaw feed-forward");
  check(attitude.normalized_thrust_body_frd[2] == -0.55,
        "common PX4 thrust normalization feeds attitude handoff");
  check(attitude.reset_integral.has_value() && *attitude.reset_integral,
        "attitude reset-integral choice is preserved explicitly");

  GeometricRateOutput geometric{};
  geometric.body_rate_setpoint_frd_radps = {0.1, -0.2, 0.3};
  const auto rate = makeRateHandoffInputs(geometric, thrust, false, timing);
  check(rate.reset_integral.has_value() && !*rate.reset_integral,
        "rate reset-integral choice is preserved explicitly");
  check(rate.body_rate_setpoint_frd_radps[0] == 0.1 &&
            rate.body_rate_setpoint_frd_radps[1] == -0.2 &&
            rate.body_rate_setpoint_frd_radps[2] == 0.3,
        "geometric FRD rate output feeds rate handoff");

  Px4RateOutput mirror{};
  mirror.normalized_torque_frd = {0.11, -0.12, 0.02};
  mirror.normalized_thrust_body_frd = {0.0, 0.0, -0.58};
  const auto px4 = makePx4MirrorHandoffInputs(mirror, timing);
  check(px4.normalized_torque_frd[0] == 0.11 &&
            px4.normalized_thrust_body_frd[2] == -0.58,
        "PX4 mirror normalized coordinates feed wrench boundary unchanged");

  F450WrenchResult simulation{};
  simulation.ok = true;
  simulation.normalized_torque_frd = {0.08, 0.02, -0.01};
  simulation.normalized_thrust_body_frd = {0.0, 0.0, -0.6};
  const auto lee_sim = makeLeeWrenchHandoffInputs(simulation, timing);
  check(lee_sim.physical_wrench_normalization_ok,
        "simulation physical-wrench success is preserved");
  check(lee_sim.normalized_torque_frd[0] == 0.08,
        "simulation normalized torque is preserved");

  HardwareWrenchResult hardware{};
  hardware.ok = false;
  const auto lee_hw = makeLeeWrenchHandoffInputs(hardware, timing);
  check(!lee_hw.physical_wrench_normalization_ok,
        "hardware physical-wrench failure is preserved for fail-closed serialization");

  HandoffValidationLimits limits{};
  limits.quaternion_norm_tolerance = 1e-6;
  check(buildPx4Handoff(HandoffMode::attitude_handoff, attitude, limits).ok,
        "adapted attitude command serializes");
  check(buildPx4Handoff(HandoffMode::rate_handoff, rate, limits).ok,
        "adapted rate command serializes");
  check(buildPx4Handoff(HandoffMode::px4_mirror, px4, limits).ok,
        "adapted PX4 mirror command serializes");
  check(!buildPx4Handoff(HandoffMode::lee_wrench, lee_hw, limits).ok,
        "failed hardware normalizer cannot serialize lee_wrench");
  return 0;
}
