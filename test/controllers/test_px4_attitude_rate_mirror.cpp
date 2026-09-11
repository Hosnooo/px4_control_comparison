#include "px4_offboard_controllers/controllers/px4_attitude_rate_mirror.hpp"
#include "test_support.hpp"

#include <cmath>
#include <stdexcept>

using namespace px4_offboard;

namespace {

void vecNear(const Vec3 &actual, const Vec3 &expected, double tolerance,
             const std::string &message) {
  checkNear(actual.x, expected.x, tolerance, message + " x");
  checkNear(actual.y, expected.y, tolerance, message + " y");
  checkNear(actual.z, expected.z, tolerance, message + " z");
}

Px4AttitudeConfig attitudeConfig() {
  Px4AttitudeConfig config{};
  config.proportional_gain = {6.5, 6.5, 2.8};
  config.yaw_weight = 0.4;
  config.rate_limit_radps = {3.84, 3.84, 3.49};
  return config;
}

Px4RateConfig rateConfig() {
  Px4RateConfig config{};
  config.k = {1.0, 1.0, 1.0};
  config.p = {0.15, 0.15, 0.2};
  config.i = {0.2, 0.2, 0.1};
  config.d = {0.003, 0.003, 0.0};
  config.ff = {0.0, 0.0, 0.0};
  config.integrator_limit = {0.3, 0.3, 0.3};
  config.yaw_torque_cutoff_hz = 0.0;
  return config;
}

}  // namespace

int main() {
  bool threw = false;
  try {
    Px4AttitudeRateController unconfigured(Px4AttitudeConfig{}, Px4RateConfig{});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "PX4 mirror requires explicit configuration");

  const Px4AttitudeConfig attitude_config = attitudeConfig();
  const Px4RateConfig rate_config = rateConfig();
  Px4AttitudeRateController controller(attitude_config, rate_config);

  vecNear(controller.attitudeUpdate({1, 0, 0, 0}, {1, 0, 0, 0}, 0.0), {}, 1e-12,
          "identity attitude");
  const double yaw = 0.5;
  const auto yaw_rate = controller.attitudeUpdate(
      {1, 0, 0, 0}, Quat::fromAxisAngle({0, 0, 1}, yaw), 0.0);
  const double expected_yaw =
      (2.8 / 0.4) * 2.0 * std::sin(0.4 * yaw / 2.0);
  checkNear(yaw_rate.z, expected_yaw, 1e-12, "PX4 yaw weighted quaternion law");

  const auto limited = controller.attitudeUpdate(
      {1, 0, 0, 0}, Quat::fromAxisAngle({1, 0, 0}, 2.0), 0.0);
  check(std::abs(limited.x) <= attitude_config.rate_limit_radps.x + 1e-12,
        "attitude rate limit");

  const double half_sqrt_two = std::sqrt(0.5);
  const Quat body_z_world_x{half_sqrt_two, 0.0, half_sqrt_two, 0.0};
  const Quat body_z_world_minus_x{half_sqrt_two, 0.0, -half_sqrt_two, 0.0};
  const auto opposite_thrust =
      controller.attitudeUpdate(body_z_world_x, body_z_world_minus_x, 0.0);
  checkNear(opposite_thrust.x, 0.0, 1e-12, "PX4 opposite-thrust corner roll command");
  checkNear(opposite_thrust.y, -attitude_config.rate_limit_radps.y, 1e-12,
            "PX4 opposite-thrust corner uses full desired attitude");
  checkNear(opposite_thrust.z, 0.0, 1e-12, "PX4 opposite-thrust corner yaw command");

  const Quat rolled = Quat::fromAxisAngle({1, 0, 0}, 0.4);
  const auto feedforward = controller.attitudeUpdate(rolled, rolled, 0.7);
  const Vec3 world_z_body = rolled.inverse().dcmZ();
  vecNear(feedforward, world_z_body * 0.7, 1e-12,
          "yaw feedforward world z into body");

  RateControlInput input{};
  input.body_rate_frd_radps = {0.1, -0.2, 0.3};
  input.body_rate_setpoint_frd_radps = {0.5, 0.1, -0.1};
  input.body_angular_accel_frd_radps2 = {2.0, -3.0, 4.0};
  input.normalized_thrust_body_frd = {0.0, 0.0, -0.5};
  input.dt_s = 0.01;
  input.armed = true;

  RateControlInput invalid_timing = input;
  invalid_timing.dt_s = -0.01;
  threw = false;
  try {
    controller.rateUpdate(invalid_timing);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "negative rate-controller dt is rejected");

  const auto first = controller.rateUpdate(input);
  const Vec3 error = input.body_rate_setpoint_frd_radps - input.body_rate_frd_radps;
  const Vec3 raw = hadamard(hadamard(rate_config.k, rate_config.p), error) -
                   hadamard(hadamard(rate_config.k, rate_config.d),
                            input.body_angular_accel_frd_radps2);
  vecNear(first.normalized_torque_frd, raw, 1e-12,
          "first rate PID output uses old zero integrator");

  Px4RateConfig scaled_rate = rate_config;
  scaled_rate.k = {2.0, 3.0, 4.0};
  Px4AttitudeRateController scaled_controller(attitude_config, scaled_rate);
  const auto scaled_output = scaled_controller.rateUpdate(input);
  const Vec3 scaled_expected =
      hadamard(hadamard(scaled_rate.k, scaled_rate.p), error) -
      hadamard(hadamard(scaled_rate.k, scaled_rate.d),
               input.body_angular_accel_frd_radps2);
  vecNear(scaled_output.normalized_torque_frd, scaled_expected, 1e-12,
          "PX4 MC rate K scales P and D before RateControl");

  const auto second = controller.rateUpdate(input);
  check(second.normalized_torque_frd.x > first.normalized_torque_frd.x,
        "integrator contributes next cycle");

  controller.reset();
  input.saturation_positive = {true, false, false};
  const auto saturated_first = controller.rateUpdate(input);
  const auto saturated_second = controller.rateUpdate(input);
  checkNear(saturated_first.normalized_torque_frd.x,
            saturated_second.normalized_torque_frd.x, 1e-12,
            "positive saturation blocks positive integral");

  controller.reset();
  input.saturation_positive = {false, false, false};
  input.dt_s = 1e-9;
  (void)controller.rateUpdate(input);
  checkNear(controller.lastDtS(), 0.000125, 1e-15, "PX4 lower dt clamp");
  input.dt_s = 1.0;
  (void)controller.rateUpdate(input);
  checkNear(controller.lastDtS(), 0.02, 1e-15, "PX4 upper dt clamp");

  Px4RateConfig filtered_rate = rate_config;
  filtered_rate.yaw_torque_cutoff_hz = 2.0;
  Px4AttitudeRateController filtered_controller(attitude_config, filtered_rate);
  RateControlInput yaw_input{};
  yaw_input.body_rate_setpoint_frd_radps = {0.0, 0.0, 1.0};
  yaw_input.dt_s = 0.01;
  yaw_input.armed = true;
  const auto yaw_filtered = filtered_controller.rateUpdate(yaw_input);
  const double tau = 1.0 / (2.0 * kPi * 2.0);
  const double alpha = 0.01 / (tau + 0.01);
  checkNear(yaw_filtered.normalized_torque_frd.z, alpha * 0.2, 1e-12,
            "PX4 alpha yaw torque filter");

  Px4RateConfig battery_rate = rate_config;
  battery_rate.battery_scaling_enabled = true;
  Px4AttitudeRateController battery_controller(attitude_config, battery_rate);
  input = {};
  input.body_rate_setpoint_frd_radps = {10.0, 0.0, 0.0};
  input.normalized_thrust_body_frd = {0.0, 0.0, -0.8};
  input.dt_s = 0.01;
  input.armed = true;
  input.battery_scale = 1.5;
  const auto battery_output = battery_controller.rateUpdate(input);
  checkNear(battery_output.normalized_torque_frd.x, 1.0, 1e-12,
            "battery torque clamp");
  checkNear(battery_output.normalized_thrust_body_frd.z, -1.0, 1e-12,
            "battery thrust clamp");

  threw = false;
  try {
    Px4RateConfig bad_rate = rate_config;
    bad_rate.p.x = -0.1;
    Px4AttitudeRateController invalid(attitude_config, bad_rate);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "PX4 mirror rejects negative gains");
  return 0;
}
