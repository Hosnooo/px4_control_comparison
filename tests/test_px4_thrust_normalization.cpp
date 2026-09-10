#include "control/px4_thrust_normalization.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace control;

namespace {

void checkVecNear(const Vec3 &actual, const Vec3 &expected, double tolerance,
                  const std::string &message) {
  checkNear(actual.x, expected.x, tolerance, message + " x");
  checkNear(actual.y, expected.y, tolerance, message + " y");
  checkNear(actual.z, expected.z, tolerance, message + " z");
}

Px4ThrustConfig testConfig() {
  Px4ThrustConfig config{};
  config.hover_thrust = 0.5;
  config.gravity_mps2 = kPx4OneGmps2;
  config.tilt_limit_rad = 0.7;
  config.min_thrust = 0.1;
  config.max_thrust = 0.9;
  config.horizontal_thrust_margin = 0.3;
  config.decouple_horizontal_and_vertical_acceleration = false;
  return config;
}

}  // namespace

int main() {
  bool threw = false;
  try {
    Px4ThrustNormalization unconfigured(Px4ThrustConfig{});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "PX4 thrust normalization requires explicit configuration");

  const Px4ThrustConfig config = testConfig();
  const Px4ThrustNormalization normalizer(config);
  const double mass_kg = 2.0;
  const double gravity_mps2 = config.gravity_mps2;

  const auto hover = normalizer.fromPhysicalRotorForce(
      {0.0, 0.0, -mass_kg * gravity_mps2}, mass_kg);
  checkVecNear(hover.acceleration_setpoint_ned_mps2, {}, 1e-12, "hover acceleration");
  checkVecNear(hover.desired_body_z_ned, {0.0, 0.0, 1.0}, 1e-12, "hover body z");
  checkVecNear(hover.normalized_thrust_ned, {0.0, 0.0, -0.5}, 1e-12,
               "hover normalized thrust");
  checkNear(hover.normalized_collective_thrust_magnitude, 0.5, 1e-12,
            "hover collective magnitude");
  checkVecNear(hover.normalized_thrust_body_frd, {0.0, 0.0, -0.5}, 1e-12,
               "PX4 multicopter thrust is negative body Z in FRD");

  // +2 m/s^2 Down requires less upward thrust in NED.
  const auto downward_acceleration = normalizer.fromAccelerationSetpoint({0.0, 0.0, 2.0});
  const double expected_z = -0.5 + 2.0 * 0.5 / gravity_mps2;
  checkNear(downward_acceleration.normalized_thrust_ned.z, expected_z, 1e-12,
            "vertical PX4 formula");

  const auto lateral = normalizer.fromAccelerationSetpoint({3.0, 0.0, 0.0});
  check(lateral.desired_body_z_ned.x < 0.0,
        "positive north acceleration tilts body Z north-negative");
  check(lateral.normalized_thrust_ned.x > 0.0,
        "negative collective and negative body Z produce north-positive thrust coordinate");
  check(lateral.normalized_collective_thrust_magnitude >= config.min_thrust,
        "minimum thrust respected");
  check(lateral.normalized_thrust_ned.norm() <= config.max_thrust + 1e-12,
        "maximum thrust respected");

  // PX4 treats this nearly parallel body-Z case as parallel using FLT_EPSILON,
  // then selects +X as ControlMath::limitTilt()'s deterministic rejection axis.
  const auto near_parallel = normalizer.fromAccelerationSetpoint({0.0, 0.001, 0.0});
  check(near_parallel.desired_body_z_ned.x > 5e-5,
        "PX4 float-epsilon near-parallel fallback selects positive X");
  check(std::abs(near_parallel.desired_body_z_ned.y) < 1e-6,
        "near-parallel fallback suppresses the tiny original rejection direction");

  Px4ThrustConfig limited_config = config;
  limited_config.tilt_limit_rad = 0.2;
  const auto tilt_limited =
      Px4ThrustNormalization(limited_config).fromAccelerationSetpoint({20.0, 0.0, 0.0});
  const double tilt_rad = std::acos(std::clamp(tilt_limited.desired_body_z_ned.z, -1.0, 1.0));
  check(tilt_rad <= limited_config.tilt_limit_rad + 1e-12, "tilt limited");

  Px4ThrustConfig zero_min_config = config;
  zero_min_config.min_thrust = 0.0;
  const auto minimum_floor = Px4ThrustNormalization(zero_min_config)
                                 .fromAccelerationSetpoint({1.0, 0.0, gravity_mps2});
  checkNear(minimum_floor.normalized_collective_thrust_magnitude, 0.001, 1e-12,
            "PX4 setThrustLimits applies the 0.001 minimum thrust floor");

  const auto saturated = normalizer.fromAccelerationSetpoint({100.0, 100.0, -30.0});
  check(saturated.normalized_thrust_ned.norm() <= config.max_thrust + 1e-12,
        "combined thrust max");
  check(saturated.saturated, "saturation reported");
  checkVecNear(saturated.desired_body_z_ned, normalized(-saturated.normalized_thrust_ned),
               1e-12, "attitude direction follows PX4 final saturated thrust vector");

  threw = false;
  try {
    Px4ThrustConfig invalid_config = config;
    invalid_config.gravity_mps2 = 0.0;
    Px4ThrustNormalization invalid(invalid_config);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "zero gravity rejected");

  threw = false;
  try {
    Px4ThrustConfig non_px4_gravity = config;
    non_px4_gravity.gravity_mps2 = 9.81;
    Px4ThrustNormalization invalid(non_px4_gravity);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "PX4 mirror rejects a configurable replacement for CONSTANTS_ONE_G");

  threw = false;
  try {
    normalizer.fromPhysicalRotorForce({0.0, 0.0, -1.0}, 0.0);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "zero mass rejected");
  return 0;
}
