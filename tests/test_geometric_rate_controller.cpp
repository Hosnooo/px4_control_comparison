#include "control/geometric_rate_controller.hpp"
#include "test_support.hpp"

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

GeometricRateConfig testConfig() {
  GeometricRateConfig config{};
  config.k_attitude = {4.0, 5.0, 2.0};
  config.rate_limit_radps = {2.0, 2.0, 1.0};
  return config;
}

}  // namespace

int main() {
  bool threw = false;
  try {
    GeometricRateController unconfigured(GeometricRateConfig{});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "geometric-rate controller requires explicit configuration");

  const GeometricRateConfig config = testConfig();
  const GeometricRateController controller(config);
  const Mat3 identity = Mat3::identity();

  checkVecNear(controller.update(identity, identity, {}).body_rate_setpoint_frd_radps,
               {}, 1e-12, "zero error");

  Mat3 desired = Quat::fromAxisAngle({1.0, 0.0, 0.0}, 0.1).toMat3();
  const auto roll_output = controller.update(identity, desired, {});
  check(roll_output.attitude_error.x < 0.0,
        "Lee attitude error is negative for positive desired roll from current");
  check(roll_output.body_rate_setpoint_frd_radps.x > 0.0,
        "rate command drives positive desired roll");

  const Vec3 desired_rate{0.2, -0.1, 0.3};
  checkVecNear(controller.update(desired, desired, desired_rate).body_rate_setpoint_frd_radps,
               desired_rate, 1e-12, "feed-forward at zero error");

  desired = Quat::fromAxisAngle({0.0, 0.0, 1.0}, 1.2).toMat3();
  const auto limited = controller.update(identity, desired, {0.0, 0.0, 2.0});
  check(std::abs(limited.body_rate_setpoint_frd_radps.z) <=
            config.rate_limit_radps.z + 1e-12,
        "yaw rate limited");

  threw = false;
  try {
    GeometricRateConfig invalid_config = config;
    invalid_config.k_attitude.y = -0.1;
    GeometricRateController invalid(invalid_config);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "geometric-rate gains must be non-negative");
  return 0;
}
