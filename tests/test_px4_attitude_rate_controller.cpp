#include "control/px4_attitude_rate_controller.hpp"
#include "test_support.hpp"

using namespace control;

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

  const Px4AttitudeConfig ac = attitudeConfig();
  const Px4RateConfig rc = rateConfig();
  Px4AttitudeRateController px4(ac, rc);
  vecNear(px4.attitudeUpdate({1, 0, 0, 0}, {1, 0, 0, 0}, 0.0), {}, 1e-12,
          "identity attitude");
  const double yaw = 0.5;
  const auto yaw_rate =
      px4.attitudeUpdate({1, 0, 0, 0}, Quat::fromAxisAngle({0, 0, 1}, yaw), 0.0);
  const double expected = (2.8 / 0.4) * 2.0 * std::sin(0.4 * yaw / 2.0);
  checkNear(yaw_rate.z, expected, 1e-12, "PX4 yaw weighted quaternion law");
  auto limited=px4.attitudeUpdate({1,0,0,0},Quat::fromAxisAngle({1,0,0},2.0),0);
  check(std::abs(limited.x)<=ac.rate_limit_radps.x+1e-12,"attitude rate limit");

  const double half_sqrt_two = std::sqrt(0.5);
  const Quat body_z_world_x{half_sqrt_two, 0.0, half_sqrt_two, 0.0};
  const Quat body_z_world_minus_x{half_sqrt_two, 0.0, -half_sqrt_two, 0.0};
  const auto opposite_thrust =
      px4.attitudeUpdate(body_z_world_x, body_z_world_minus_x, 0.0);
  checkNear(opposite_thrust.x, 0.0, 1e-12, "PX4 opposite-thrust corner roll command");
  checkNear(opposite_thrust.y, -ac.rate_limit_radps.y, 1e-12,
            "PX4 opposite-thrust corner uses full desired attitude");
  checkNear(opposite_thrust.z, 0.0, 1e-12, "PX4 opposite-thrust corner yaw command");

  auto ff=px4.attitudeUpdate(Quat::fromAxisAngle({1,0,0},0.4),Quat::fromAxisAngle({1,0,0},0.4),0.7);
  Vec3 world_z_body=Quat::fromAxisAngle({1,0,0},0.4).inverse().dcmZ();
  vecNear(ff,world_z_body*0.7,1e-12,"yaw feedforward world z into body");

  RateControlInput in{};
  in.body_rate_frd_radps = {0.1, -0.2, 0.3};
  in.body_rate_setpoint_frd_radps = {0.5, 0.1, -0.1};
  in.body_angular_accel_frd_radps2 = {2.0, -3.0, 4.0};
  in.normalized_thrust_body_frd = {0.0, 0.0, -0.5};
  in.dt_s = 0.01;
  in.armed = true;
  in.landed_or_maybe_landed = false;
  RateControlInput invalid_timing = in;
  invalid_timing.dt_s = -0.01;
  threw = false;
  try {
    px4.rateUpdate(invalid_timing);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "negative rate-controller dt is rejected at the offboard mirror boundary");

  auto o1=px4.rateUpdate(in);
  Vec3 err=in.body_rate_setpoint_frd_radps-in.body_rate_frd_radps;
  Vec3 raw=hadamard(hadamard(rc.k,rc.p),err)-
           hadamard(hadamard(rc.k,rc.d),in.body_angular_accel_frd_radps2);
  vecNear(o1.normalized_torque_frd,raw,1e-12,"first rate PID output uses old zero integrator");

  Px4RateConfig scaled_rate = rc;
  scaled_rate.k = {2.0, 3.0, 4.0};
  Px4AttitudeRateController scaled_px4(ac, scaled_rate);
  const auto scaled_output = scaled_px4.rateUpdate(in);
  const Vec3 scaled_expected =
      hadamard(hadamard(scaled_rate.k, scaled_rate.p), err) -
      hadamard(hadamard(scaled_rate.k, scaled_rate.d), in.body_angular_accel_frd_radps2);
  vecNear(scaled_output.normalized_torque_frd, scaled_expected, 1e-12,
          "PX4 MC rate K scales P and D before RateControl");
  auto o2=px4.rateUpdate(in);
  check(o2.normalized_torque_frd.x>o1.normalized_torque_frd.x,"integrator contributes next cycle");

  px4.reset();
  in.saturation_positive={true,false,false};
  auto s1=px4.rateUpdate(in); auto s2=px4.rateUpdate(in);
  checkNear(s1.normalized_torque_frd.x, s2.normalized_torque_frd.x, 1e-12,
            "positive saturation blocks positive integral");

  px4.reset(); in.saturation_positive={false,false,false}; in.dt_s=1e-9;
  auto tiny=px4.rateUpdate(in); (void)tiny;
  checkNear(px4.lastDtS(),0.000125,1e-15,"PX4 lower dt clamp");
  in.dt_s=1.0; px4.rateUpdate(in); checkNear(px4.lastDtS(),0.02,1e-15,"PX4 upper dt clamp");

  Px4RateConfig filt=rc; filt.yaw_torque_cutoff_hz=2.0; Px4AttitudeRateController pf(ac,filt);
  RateControlInput yi{}; yi.body_rate_setpoint_frd_radps={0,0,1}; yi.dt_s=0.01; yi.armed=true;
  auto yf=pf.rateUpdate(yi);
  const double tau=1.0/(2.0*kPi*2.0), alpha=0.01/(tau+0.01);
  checkNear(yf.normalized_torque_frd.z,alpha*0.2,1e-12,"PX4 alpha yaw torque filter");

  Px4RateConfig bat=rc; bat.battery_scaling_enabled=true; Px4AttitudeRateController pb(ac,bat);
  in = {};
  in.body_rate_setpoint_frd_radps = {10.0, 0.0, 0.0};
  in.normalized_thrust_body_frd = {0.0, 0.0, -0.8};
  in.dt_s = 0.01;
  in.armed = true;
  in.battery_scale = 1.5;
  auto bo=pb.rateUpdate(in);
  checkNear(bo.normalized_torque_frd.x,1.0,1e-12,"battery torque clamp");
  checkNear(bo.normalized_thrust_body_frd.z,-1.0,1e-12,"battery thrust clamp");

  threw = false;
  try {
    Px4RateConfig bad_rate = rc;
    bad_rate.p.x = -0.1;
    Px4AttitudeRateController invalid(ac, bad_rate);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "PX4 mirror rejects negative gains");
  return 0;
}
