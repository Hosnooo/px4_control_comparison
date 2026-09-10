#include "control/state.hpp"
#include "control/trajectory.hpp"
#include "test_support.hpp"

using namespace control;

static void vecNear(const Vec3 &a, const Vec3 &b, double tol, const char *msg) {
  checkNear(a.x, b.x, tol, std::string(msg) + " x");
  checkNear(a.y, b.y, tol, std::string(msg) + " y");
  checkNear(a.z, b.z, tol, std::string(msg) + " z");
}

int main() {
  StateValidityLimits startup_limits{};
  startup_limits.max_external_position_age_s = 0.05;
  startup_limits.max_ekf_velocity_age_s = 0.05;
  startup_limits.max_attitude_age_s = 0.05;
  startup_limits.max_body_rate_age_s = 0.05;
  startup_limits.max_angular_accel_age_s = 0.05;
  startup_limits.quaternion_norm_tolerance = 1e-3;

  check(!TrajectoryReference{}.finite(),
        "default trajectory reference must be invalid until timestamped");
  check(!validateState(CanonicalState{}, 0.0, startup_limits, false).ok,
        "default canonical state must be invalid until measurements arrive");

  bool threw = false;
  try {
    hoverReference({1.0, -2.0, -1.5}, 0.4,
                   std::numeric_limits<double>::quiet_NaN());
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "trajectory generators reject invalid timestamps at their boundary");

  const auto hover = hoverReference({1.0, -2.0, -1.5}, 0.4, 3.0);
  vecNear(hover.position_ned_m, {1.0, -2.0, -1.5}, 1e-12, "hover position");
  vecNear(hover.velocity_ned_mps, {}, 1e-12, "hover velocity");
  vecNear(hover.snap_ned_mps4, {}, 1e-12, "hover snap");
  checkNear(hover.yaw_rad, 0.4, 1e-12, "hover yaw");

  const Vec3 p0{0.0, 0.0, 0.0}, p1{2.0, -1.0, 0.5};
  const auto q0 = quinticReference(p0, p1, 0.2, 0.8, 2.0, 0.0, 100.0);
  const auto q1 = quinticReference(p0, p1, 0.2, 0.8, 2.0, 2.0, 102.0);
  vecNear(q0.position_ned_m, p0, 1e-12, "quintic start position");
  vecNear(q0.velocity_ned_mps, {}, 1e-12, "quintic start velocity");
  vecNear(q0.acceleration_ned_mps2, {}, 1e-12, "quintic start acceleration");
  vecNear(q1.position_ned_m, p1, 1e-12, "quintic end position");
  vecNear(q1.velocity_ned_mps, {}, 1e-12, "quintic end velocity");
  vecNear(q1.acceleration_ned_mps2, {}, 1e-12, "quintic end acceleration");
  checkNear(q0.timestamp_s, 100.0, 1e-12, "quintic reference timestamp");
  checkNear(q1.timestamp_s, 102.0, 1e-12, "quintic reference timestamp after motion");
  const auto q_after = quinticReference(p0, p1, 0.2, 0.8, 2.0, 3.0, 103.0);
  vecNear(q_after.velocity_ned_mps, {}, 1e-12, "quintic hold velocity after motion");
  vecNear(q_after.acceleration_ned_mps2, {}, 1e-12,
          "quintic hold acceleration after motion");
  vecNear(q_after.jerk_ned_mps3, {}, 1e-12, "quintic hold jerk after motion");
  vecNear(q_after.snap_ned_mps4, {}, 1e-12, "quintic hold snap after motion");

  const double t = 0.37, h = 1e-5;
  const auto qm = quinticReference(p0, p1, 0.2, 0.8, 2.0, t - h, 200.0 - h);
  const auto qc = quinticReference(p0, p1, 0.2, 0.8, 2.0, t, 200.0);
  const auto qp = quinticReference(p0, p1, 0.2, 0.8, 2.0, t + h, 200.0 + h);
  vecNear((qp.position_ned_m - qm.position_ned_m) / (2.0 * h), qc.velocity_ned_mps,
          2e-8, "quintic analytic velocity");
  vecNear((qp.velocity_ned_mps - qm.velocity_ned_mps) / (2.0 * h),
          qc.acceleration_ned_mps2, 2e-7, "quintic analytic acceleration");
  vecNear((qp.acceleration_ned_mps2 - qm.acceleration_ned_mps2) / (2.0 * h),
          qc.jerk_ned_mps3, 2e-6, "quintic analytic jerk");
  vecNear((qp.jerk_ned_mps3 - qm.jerk_ned_mps3) / (2.0 * h), qc.snap_ned_mps4,
          2e-5, "quintic analytic snap");

  threw = false;
  try {
    circleReference({1.0, 2.0, -1.0}, 1.5, 0.7, 0.25,
                    std::numeric_limits<double>::infinity(), 301.2);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "analytic trajectories reject non-finite elapsed time");

  const auto circle = circleReference({1.0, 2.0, -1.0}, 1.5, 0.7, 0.25, 1.2, 301.2);
  const auto circle_m = circleReference({1.0, 2.0, -1.0}, 1.5, 0.7, 0.25, 1.2 - h, 301.2 - h);
  const auto circle_p = circleReference({1.0, 2.0, -1.0}, 1.5, 0.7, 0.25, 1.2 + h, 301.2 + h);
  vecNear((circle_p.jerk_ned_mps3 - circle_m.jerk_ned_mps3) / (2 * h),
          circle.snap_ned_mps4, 2e-7, "circle snap");

  const auto fig = figureEightReference({0.0, 0.0, -2.0}, 2.0, 1.0, 0.6, 0.0, 0.9, 400.9);
  const auto fig_m = figureEightReference({0.0, 0.0, -2.0}, 2.0, 1.0, 0.6, 0.0, 0.9-h, 400.9-h);
  const auto fig_p = figureEightReference({0.0, 0.0, -2.0}, 2.0, 1.0, 0.6, 0.0, 0.9+h, 400.9+h);
  vecNear((fig_p.acceleration_ned_mps2 - fig_m.acceleration_ned_mps2)/(2*h),
          fig.jerk_ned_mps3, 3e-7, "figure eight jerk");
  const double figure_eight_peak_time = kPi / (4.0 * 0.6);
  const auto fig_peak = figureEightReference({0.0, 0.0, -2.0}, 2.0, 1.0, 0.6, 0.0,
                                             figure_eight_peak_time, 500.0);
  checkNear(fig_peak.position_ned_m.y, 1.0, 1e-12,
            "figure-eight y_amplitude_m is the actual peak amplitude");

  CanonicalState state{};
  state.external_position_ned_m = {{1,2,3}, 9.95};
  state.ekf_position_ned_m = {{1.1,2.1,3.1}, 9.8};
  state.ekf_velocity_ned_mps = {{0.1,0.2,0.3}, 9.96};
  state.attitude_ned_frd = {{1,0,0,0}, 9.97};
  state.body_rate_frd_radps = {{0.01,0.02,0.03}, 9.98};
  state.body_angular_accel_frd_radps2 = {{0.1,0.2,0.3}, 9.99};
  StateValidityLimits limits{};
  limits.max_external_position_age_s = 0.05;
  limits.max_ekf_velocity_age_s = 0.05;
  limits.max_attitude_age_s = 0.05;
  limits.max_body_rate_age_s = 0.05;
  limits.max_angular_accel_age_s = 0.05;
  limits.quaternion_norm_tolerance = 1e-3;
  auto valid = validateState(state, 10.0, limits, true);
  check(valid.ok, "state at inclusive age threshold must be valid");
  checkNear(valid.external_position_age_s, 0.05, 1e-12, "position age");

  state.external_position_ned_m.timestamp_s = 9.949;
  check(!validateState(state, 10.0, limits, false).ok, "stale direct position rejected");
  state.external_position_ned_m.timestamp_s = 9.99;
  state.body_angular_accel_frd_radps2.value.x = std::numeric_limits<double>::quiet_NaN();
  check(validateState(state, 10.0, limits, false).ok,
        "angular acceleration optional outside exact mirror");
  check(!validateState(state, 10.0, limits, true).ok,
        "angular acceleration mandatory when requested");

  state.attitude_ned_frd.value = {2,0,0,0};
  check(!validateState(state, 10.0, limits, false).ok, "non-unit attitude rejected");

  state.attitude_ned_frd.value = {1, 0, 0, 0};
  StateValidityLimits invalid_limits = limits;
  invalid_limits.max_attitude_age_s = -0.1;
  const auto invalid_limits_result = validateState(state, 10.0, invalid_limits, false);
  check(!invalid_limits_result.ok, "negative freshness limit rejected");
  check(invalid_limits_result.reason == "invalid state-validity limits",
        "invalid freshness configuration is distinguished from stale data");
  return 0;
}
