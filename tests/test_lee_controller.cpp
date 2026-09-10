#include "control/lee_controller.hpp"
#include "control/trajectory.hpp"
#include "test_support.hpp"

using namespace control;

static void vecNear(const Vec3 &a, const Vec3 &b, double tol, const char *msg) {
  checkNear(a.x,b.x,tol,std::string(msg)+" x");
  checkNear(a.y,b.y,tol,std::string(msg)+" y");
  checkNear(a.z,b.z,tol,std::string(msg)+" z");
}
static double matMaxAbs(const Mat3 &m) {
  double v=0; for(double x:m.a) v=std::max(v,std::abs(x)); return v;
}

static CanonicalState hoverState() {
  CanonicalState s{};
  s.external_position_ned_m.value = {0,0,-1};
  s.ekf_velocity_ned_mps.value = {};
  s.attitude_ned_frd.value = {1,0,0,0};
  s.body_rate_frd_radps.value = {};
  return s;
}

int main() {
  bool threw = false;
  try {
    LeeController unconfigured(LeeConfig{});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "Lee controller requires explicit configuration");

  LeeConfig cfg{};
  cfg.mass_kg=2.0;
  cfg.gravity_mps2=9.80665;
  cfg.k_position={4,4,6};
  cfg.k_velocity={3,3,4};
  cfg.k_attitude={3.5,3.5,1.2};
  cfg.k_rate={0.35,0.35,0.2};
  cfg.inertia_kgm2=diagonal({0.02,0.02,0.04});
  LeeController lee(cfg);

  auto s=hoverState();
  auto r=hoverReference({0,0,-1},0,0);
  const auto out=lee.update(s,r);
  vecNear(out.position_error_ned_m,{},1e-12,"hover position error");
  vecNear(out.velocity_error_ned_mps,{},1e-12,"hover velocity error");
  vecNear(out.force_ned_n,{0,0,-cfg.mass_kg*cfg.gravity_mps2},1e-10,"hover control force");
  vecNear(out.desired_body_z_ned,{0,0,1},1e-12,"hover body z");
  checkNear(out.collective_thrust_n,cfg.mass_kg*cfg.gravity_mps2,1e-10,"hover thrust");
  check(Quat::fromMat3(out.desired_rotation_ned_frd).rotationDistance({1,0,0,0})<1e-10,
        "hover desired attitude is level");
  vecNear(out.desired_body_rate_frd_radps,{},1e-10,"hover desired rate");
  vecNear(out.attitude_error,{},1e-10,"hover attitude error");
  vecNear(out.body_moment_frd_nm,{},1e-10,"hover moment");

  s.attitude_ned_frd.value=Quat::fromAxisAngle({1,0,0},0.12);
  const auto tilted=lee.update(s,r);
  check(tilted.attitude_error.x>0,"positive roll attitude error sign");
  check(tilted.body_moment_frd_nm.x<0,"moment opposes positive roll error");

  // Desired yaw acceleration enters the exact feed-forward term. At zero error,
  // a pure yaw acceleration requires Jz * yaw_accel.
  r=hoverReference({0,0,-1},0.3,0.0);
  r.yaw_rate_radps=0.7;
  r.yaw_accel_radps2=0.4;
  s=hoverState();
  const auto desired=lee.update(s,r);
  s.attitude_ned_frd.value=Quat::fromMat3(desired.desired_rotation_ned_frd);
  s.body_rate_frd_radps.value=desired.desired_body_rate_frd_radps;
  const auto ff=lee.update(s,r);
  vecNear(ff.attitude_error,{},1e-9,"feedforward zero attitude error");
  vecNear(ff.rate_error_frd_radps,{},1e-9,"feedforward zero rate error");
  checkNear(ff.body_moment_frd_nm.z, cfg.inertia_kgm2(2,2)*0.4, 2e-8,
            "yaw angular acceleration feedforward moment");

  // Production derivatives are analytic. Validate them against centered finite differences
  // on a dynamic reference without putting numerical differentiation in the controller.
  s=hoverState();
  s.external_position_ned_m.value={0.15,-0.08,-1.1};
  s.ekf_velocity_ned_mps.value={0.12,0.03,-0.04};
  s.attitude_ned_frd.value=Quat::fromAxisAngle({0.2,-0.3,0.5},0.18);
  s.body_rate_frd_radps.value={0.1,-0.08,0.05};
  const double t=0.73, h=1e-4;
  auto dynamic=figureEightReference({0,0,-1},1.2,0.7,0.6,0.2,t,t);
  dynamic.yaw_rate_radps=0.15;
  dynamic.yaw_accel_radps2=-0.03;
  const auto center=lee.update(s,dynamic);
  const Mat3 rdot_relation=center.desired_rotation_ned_frd*hat(center.desired_body_rate_frd_radps);
  check(matMaxAbs(center.desired_rotation_dot-rdot_relation)<2e-10,
        "Rdot equals R hat(Omega_d)");
  check(center.force_ned_n.finite() && center.body_moment_frd_nm.finite(),
        "dynamic Lee output finite");

  // Check the internal normalized-vector kinematics by one first-order model step.
  const Mat3 predicted=center.desired_rotation_ned_frd + center.desired_rotation_dot*h +
                       center.desired_rotation_ddot*(0.5*h*h);
  const Mat3 orth=predicted.transpose()*predicted;
  check(std::abs(orth(0,0)-1.0)<2e-6 && std::abs(orth(1,1)-1.0)<2e-6 &&
        std::abs(orth(2,2)-1.0)<2e-6, "analytic desired attitude derivatives preserve SO3 locally");

  threw = false;
  try {
    LeeConfig bad = cfg;
    bad.mass_kg = 0.0;
    LeeController invalid(bad);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "invalid mass rejected");

  threw = false;
  try {
    LeeConfig bad = cfg;
    bad.inertia_kgm2 = Mat3::identity();
    bad.inertia_kgm2(0, 1) = 10.0;
    LeeController invalid(bad);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "non-symmetric inertia rejected");

  threw = false;
  try {
    LeeConfig bad = cfg;
    bad.k_attitude.x = -1.0;
    LeeController invalid(bad);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "Lee gains must be positive");
  return 0;
}
