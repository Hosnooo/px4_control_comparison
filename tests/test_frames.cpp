#include "control/frames.hpp"
#include "test_support.hpp"

#include <cmath>
#include <string>

using namespace control;

namespace {

void checkVecNear(const Vec3 &actual, const Vec3 &expected, double tolerance,
                  const std::string &message) {
  checkNear(actual.x, expected.x, tolerance, message + " x");
  checkNear(actual.y, expected.y, tolerance, message + " y");
  checkNear(actual.z, expected.z, tolerance, message + " z");
}

void testBasisVectors() {
  checkVecNear(enuToNed({1.0, 0.0, 0.0}), {0.0, 1.0, 0.0}, 1e-12, "east basis");
  checkVecNear(enuToNed({0.0, 1.0, 0.0}), {1.0, 0.0, 0.0}, 1e-12, "north basis");
  checkVecNear(enuToNed({0.0, 0.0, 1.0}), {0.0, 0.0, -1.0}, 1e-12, "up basis");

  checkVecNear(fluToFrd({1.0, 0.0, 0.0}), {1.0, 0.0, 0.0}, 1e-12, "front basis");
  checkVecNear(fluToFrd({0.0, 1.0, 0.0}), {0.0, -1.0, 0.0}, 1e-12, "left basis");
  checkVecNear(fluToFrd({0.0, 0.0, 1.0}), {0.0, 0.0, -1.0}, 1e-12, "up body basis");
}

void testVectorRoundTrips() {
  const Vec3 vector{1.2, -4.5, 2.7};
  checkVecNear(nedToEnu(enuToNed(vector)), vector, 1e-12, "world round trip");
  checkVecNear(frdToFlu(fluToFrd(vector)), vector, 1e-12, "body round trip");
}

void testRotationBoundary() {
  const Vec3 body_vector_flu{0.3, -0.8, 1.1};

  for (int index = 0; index < 200; ++index) {
    const double phase = 0.173 * static_cast<double>(index + 1);
    const Vec3 axis{1.0 + std::sin(phase),
                    -0.4 + std::cos(0.7 * phase),
                    0.2 + std::sin(1.3 * phase)};
    const double angle_rad = -2.8 + 5.6 * static_cast<double>(index) / 199.0;
    const Quat q_enu_flu = Quat::fromAxisAngle(axis, angle_rad);
    const Quat q_ned_frd = enuFluToNedFrd(q_enu_flu);
    const Quat round_trip = nedFrdToEnuFlu(q_ned_frd);

    check(q_enu_flu.rotationDistance(round_trip) < 1e-7,
          "quaternion frame conversion round trip");
    checkNear(determinant(q_ned_frd.toMat3()), 1.0, 1e-12,
              "converted rotation determinant");

    const Vec3 world_vector_enu = q_enu_flu.toMat3() * body_vector_flu;
    const Vec3 world_vector_ned = q_ned_frd.toMat3() * fluToFrd(body_vector_flu);
    checkVecNear(world_vector_ned, enuToNed(world_vector_enu), 1e-11,
                 "rotation and vector frame conversion agree");
  }
}

}  // namespace

int main() {
  testBasisVectors();
  testVectorRoundTrips();
  testRotationBoundary();
  return 0;
}
