#include "control/frames.hpp"

namespace control {
namespace {

Mat3 nedFromEnu() {
  Mat3 transform = Mat3::zero();
  transform(0, 1) = 1.0;
  transform(1, 0) = 1.0;
  transform(2, 2) = -1.0;
  return transform;
}

Mat3 frdFromFlu() {
  Mat3 transform = Mat3::zero();
  transform(0, 0) = 1.0;
  transform(1, 1) = -1.0;
  transform(2, 2) = -1.0;
  return transform;
}

}  // namespace

Vec3 enuToNed(const Vec3 &vector_enu) {
  return nedFromEnu() * vector_enu;
}

Vec3 nedToEnu(const Vec3 &vector_ned) {
  return nedFromEnu() * vector_ned;
}

Vec3 fluToFrd(const Vec3 &vector_flu) {
  return frdFromFlu() * vector_flu;
}

Vec3 frdToFlu(const Vec3 &vector_frd) {
  return frdFromFlu() * vector_frd;
}

Mat3 enuFluToNedFrd(const Mat3 &rotation_enu_flu) {
  return nedFromEnu() * rotation_enu_flu * frdFromFlu();
}

Mat3 nedFrdToEnuFlu(const Mat3 &rotation_ned_frd) {
  return nedFromEnu() * rotation_ned_frd * frdFromFlu();
}

Quat enuFluToNedFrd(const Quat &quaternion_enu_flu) {
  return Quat::fromMat3(enuFluToNedFrd(quaternion_enu_flu.toMat3()));
}

Quat nedFrdToEnuFlu(const Quat &quaternion_ned_frd) {
  return Quat::fromMat3(nedFrdToEnuFlu(quaternion_ned_frd.toMat3()));
}

}  // namespace control
