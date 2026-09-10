#pragma once
#include "control/math.hpp"

namespace control {
Vec3 enuToNed(const Vec3 &v_enu);
Vec3 nedToEnu(const Vec3 &v_ned);
Vec3 fluToFrd(const Vec3 &v_flu);
Vec3 frdToFlu(const Vec3 &v_frd);
Mat3 enuFluToNedFrd(const Mat3 &r_enu_flu);
Mat3 nedFrdToEnuFlu(const Mat3 &r_ned_frd);
Quat enuFluToNedFrd(const Quat &q_enu_flu);
Quat nedFrdToEnuFlu(const Quat &q_ned_frd);
}  // namespace control
