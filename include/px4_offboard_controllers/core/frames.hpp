#pragma once

#include "px4_offboard_controllers/core/vector3.hpp"

namespace px4_offboard {

// Gazebo ENU position to canonical world NED. Call only at the source boundary.
inline Vec3 enuToNed(const Vec3 &value_enu) {
  return {value_enu.y, value_enu.x, -value_enu.z};
}

}  // namespace px4_offboard
