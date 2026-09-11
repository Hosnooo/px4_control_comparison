#include "runtime.hpp"

#include <cassert>
#include <cmath>

using namespace px4_offboard;
using namespace px4_offboard::ros2_runtime;

int main() {
  CanonicalState state{};
  state.position_ned = TimedValue<Vec3>{{4.0, 5.0, -6.0}, 2.0};

  StateRequirements requirements{};
  requirements.position = true;
  requirements.velocity = true;

  px4_msgs::msg::VehicleLocalPosition message{};
  message.timestamp_sample = 3'000'000;
  message.xy_valid = true;
  message.z_valid = true;
  message.v_xy_valid = true;
  message.v_z_valid = true;
  message.x = 100.0F;
  message.y = 200.0F;
  message.z = -300.0F;
  message.vx = 1.0F;
  message.vy = 2.0F;
  message.vz = -3.0F;

  updateFromPx4LocalPosition(state, requirements, message);

  assert(state.position_ned.has_value());
  assert(state.position_ned->value.x == 4.0);
  assert(state.position_ned->value.y == 5.0);
  assert(state.position_ned->value.z == -6.0);
  assert(state.position_ned->timestamp_s == 2.0);

  assert(state.velocity_ned.has_value());
  assert(std::abs(state.velocity_ned->value.x - 1.0) < 1e-12);
  assert(std::abs(state.velocity_ned->value.y - 2.0) < 1e-12);
  assert(std::abs(state.velocity_ned->value.z + 3.0) < 1e-12);
  assert(std::abs(state.velocity_ned->timestamp_s - 3.0) < 1e-12);

  message.v_xy_valid = false;
  updateFromPx4LocalPosition(state, requirements, message);
  assert(!state.velocity_ned.has_value());
  assert(state.position_ned.has_value());
  assert(state.position_ned->value.x == 4.0);
}
