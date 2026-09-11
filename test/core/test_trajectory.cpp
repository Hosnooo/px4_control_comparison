#include "px4_offboard_controllers/core/trajectory.hpp"
#include <cassert>
using namespace px4_offboard;
int main(){
  TrajectoryReference r; r.position={1,2,3};
  assert(validateReference(r, ReferenceProfile::PositionOnly));
  TrajectoryReference v; v.velocity={1,0,0};
  assert(validateReference(v, ReferenceProfile::VelocityOnly));
  TrajectoryReference lee; lee.position={0,0,0}; lee.velocity={0,0,0}; lee.acceleration={0,0,0}; lee.jerk={0,0,0}; lee.snap={0,0,0}; lee.yaw=0; lee.yaw_rate=0; lee.yaw_accel=0;
  assert(validateReference(lee, ReferenceProfile::LeeFull));
}
