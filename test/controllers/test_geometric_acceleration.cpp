#include "px4_offboard_controllers/controllers/geometric_acceleration.hpp"
#include <cassert>
using namespace px4_offboard;
int main(){
  GeometricAccelerationController c({2,3,4},{5,6,7});
  CanonicalState s; s.position_ned=TimedValue<Vec3>{{1,2,3},0.0}; s.velocity_ned=TimedValue<Vec3>{{0.5,-1,2},0.0};
  TrajectoryReference r; r.position=Vec3{1,2,3}; r.velocity=Vec3{0.5,-1,2}; r.acceleration=Vec3{0.1,0.2,0.3};
  auto zero=c.compute(s,r); assert(zero.acceleration_ned.x==0.1&&zero.acceleration_ned.y==0.2&&zero.acceleration_ned.z==0.3);
  r.position=Vec3{0,1,1}; r.velocity=Vec3{0,0,1}; r.acceleration=Vec3{0,0,0};
  auto out=c.compute(s,r);
  assert(out.acceleration_ned.x==-4.5); // -2*(1)-5*(0.5)
  assert(out.acceleration_ned.y==3.0);  // -3*(1)-6*(-1)
  assert(out.acceleration_ned.z==-15.0); // -4*(2)-7*(1)
}
