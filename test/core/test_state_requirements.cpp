#include "px4_offboard_controllers/core/state.hpp"
#include <cassert>
using namespace px4_offboard;
int main(){
  { auto r=requirementsFor(ControllerKind::Px4Position); assert(!r.position&&!r.velocity&&!r.attitude&&!r.body_rate); }
  { auto r=requirementsFor(ControllerKind::Px4Velocity); assert(!r.position&&!r.velocity&&!r.attitude&&!r.body_rate); }
  { auto r=requirementsFor(ControllerKind::GeometricAcceleration); assert(r.position&&r.velocity&&!r.attitude&&!r.body_rate); }
  { auto r=requirementsFor(ControllerKind::GeometricRate); assert(r.position&&r.velocity&&r.attitude&&!r.body_rate); }
  { auto r=requirementsFor(ControllerKind::LeeWrench); assert(r.position&&r.velocity&&r.attitude&&r.body_rate); }
  auto m=requirementsFor(ControllerKind::Px4AttitudeRateMirror);
  assert(m.attitude && m.body_rate);
}
