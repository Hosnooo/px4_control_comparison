#include "px4_offboard_controllers/px4/command_builder.hpp"
#include <cassert>
using namespace px4_offboard;
int main(){
 auto p=offboardFlags(OffboardControlLevel::Position); assert(p.position&&!p.velocity&&!p.acceleration&&!p.attitude&&!p.body_rate&&!p.thrust_and_torque);
 auto v=offboardFlags(OffboardControlLevel::Velocity); assert(!v.position&&v.velocity&&!v.acceleration);
 auto a=offboardFlags(OffboardControlLevel::Acceleration); assert(!a.position&&!a.velocity&&a.acceleration);
 auto q=offboardFlags(OffboardControlLevel::Attitude); assert(q.attitude&&!q.body_rate);
 auto r=offboardFlags(OffboardControlLevel::BodyRate); assert(!r.attitude&&r.body_rate);
 auto w=offboardFlags(OffboardControlLevel::Wrench); assert(w.thrust_and_torque&&!w.body_rate);
}
