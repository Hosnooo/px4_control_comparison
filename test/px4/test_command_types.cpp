#include "px4_offboard_controllers/px4/command_types.hpp"
#include <cassert>
using namespace px4_offboard;
int main(){
  PositionCommand p{{1,2,3},42}; assert(p.valid());
  VelocityCommand v{{1,2,3},42}; assert(v.valid());
  AccelerationCommand a{{1,2,3},42}; assert(a.valid());
  NormalizedWrenchCommand w{{0.1,0.2,0.3},{0,0,-0.5},42}; assert(w.valid());
  NormalizedWrenchCommand bad{{2,0,0},{0,0,0},42}; assert(!bad.valid());
}
