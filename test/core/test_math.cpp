#include "px4_offboard_controllers/core/vector3.hpp"
#include <cassert>
#include <stdexcept>
using px4_offboard::Vec3;
int main(){
  Vec3 v{1.0,2.0,3.0};
  assert(v.at(0)==1.0 && v.at(1)==2.0 && v.at(2)==3.0);
  bool threw=false; try { (void)v.at(3); } catch(const std::out_of_range&) { threw=true; }
  assert(threw);
}
