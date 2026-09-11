#include "px4_offboard_controllers/state_sources/gazebo_position.hpp"
#include <cassert>
using namespace px4_offboard;
int main(){
  GazeboPoseVectorSample s; s.timestamp_s=1.25; s.poses={{"other",{9,9,9}},{"uav",{1,2,3}}};
  auto sel=selectGazeboModelPose(s,"default","uav"); assert(sel.ok&&sel.source_pose_index==1);
  auto out=makeCanonicalGazeboPosition(sel.pose,"default","uav"); assert(out.ok);
  assert(out.position_ned_m.value.x==2&&out.position_ned_m.value.y==1&&out.position_ned_m.value.z==-3&&out.position_ned_m.timestamp_s==1.25);
  s.poses.push_back({"uav",{0,0,0}}); assert(!selectGazeboModelPose(s,"default","uav").ok);
}
