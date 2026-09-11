#include "px4_offboard_controllers/state_sources/gazebo_position.hpp"
#include "px4_offboard_controllers/core/frames.hpp"
#include <cmath>
namespace px4_offboard {
namespace { bool validTimestamp(double t){return std::isfinite(t)&&t>=0.0;} }
GazeboPoseSelectionResult selectGazeboModelPose(const GazeboPoseVectorSample&s,std::string_view world,std::string_view model){
 GazeboPoseSelectionResult r{}; if(world.empty()||model.empty()){r.reason="invalid Gazebo identity configuration";return r;} if(!validTimestamp(s.timestamp_s)){r.reason="invalid Gazebo simulation timestamp";return r;}
 const GazeboNamedPose* match=nullptr; std::size_t idx=0,count=0; for(std::size_t i=0;i<s.poses.size();++i){if(s.poses[i].name==model){match=&s.poses[i];idx=i;++count;}}
 if(count==0){r.reason="Gazebo model pose not found";return r;} if(count!=1){r.reason="Gazebo model pose is ambiguous";return r;} if(!match||!match->position_enu_m.finite()){r.reason="invalid Gazebo model position";return r;}
 r.source_pose_index=idx; r.pose={std::string(world),std::string(model),match->position_enu_m,s.timestamp_s}; r.ok=true; return r;
}
GazeboPositionResult makeCanonicalGazeboPosition(const GazeboDirectPose&p,std::string_view world,std::string_view model){
 GazeboPositionResult r{}; if(world.empty()||model.empty()){r.reason="invalid Gazebo identity configuration";return r;} if(p.frame_id!=world||p.child_frame_id!=model){r.reason="unexpected Gazebo pose identity";return r;} if(!validTimestamp(p.timestamp_s)){r.reason="invalid Gazebo simulation timestamp";return r;} if(!p.position_enu_m.finite()){r.reason="invalid Gazebo model position";return r;} r.position_ned_m={enuToNed(p.position_enu_m),p.timestamp_s}; r.ok=true; return r;
}
}
