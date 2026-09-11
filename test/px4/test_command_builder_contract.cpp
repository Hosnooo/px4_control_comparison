#include "px4_offboard_controllers/px4/command_builder.hpp"
#include <cassert>
using namespace px4_offboard;
int main(){
 assert(controlLevelFor(ControllerKind::Px4Position)==OffboardControlLevel::Position);
 assert(controlLevelFor(ControllerKind::Px4Velocity)==OffboardControlLevel::Velocity);
 assert(controlLevelFor(ControllerKind::GeometricAcceleration)==OffboardControlLevel::Acceleration);
 assert(controlLevelFor(ControllerKind::GeometricAttitude)==OffboardControlLevel::Attitude);
 assert(controlLevelFor(ControllerKind::GeometricRate)==OffboardControlLevel::BodyRate);
 assert(controlLevelFor(ControllerKind::Px4AttitudeRateMirror)==OffboardControlLevel::BodyRate);
 assert(controlLevelFor(ControllerKind::LeeWrench)==OffboardControlLevel::Wrench);
 assert(kTrajectorySetpointTopic=="/fmu/in/trajectory_setpoint");
 assert(kOffboardControlModeTopic=="/fmu/in/offboard_control_mode");
}
