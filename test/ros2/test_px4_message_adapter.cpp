#include "px4_offboard_controllers/px4/message_adapter.hpp"
#include <cmath>
#include <stdexcept>
using namespace px4_offboard;
int main(){
  PositionCommand p{{1,2,3},42}; auto tp=toPx4TrajectorySetpoint(p); if(tp.timestamp!=42||tp.position[0]!=1||!std::isnan(tp.velocity[0])||!std::isnan(tp.acceleration[0])) return 1;
  VelocityCommand v{{4,5,6},43}; auto tv=toPx4TrajectorySetpoint(v); if(tv.velocity[2]!=6||!std::isnan(tv.position[0])) return 2;
  AccelerationCommand a{{7,8,9},44}; auto ta=toPx4TrajectorySetpoint(a); if(ta.acceleration[1]!=8||!std::isnan(ta.position[2])) return 3;
  auto mode=toPx4OffboardControlMode(OffboardControlLevel::BodyRate,50); if(!mode.body_rate||mode.attitude||mode.timestamp!=50) return 4;
  AttitudeCommand q{{1,0,0,0},.4,51}; auto qa=toPx4VehicleAttitudeSetpoint(q); if(qa.q_d[0]!=1||qa.thrust_body[2]!=-.4f) return 5;
  BodyRateCommand r{{.1,.2,.3},.5,52}; auto rr=toPx4VehicleRatesSetpoint(r); if(rr.roll!=.1f||rr.thrust_body[2]!=-.5f) return 6;
  NormalizedWrenchCommand w{{.1,.2,.3},{0,0,-.6},53}; auto tw=toPx4VehicleTorqueSetpoint(w); auto fw=toPx4VehicleThrustSetpoint(w); if(tw.xyz[1]!=.2f||fw.xyz[2]!=-.6f||tw.timestamp_sample!=53) return 7;
  return 0;
}
