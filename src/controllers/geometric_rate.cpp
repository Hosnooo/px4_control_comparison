#include "px4_offboard_controllers/controllers/geometric_rate.hpp"
#include <stdexcept>
namespace px4_offboard {
GeometricRateController::GeometricRateController(GeometricRateConfig c):config_(c){if(!c.k_attitude.finite()||!c.rate_limit_radps.finite()||c.k_attitude.x<0||c.k_attitude.y<0||c.k_attitude.z<0||c.rate_limit_radps.x<=0||c.rate_limit_radps.y<=0||c.rate_limit_radps.z<=0)throw std::invalid_argument("invalid geometric-rate configuration");}
GeometricRateOutput GeometricRateController::update(const Mat3&r,const Mat3&rd,const Vec3&wd)const{if(!r.finite()||!rd.finite()||!wd.finite())throw std::invalid_argument("invalid geometric-rate input");GeometricRateOutput o{};o.attitude_error=.5*vee(rd.transpose()*r-r.transpose()*rd);const Vec3 ff=r.transpose()*rd*wd;o.body_rate_setpoint_frd_radps=clamped(ff-hadamard(config_.k_attitude,o.attitude_error),config_.rate_limit_radps);return o;}
}
