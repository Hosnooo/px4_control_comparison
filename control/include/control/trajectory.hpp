#pragma once

#include "control/state.hpp"

namespace control {

TrajectoryReference hoverReference(const Vec3 &position_ned_m, double yaw_rad,
                                   double timestamp_s);
TrajectoryReference quinticReference(const Vec3 &start_ned_m, const Vec3 &end_ned_m,
                                     double start_yaw_rad, double end_yaw_rad,
                                     double duration_s, double elapsed_s, double timestamp_s);
TrajectoryReference circleReference(const Vec3 &center_ned_m, double radius_m,
                                    double angular_rate_radps, double yaw_rad,
                                    double elapsed_s, double timestamp_s);
TrajectoryReference figureEightReference(const Vec3 &center_ned_m, double x_amplitude_m,
                                         double y_amplitude_m, double angular_rate_radps,
                                         double yaw_rad, double elapsed_s, double timestamp_s);

}  // namespace control
