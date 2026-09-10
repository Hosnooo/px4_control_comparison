#pragma once

#include "control/f450_wrench_model.hpp"
#include "control/geometric_rate_controller.hpp"
#include "control/handoff.hpp"
#include "control/hardware_wrench_calibration.hpp"
#include "control/lee_controller.hpp"
#include "control/px4_attitude_rate_controller.hpp"
#include "control/px4_thrust_normalization.hpp"

namespace control {

HandoffInputs makeAttitudeHandoffInputs(const LeeOutput &lee,
                                        const TrajectoryReference &reference,
                                        const Px4ThrustOutput &thrust,
                                        bool reset_integral,
                                        const HandoffTiming &timing);

HandoffInputs makeRateHandoffInputs(const GeometricRateOutput &rate,
                                    const Px4ThrustOutput &thrust,
                                    bool reset_integral,
                                    const HandoffTiming &timing);

HandoffInputs makePx4MirrorHandoffInputs(const Px4RateOutput &mirror,
                                         const HandoffTiming &timing);

HandoffInputs makeLeeWrenchHandoffInputs(const F450WrenchResult &wrench,
                                         const HandoffTiming &timing);

HandoffInputs makeLeeWrenchHandoffInputs(const HardwareWrenchResult &wrench,
                                         const HandoffTiming &timing);

}  // namespace control
