add_library(control_core
  src/frames.cpp
  src/state.cpp
  src/trajectory.cpp
  src/lee_controller.cpp
  src/px4_thrust_normalization.cpp
  src/geometric_rate_controller.cpp
  src/px4_attitude_rate_controller.cpp
  src/f450_wrench_model.cpp
  src/hardware_wrench_calibration.cpp
  src/handoff.cpp
  src/controller_handoff.cpp
)
target_include_directories(control_core PUBLIC include)
