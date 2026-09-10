add_library(control_core
  src/frames.cpp
  src/state.cpp
  src/trajectory.cpp
  src/lee_controller.cpp
  src/px4_thrust_normalization.cpp
  src/geometric_rate_controller.cpp
  src/px4_attitude_rate_controller.cpp
)
target_include_directories(control_core PUBLIC include)
