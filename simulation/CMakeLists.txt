add_library(simulation_position
  src/gazebo_position.cpp
)
target_include_directories(simulation_position PUBLIC include)
target_link_libraries(simulation_position PUBLIC control_core)
