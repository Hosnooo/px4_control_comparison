cmake_minimum_required(VERSION 3.16)
project(px4_control_comparison_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

add_library(simulation_position_runtime_core STATIC
  ../src/gazebo_position.cpp
  ../../control/src/frames.cpp
)
set_target_properties(simulation_position_runtime_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(simulation_position_runtime_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../control/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_library(gazebo_position_runtime SHARED
  src/gazebo_pose_relay.cpp
  src/gazebo_position_ros.cpp
)
target_include_directories(gazebo_position_runtime PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../control/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(gazebo_position_runtime
  PUBLIC gz-msgs::core
  PRIVATE simulation_position_runtime_core
)
ament_target_dependencies(gazebo_position_runtime PUBLIC geometry_msgs)

add_executable(gazebo_pose_relay src/gazebo_pose_relay_main.cpp)
target_link_libraries(gazebo_pose_relay PRIVATE gazebo_position_runtime gz-transport::core)

install(TARGETS gazebo_position_runtime
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS gazebo_pose_relay
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY ../include/simulation DESTINATION include/${PROJECT_NAME})
install(FILES
  ../../control/include/control/frames.hpp
  ../../control/include/control/math.hpp
  ../../control/include/control/state.hpp
  DESTINATION include/${PROJECT_NAME}/control
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_gazebo_runtime test/test_gazebo_runtime.cpp)
  target_include_directories(test_gazebo_runtime PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tests
  )
  target_link_libraries(test_gazebo_runtime PRIVATE gazebo_position_runtime)
  add_test(NAME test_gazebo_runtime COMMAND test_gazebo_runtime)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs gz_msgs_vendor gz-msgs gz_transport_vendor gz-transport)
ament_package()
