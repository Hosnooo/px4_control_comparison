cmake_minimum_required(VERSION 3.16)
project(px4_offboard_controllers LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)

add_library(px4_offboard_core src/core/state.cpp src/core/trajectory.cpp)
target_include_directories(px4_offboard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(px4_offboard_controllers_lib
  src/controllers/geometric_acceleration.cpp
  src/controllers/geometric_rate.cpp
  src/controllers/lee.cpp
  src/controllers/px4_attitude_rate_mirror.cpp
  src/controllers/px4_thrust_normalization.cpp)
target_link_libraries(px4_offboard_controllers_lib PUBLIC px4_offboard_core)
target_include_directories(px4_offboard_controllers_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(px4_offboard_px4 src/px4/command_builder.cpp)
target_link_libraries(px4_offboard_px4 PUBLIC px4_offboard_core)
target_include_directories(px4_offboard_px4 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(px4_offboard_state_sources src/state_sources/gazebo_position.cpp)
target_link_libraries(px4_offboard_state_sources PUBLIC px4_offboard_core)
target_include_directories(px4_offboard_state_sources PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(px4_offboard_f450 vehicles/f450/src/wrench_model.cpp)
target_link_libraries(px4_offboard_f450 PUBLIC px4_offboard_core)
target_include_directories(px4_offboard_f450 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vehicles/f450/include>
  $<INSTALL_INTERFACE:include>)

add_library(px4_offboard_hardware_calibration
  experiment/calibration/src/hardware_wrench_calibration.cpp)
target_link_libraries(px4_offboard_hardware_calibration PUBLIC px4_offboard_core)
target_include_directories(px4_offboard_hardware_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/experiment/calibration/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(px4_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(gz_msgs_vendor REQUIRED)
  find_package(gz-msgs REQUIRED)
  find_package(gz_transport_vendor REQUIRED)
  find_package(gz-transport REQUIRED)

  add_library(px4_offboard_ros2_messages src/ros2/px4_message_adapter.cpp)
  target_link_libraries(px4_offboard_ros2_messages PUBLIC px4_offboard_px4)
  target_include_directories(px4_offboard_ros2_messages PUBLIC include)
  ament_target_dependencies(px4_offboard_ros2_messages px4_msgs)

  add_library(px4_offboard_gazebo_runtime SHARED
    src/ros2/gazebo_pose_relay.cpp
    src/ros2/gazebo_position_ros.cpp)
  target_include_directories(px4_offboard_gazebo_runtime PRIVATE src/ros2)
  target_link_libraries(px4_offboard_gazebo_runtime
    PRIVATE px4_offboard_state_sources gz-msgs::core)
  ament_target_dependencies(px4_offboard_gazebo_runtime geometry_msgs)

  add_executable(gazebo_pose_relay src/ros2/gazebo_pose_relay_main.cpp)
  target_include_directories(gazebo_pose_relay PRIVATE src/ros2)
  target_link_libraries(gazebo_pose_relay
    PRIVATE px4_offboard_gazebo_runtime gz-transport::core)

  add_executable(offboard_controller
    src/ros2/offboard_controller_node.cpp
    src/ros2/px4_state_input.cpp
    src/ros2/px4_command_publisher.cpp)
  target_link_libraries(offboard_controller PRIVATE
    px4_offboard_core px4_offboard_controllers_lib px4_offboard_px4 px4_offboard_ros2_messages)
  ament_target_dependencies(offboard_controller rclcpp px4_msgs)

  install(TARGETS px4_offboard_controllers_lib px4_offboard_px4 px4_offboard_state_sources
                  px4_offboard_f450 px4_offboard_ros2_messages px4_offboard_gazebo_runtime
                  gazebo_pose_relay offboard_controller
    DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY vehicles/f450/include/ DESTINATION include)
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  ament_package()
else()
  # Host-only builds intentionally exclude native ROS 2/Gazebo execution. Message conversion is
  # still compiled below against audited field-shape stubs so the PX4 boundary remains testable.
  add_executable(offboard_controller src/ros2/offboard_controller_node.cpp)
  target_link_libraries(offboard_controller PRIVATE
    px4_offboard_core px4_offboard_controllers_lib px4_offboard_px4)
endif()

if(BUILD_TESTING)
  function(add_host_test name source)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(${name} PRIVATE px4_offboard_core)
    add_test(NAME ${name} COMMAND ${name})
  endfunction()

  add_host_test(test_math test/core/test_math.cpp)
  add_host_test(test_state_requirements test/core/test_state_requirements.cpp)
  add_host_test(test_trajectory test/core/test_trajectory.cpp)

  foreach(test_name IN ITEMS geometric_acceleration geometric_rate lee px4_attitude_rate_mirror
                             px4_thrust_normalization)
    add_executable(test_${test_name} test/controllers/test_${test_name}.cpp)
    target_include_directories(test_${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(test_${test_name} PRIVATE px4_offboard_controllers_lib)
    add_test(NAME test_${test_name} COMMAND test_${test_name})
  endforeach()

  foreach(test_name IN ITEMS command_types command_builder_contract offboard_flags wrench_normalization)
    add_executable(test_${test_name} test/px4/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} PRIVATE px4_offboard_px4)
    add_test(NAME test_${test_name} COMMAND test_${test_name})
  endforeach()

  add_executable(test_gazebo_position test/state_sources/test_gazebo_position.cpp)
  target_link_libraries(test_gazebo_position PRIVATE px4_offboard_state_sources)
  add_test(NAME test_gazebo_position COMMAND test_gazebo_position)

  add_executable(test_f450_wrench_model test/vehicles/test_f450_wrench_model.cpp)
  target_include_directories(test_f450_wrench_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_f450_wrench_model PRIVATE px4_offboard_f450)
  add_test(NAME test_f450_wrench_model COMMAND test_f450_wrench_model)

  add_executable(test_hardware_wrench_calibration
    test/calibration/test_hardware_wrench_calibration.cpp)
  target_include_directories(test_hardware_wrench_calibration PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_hardware_wrench_calibration PRIVATE
    px4_offboard_hardware_calibration)
  add_test(NAME test_hardware_wrench_calibration COMMAND test_hardware_wrench_calibration)

  if(NOT ament_cmake_FOUND)
    add_library(px4_offboard_ros2_messages src/ros2/px4_message_adapter.cpp)
    target_include_directories(px4_offboard_ros2_messages PUBLIC include test/stubs)
    target_link_libraries(px4_offboard_ros2_messages PUBLIC px4_offboard_px4)
  endif()
  add_executable(test_px4_message_adapter test/ros2/test_px4_message_adapter.cpp)
  target_include_directories(test_px4_message_adapter PRIVATE test/stubs)
  target_link_libraries(test_px4_message_adapter PRIVATE px4_offboard_ros2_messages)
  add_test(NAME test_px4_message_adapter COMMAND test_px4_message_adapter)
endif()
