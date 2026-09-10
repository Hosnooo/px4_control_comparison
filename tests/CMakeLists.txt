function(add_control_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE control_core)
  add_test(NAME ${name} COMMAND ${name})
endfunction()
add_control_test(test_frames)
add_control_test(test_state_trajectory)
add_control_test(test_lee_controller)
add_control_test(test_px4_thrust_normalization)
add_control_test(test_geometric_rate_controller)
add_control_test(test_px4_attitude_rate_controller)
add_control_test(test_f450_wrench_model)
add_control_test(test_hardware_wrench_calibration)
add_control_test(test_handoff)
add_control_test(test_controller_handoff)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(
  NAME test_fit_wrench_calibration
  COMMAND ${Python3_EXECUTABLE} -m unittest tests/test_fit_wrench_calibration.py -v
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
