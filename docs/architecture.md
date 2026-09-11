# Architecture

The repository is one top-level `ament_cmake` package with narrow ownership boundaries.
`px4_offboard_core` owns math, canonical NED/FRD state, and trajectory contracts.
`px4_offboard_controllers_lib` owns ROS-independent controller equations.
`px4_offboard_px4` owns domain commands and PX4 control-level mapping.
`px4_offboard_state_sources` owns pure source selection/conversion.
`px4_offboard_ros2_messages` is the generated-`px4_msgs` boundary when ROS is available.
`px4_offboard_f450` is vehicle-specific and is never included by generic controller/PX4 headers.
Hardware calibration lives under `experiment/calibration` for the same reason.

The dependency direction is core -> controllers/PX4/state sources -> ROS orchestration, with the
F450 and calibration implementations optional at the edge. A `.cpp` file belongs to one logical
target. The default in-process trajectory/controller/command path uses C++ values and therefore
adds no internal `/custom/*` ROS plumbing.

World vectors use NED and body vectors use FRD. Gazebo position arrives in ENU and is converted
once in `makeCanonicalGazeboPosition`; downstream controller code does not repeat frame changes.
