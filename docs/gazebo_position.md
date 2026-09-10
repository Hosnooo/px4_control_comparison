# Gazebo direct-position contract

## Purpose

Simulation uses the Gazebo model pose as the controller's primary position source. It does not route
that primary position through PX4 estimation, and it does not derive velocity by differentiating the
Gazebo position.

The transport-independent selection/conversion contract and the thin runtime source binding are now
implemented. `simulation/ros2` contains the Gazebo Transport relay, the ROS `TransformStamped`
adapter and the launch-time `ros_gz_bridge` wiring. Native ROS 2 Jazzy / Gazebo Harmonic execution
remains a validation requirement because those target packages are not installed in the host test
environment used for this milestone.

## Frozen source behavior

The frozen PX4 F450 airframe selects `PX4_GZ_WORLD=default` and `PX4_SIM_MODEL=f450`. During the
normal spawn path, `px4-rc.gzsim` names the Gazebo entity `f450_<px4_instance>` (therefore `f450_0`
for the first instance) and passes that exact name to PX4's `gz_bridge`. If `PX4_GZ_MODEL_NAME` is
set, PX4 attaches to that exact existing model name instead. The F450 SDF already comes from the pinned `ANCL/PX4-gazebo-models` submodule; this project does not duplicate or
modify that vehicle model for position feedback.

At the frozen PX4 revision, `src/modules/simulation/gz_bridge/GZBridge.cpp` subscribes to
`/world/<world>/pose/info`, receives `gz.msgs.Pose_V`, selects the entry whose `Pose.name` equals the
runtime model name, and converts position from ENU to NED as `[y, x, -z]`.

Gazebo Harmonic's `SceneBroadcaster` is the source of `/world/default/pose/info`. Its `Pose_V` carries
simulation time in the outer header while each model pose carries the entity `name` and `id`. The
standard ROS Jazzy `ros_gz_bridge` `Pose_V -> PoseArray` conversion does not preserve those Gazebo
entity names, so that mapping is not acceptable for primary position feedback.

## Runtime data flow

The required runtime path is:

```text
Gazebo /world/default/pose/info (gz.msgs.Pose_V)
  -> Gazebo relay: require exactly one Pose.name == configured_model_name
  -> copy that exact source pose to /model/<configured_model_name>/direct_pose (gz.msgs.Pose)
     with source simulation stamp, frame_id=<configured_world>, child_frame_id=<configured_model_name>
  -> ros_gz_bridge parameter_bridge, direction GZ_TO_ROS
  -> geometry_msgs/msg/TransformStamped
  -> ROS simulation-position adapter
  -> makeCanonicalGazeboPosition()
  -> control::enuToNed() exactly once
  -> CanonicalState.external_position_ned_m
```

`geometry_msgs/msg/TransformStamped` is intentional. The Jazzy `ros_gz_bridge` conversion from
`gz.msgs.Pose` preserves the Gazebo header and its `child_frame_id`. The ROS adapter can therefore
verify both the world frame and the exact configured F450 instance identity before accepting position.

The relay must copy the original selected Gazebo pose, including orientation, rather than fabricating
a transform. Orientation is transported for semantic correctness but is ignored by the canonical
position adapter; canonical attitude continues to come from PX4.

## Pure contract

`selectGazeboModelPose()` receives a source-shaped pose vector sample, world-frame name and expected
model name. It rejects invalid configuration, invalid simulation time, a missing model, duplicate
model matches and non-finite selected position. On success it returns the selected source index so a
thin Gazebo wrapper can copy the exact raw pose without searching a second time.

`makeCanonicalGazeboPosition()` accepts only the identity-stamped direct pose. It verifies the expected
world and child frames, finite non-negative simulation time and finite ENU position. It then calls the
existing `control::enuToNed()` boundary conversion once and returns a timestamped NED position.

No orientation conversion, velocity estimate, acceleration estimate or PX4 EKF position is produced by
this adapter.

## Timestamp contract

The source timestamp is Gazebo simulation time from the `Pose_V` header. A zero timestamp is valid at
simulation start; negative or non-finite timestamps are rejected. Pinned PX4 already subscribes to
`/world/<world>/clock` and synchronizes its SITL clock from the Gazebo `sim` time on every callback.
The ROS launch bridges that same Gazebo clock to `/clock`; controller-side ROS nodes must set
`use_sim_time=true` so freshness checks use the same simulation-time domain. The bridge leaves
`override_timestamps_with_wall_time` at its Jazzy default `false`, so the relayed pose stamp is not
replaced by wall time.

## ROS/Gazebo runtime package

`simulation/ros2` is the ament package `px4_control_comparison_simulation`. Its dependencies are
explicit: `geometry_msgs`, `gz_msgs_vendor`, `gz_transport_vendor`, `ros_gz_bridge`, `rosgraph_msgs`,
`launch`, and `launch_ros`. The package builds a plain Gazebo Transport relay executable and a shared
message-adapter library. The launch file starts the relay with source-derived defaults
`world_name=default` and `model_name=f450_0`, then starts the installed `ros_gz_bridge`
`parameter_bridge` in Gazebo-to-ROS-only mode for both the direct pose and Gazebo clock.

The relay publishes `/model/<model_name>/direct_pose` as `gz.msgs.Pose`. The bridge exposes the same
ROS topic as `geometry_msgs/msg/TransformStamped`; `/world/<world_name>/clock` is remapped on the ROS
side to `/clock`. Future ROS controller nodes consuming this path must use simulation time so freshness
checks compare timestamps in the same clock domain.

## Runtime validation still required

The runtime package has been compile-tested with strict GCC/Clang and local interface stubs that match
the audited Jazzy/Harmonic APIs, and its message-level tests exercise identity, timestamp and ENU-to-NED
behavior. That is not a substitute for a native target build. Validation on Ubuntu 24.04 / ROS 2 Jazzy /
Gazebo Harmonic must still run `colcon build`, package tests and the actual F450 SITL path, then prove the
real topic type/rate, exact `f450_0` selection, GZ-to-ROS-only bridge direction, `/clock`, preserved
timestamp/frames and the final numeric ENU-to-NED result before runtime validation is claimed.
