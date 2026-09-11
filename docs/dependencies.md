# Dependencies

The audited gitlinks are retained exactly, only moved from `dependencies/` to `third_party/`:

| Repository | Pin |
| --- | --- |
| ANCL/PX4-Autopilot | `c0a1a2ecbb4c36648e12b69e7db1f29fd335f2dc` |
| ANCL/px4_msgs | `392e831c1f659429ca83902e66820d7094591410` |
| ANCL/ros2-vicon-receiver | `49a026301e0f009e0ae9b21f86bed1e7cae73f0d` |
| ANCL/Micro-XRCE-DDS-Agent | `73622810d984349b80bbac0ef55fc0b694d62222` |

Their `.gitmodules` URLs and gitlink SHAs are unchanged. ROS 2 Jazzy packages (`rclcpp`,
`geometry_msgs`, `rosgraph_msgs`) and Gazebo Harmonic vendor packages are installed dependencies.
`ros_gz_bridge` is installed from ROS and is intentionally **not** a Git submodule.

## PX4 DDS observability patch

At the frozen PX4 pin, `/fmu/out/vehicle_angular_velocity` exists in
`src/modules/uxrce_dds_client/dds_topics.yaml` but its publication is commented out. The runtime
requires that native topic for body angular velocity and PX4-provided angular acceleration.
Apply `third_party/patches/px4_vehicle_angular_velocity_dds.patch` to the checked-out
`third_party/PX4-Autopilot` source before building PX4 for this project.

The patch is observability-only: it uncomments exactly that DDS publication and does not change
PX4 controller behavior. The PX4 gitlink remains pinned to the audited commit above so the source
baseline and the local delta are both explicit and reviewable.
