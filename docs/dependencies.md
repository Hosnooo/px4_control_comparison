# Dependencies

The audited gitlinks are retained exactly, only moved from `dependencies/` to `third_party/`:

| Repository | Pin |
| --- | --- |
| ANCL/PX4-Autopilot | `c0a1a2ecbb4c36648e12b69e7db1f29fd335f2dc` |
| ANCL/px4_msgs | `392e831c1f659429ca83902e66820d7094591410` |
| ANCL/ros2-vicon-receiver | `49a026301e0f009e0ae9b21f86bed1e7cae73f0d` |
| ANCL/Micro-XRCE-DDS-Agent | `73622810d984349b80bbac0ef55fc0b694d62222` |

Their `.gitmodules` URLs are unchanged. ROS 2 Jazzy packages (`rclcpp`, `geometry_msgs`,
`rosgraph_msgs`) and Gazebo Harmonic vendor packages are installed dependencies.
`ros_gz_bridge` is installed from ROS and is intentionally **not** a Git submodule.
