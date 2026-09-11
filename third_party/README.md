# Third-party sources

The four audited Git submodules listed in `docs/dependencies.md` retain their original repository
URLs and exact commit SHAs; only their paths moved from `dependencies/` to `third_party/`.
`ros_gz_bridge` is installed through ROS and must not be added here as a submodule.

`patches/` contains explicit project-required deltas applied on top of frozen gitlinks. The current
PX4 patch only enables the already-defined `VehicleAngularVelocity` DDS publication required by
the native state interface; it changes observability, not PX4 control behavior.
