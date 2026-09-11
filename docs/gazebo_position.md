# Gazebo direct position

Gazebo publishes a world `Pose_V` containing many entities. The retained identity relay selects
exactly one requested `model_name`, preserves the Gazebo simulation timestamp, and republishes a
single pose carrying explicit world/model identity. Stock `ros_gz_bridge` then converts that pose
to a standard ROS geometry message in the Gazebo-to-ROS direction.

The pure state-source adapter validates `world_name`, `model_name`, timestamp, uniqueness, and
finite position. Only then does it convert position from Gazebo ENU to canonical NED:
`(x_n, y_e, z_d) = (y_enu, x_enu, -z_enu)`. This is the only ENU-to-NED conversion on the direct
position path.

The launch file requires both names and contains no `f450_0` or other hidden vehicle default.
Native Gazebo Harmonic execution must be validated on a host with the vendor packages installed.
