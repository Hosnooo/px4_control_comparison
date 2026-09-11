# px4_offboard_controllers

`px4_offboard_controllers` is a ROS 2 package for comparing multirotor control boundaries while
keeping controller math independent of ROS, PX4 messages, Gazebo, and airframe-specific models.
The canonical controller frames are world NED and body FRD.

## Controller selections

| Selection | Controller-owned work | Required state | PX4 command boundary | PX4-owned lower loops |
| --- | --- | --- | --- | --- |
| `px4_position` | position reference only | none | `TrajectorySetpoint.position` | position and below |
| `px4_velocity` | velocity reference only | none | `TrajectorySetpoint.velocity` | velocity and below |
| `geometric_acceleration` | position/velocity feedback | position, velocity | `TrajectorySetpoint.acceleration` | acceleration normalization and below |
| `geometric_attitude` | geometric outer loop to attitude | position, velocity, attitude | `VehicleAttitudeSetpoint` | attitude/rate/allocator |
| `geometric_rate` | geometric outer loop and attitude error | position, velocity, attitude | `VehicleRatesSetpoint` | rate/allocator |
| `px4_attitude_rate_mirror` | frozen PX4 attitude/rate mirror | attitude, body rate, body angular acceleration | normalized torque/thrust | allocator |
| `lee_wrench` | Lee SE(3) force and moment | position, velocity, attitude, body rate | normalized torque/thrust | allocator |

All PX4 traffic uses native `/fmu/in/*` and `/fmu/out/*` topics. Internal default controller stages
communicate as C++ values, not project-owned ROS topics.

## Host build

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s test -p 'test_*.py' -v
```

A ROS 2 Jazzy workspace with the dependencies in `package.xml` enables the ament/native PX4
runtime. `controller` is a required launch argument; there is no F450 controller default.
Gazebo direct position additionally requires explicit `world_name` and `model_name`.

See `docs/architecture.md`, `docs/controllers.md`, `docs/px4_interface.md`, and
`docs/validation.md` before flight or SITL use. Host tests do not constitute native Jazzy,
Gazebo Harmonic, or PX4 SITL validation.
