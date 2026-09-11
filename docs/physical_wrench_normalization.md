# Physical wrench normalization

`lee_wrench` produces physical collective thrust in newtons and FRD moments in N*m. Those values
are not PX4 normalized commands. A normalization provider must reconstruct the requested physical
wrench and return an explicit success result before a `NormalizedWrenchCommand` may be created.
Failure is closed: no torque/thrust message is produced.

The F450 simulation model is isolated under `vehicles/f450`. It preserves separate audited PX4
allocator geometry and Gazebo physical geometry, the quadratic motor thrust law, ESC speed range,
yaw moment ratios, allocator bounds, Newton moment solve, condition check, backtracking, and force
and moment residual tolerances.

Measured hardware calibration remains under `experiment/calibration`. Records identify the
vehicle and source-data digest, contain fit-quality acceptance limits, have a bounded age and
operating range, and must pass full-wrench reconstruction after affine inversion. A simulation
model is never silently substituted for measured hardware authority.
