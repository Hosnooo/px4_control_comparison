# Original control-boundary comparison mapping

The original experiment compared four deeper offboard boundaries. Their preserved behavior maps to
the redesigned package as follows:

| Original comparison boundary | Redesigned selection/component |
| --- | --- |
| attitude handoff | `geometric_attitude` -> native `VehicleAttitudeSetpoint` |
| rate handoff | `geometric_rate` -> native `VehicleRatesSetpoint` |
| PX4 attitude/rate mirror | `px4_attitude_rate_mirror` -> normalized torque/thrust |
| Lee physical wrench | `lee_wrench` -> explicit normalization -> normalized torque/thrust |

The redesign additionally makes PX4 position, PX4 velocity, and geometric acceleration first-class
public boundaries. Research mapping is documentation only; the runtime no longer exposes the old
comparison `HandoffMode` or project-specific ROS message schema.
