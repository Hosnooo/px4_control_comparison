# Controllers

`GeometricAccelerationController` implements
`a_cmd = a_d - Kp (p-p_d) - Kv (v-v_d)` in world NED. It deliberately adds no gravity term:
PX4's acceleration-to-thrust path owns gravity compensation below this boundary.

`GeometricRateController` preserves the audited SO(3) attitude-error expression
`0.5 vee(Rd^T R - R^T Rd)` and the desired-rate feed-forward transform into the current FRD body
frame before axis rate limiting.

`Px4ThrustNormalization` mirrors the frozen PX4 multicopter acceleration-control thrust
normalization, including the PX4 one-g constant, tilt limit, minimum-thrust floor, horizontal
margin, and combined thrust saturation.

`Px4AttitudeRateController` mirrors the audited PX4 attitude/rate behavior used by the original
comparison: reduced-attitude/yaw weighting, rate limiting, rate PID/FF, integration inhibition,
dt clamping, yaw torque filtering, and optional battery scaling. Its rate stage consumes PX4's
measured FRD body angular acceleration for the preserved D term.

`LeeController` implements the audited Lee SE(3) force/moment equations in NED/FRD, including
analytic desired-attitude derivatives and feed-forward angular acceleration. Its output is a
physical collective thrust and FRD body moment. It cannot be published until a physical-wrench
normalizer succeeds.

Each public mode validates only the state/reference fields it consumes. Position and velocity PX4
modes require no controller-state feedback in this package; Lee requires the complete state and
reference derivative set.
