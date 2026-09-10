# Physical Wrench Normalization

## Decision

`lee_wrench` preserves the same PX4 acceleration-to-normalized-thrust mapping used by the other
three comparison modes. Physical torque normalization is conditional on that fixed normalized
thrust command. It may change normalized body torque, but it must not silently replace the common
thrust mapping with a simulator-specific collective-thrust inverse.

That common-thrust invariant does not waive physical-wrench consistency. After solving the physical
moment, the implementation forward-reconstructs the full wrench. If the fixed common PX4 thrust
command cannot reproduce the requested collective force within the configured acceptance tolerance,
`lee_wrench` fails closed for that request instead of reporting a successful physical-wrench inverse.

## Frozen simulation chain

The F450 simulation path is reconstructed from the pinned sources in `sources.md`:

1. PX4 receives normalized body-FRD torque and thrust coordinates.
2. The F450 control allocator uses rotor positions `+/-0.159 m`, moment ratios `+/-0.014`, and the
   normalized pseudo-inverse mixer from the pinned PX4 allocation implementation.
3. A candidate is accepted only when the raw allocation already lies in `[0, 1]` for every motor.
   The normalizer does not model or rely on sequential desaturation.
4. With the frozen `THR_MDL_FAC=0`, PX4 maps each non-reversible motor command `u` to the configured
   simulation ESC range: `omega = 150 + 850 u` rad/s after the arming ramp.
5. Gazebo produces rotor thrust `f_i = 1.2e-5 omega_i^2` N. Physical roll and pitch moments use the
   SDF rotor positions `+/-0.1626345596714 m`; yaw moment uses the SDF ratio `+/-0.0137`.

The physical and allocator rotor positions are deliberately separate. Motor order is PX4/Gazebo
order 0 through 3, and the SDF ENU/FLU positions are converted once to body FRD before moment
reconstruction.

## Simulation solve

Inputs are positive physical collective thrust demand in newtons, physical body-FRD moment demand
in newton-metres, and the common normalized collective-thrust magnitude from
`Px4ThrustNormalization`.

For fixed normalized thrust, the solver varies only normalized torque. It evaluates the complete
PX4-allocation-to-Gazebo forward model and solves the three physical moment residuals with a bounded
Newton iteration and backtracking. A result is accepted only when:

- every input and intermediate value is finite;
- normalized thrust is in `[0, 1]`;
- normalized torque is in `[-1, 1]` on every axis;
- every raw motor command is in `[0, 1]`, so PX4 desaturation is unnecessary;
- rotor speeds stay within the sourced ESC/model operating range;
- the physical moment residual is within the configured tolerance;
- the reconstructed collective force is within the configured force tolerance of the requested
  collective force; and
- the local moment Jacobian is nonsingular and sufficiently conditioned.

The result includes normalized torque, normalized body thrust, raw motor commands, rotor speeds,
rotor thrusts, reconstructed moment, reconstructed collective thrust, collective-force residual,
iteration count, and an explicit rejection reason. A force residual outside tolerance makes the
physical-wrench request infeasible under the fixed common-thrust mapping.

## Hardware calibration

Hardware never uses Gazebo constants or a simulation-tagged record. A measured calibration record
contains:

- schema version and `measured_hardware` authority;
- vehicle identifier, UTC calibration date, method, and source-data SHA-256;
- explicit normalized-input and physical-output units;
- valid normalized thrust and torque ranges;
- a measured affine coefficient matrix mapping
  `[1, normalized_thrust, torque_x, torque_y, torque_z]` to
  `[collective_thrust_N, moment_x_Nm, moment_y_Nm, moment_z_Nm]`;
- sample count, per-output RMSE, maximum residual, and design-matrix condition number; and
- validation limits used to accept or reject the record.

The fitter records its acceptance limits, and runtime limits supplied by the application must be
at least as strict. The affine model is intentionally modest: it is valid only over its measured
operating envelope. The fitting utility rejects insufficient, non-finite, rank-deficient, poorly
conditioned, or high-residual data. The runtime loader repeats structural/provenance/quality checks
rather than trusting a file produced elsewhere.

For a fixed common thrust command, the hardware normalizer solves the measured 3-by-3 torque block,
then reconstructs all four physical wrench outputs. The effective collective-force reconstruction
bound is the stricter of the calibration record's accepted maximum collective residual and the
runtime acceptance limit. Moment reconstruction uses the configured runtime per-axis tolerance.
Missing, malformed, simulation-tagged, singular, poor-fit, stale, wrong-vehicle, out-of-range, or
full-wrench-inconsistent calibration use is rejected. Until a real F450 record passes those checks,
hardware `lee_wrench` remains unavailable.

## Verification

Simulation tests use independently calculated golden cases from the pinned F450/PX4 chain:

- zero-moment symmetry at multiple normalized thrusts;
- positive roll, pitch, and yaw sign cases;
- simultaneous three-axis moment reconstruction;
- explicit physical-versus-allocator geometry checks;
- exact full-wrench reconstruction for accepted cases;
- collective-force mismatch rejection;
- motor/torque saturation and infeasible-request rejection; and
- deterministic repeatability and forward reconstruction.

Hardware tests cover valid synthetic calibration and rejection of missing fields, wrong authority,
wrong vehicle, invalid units, non-finite coefficients, inadequate samples, singular/ill-conditioned
fits, excessive residuals, stale records, out-of-range commands, and collective-force reconstruction
mismatch. A synthetic dataset exercises the fitter-to-loader-to-normalizer round trip. Existing
controller-core tests remain independent of this normalization policy.
