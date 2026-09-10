# Measured hardware wrench calibration

Hardware `lee_wrench` requires a measured record; simulator constants are never a fallback. Collect
static wrench-stand samples as CSV with this exact header:

```text
normalized_thrust,torque_x,torque_y,torque_z,collective_thrust_n,moment_x_nm,moment_y_nm,moment_z_nm
```

All normalized inputs are PX4 body-FRD control coordinates. Collective thrust is positive in
newtons; moments are body-FRD newton-metres. Samples must span all four inputs and remain inside
the intended operating envelope.

Run `fit_wrench_calibration.py --help` for the required provenance and acceptance arguments. The
utility fits a 4-by-5 affine map, checks sample count, rank, conditioning, RMSE, and maximum
residual, then writes the strict `key=value` record accepted by `HardwareWrenchCalibration`.
Acceptance limits are required at fitting time and again at runtime so a copied or edited record
cannot bypass the hardware gate.

Raw measurements are intentionally ignored by Git under `experiment/calibration/data/*.csv`.
Keep the source data in the lab's controlled evidence store and record its SHA-256 in the output.
