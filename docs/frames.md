# Frame and sign authority

This file is the single authority for frames, rotations, signs and controller units.

## Canonical frames

Controller mathematics uses world **NED** and body **FRD** only.

`R_ned_frd` maps a vector expressed in body FRD into world NED:

`v_ned = R_ned_frd * v_frd`.

The corresponding Hamilton quaternion is written `q_ned_frd = [w,x,y,z]` and represents the same body-FRD to world-NED rotation.

Canonical units:

| Quantity | Frame | Unit |
|---|---|---|
| position | NED | m |
| velocity | NED | m/s |
| acceleration/jerk/snap | NED | m/s², m/s³, m/s⁴ |
| body angular rate | FRD | rad/s |
| body angular acceleration | FRD | rad/s² |
| desired/world force | NED | N |
| physical body moment | FRD | N·m |
| PX4 thrust command | body FRD control coordinates | dimensionless normalized |
| PX4 torque command | body FRD control coordinates | dimensionless normalized |

## Boundary conversions

ROS/Vicon convention is treated as ENU world with FLU body. The fixed basis-change matrices are

`C_ned_enu = [[0,1,0],[1,0,0],[0,0,-1]]`

`C_frd_flu = diag(1,-1,-1)`.

Vectors:

`p_ned = C_ned_enu * p_enu`

`v_frd = C_frd_flu * v_flu`.

A rotation mapping FLU body vectors into ENU world converts as

`R_ned_frd = C_ned_enu * R_enu_flu * C_frd_flu`.

The inverse conversion uses the transposes; both matrices are involutions, so their transpose equals their inverse.

## Basis-vector checks

World conversion:

- ENU +X East -> NED +Y East.
- ENU +Y North -> NED +X North.
- ENU +Z Up -> NED -Z Down.

Body conversion:

- FLU +X Front -> FRD +X Front.
- FLU +Y Left -> FRD -Y Right.
- FLU +Z Up -> FRD -Z Down.

## Thrust sign

PX4 multicopter collective thrust is normally negative body-Z in FRD. A level hover therefore has a normalized setpoint approximately `[0, 0, -hover_thrust]` after PX4's acceleration/thrust mapping.

Physical `collective_thrust_n` in `LeeController` is a positive scalar magnitude along body `-Z`. It is never confused with the normalized PX4 `thrust_body[2]` coordinate.
