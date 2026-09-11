# Engineering guidelines

Keep controller math deterministic and ROS-independent. Use NED for world vectors and FRD for body
vectors; perform frame conversion only at named boundaries. Make units visible in public names or
contracts. Prefer typed command values over duplicate message-shaped structs.

A behavior change starts with a failing test. Preserve pinned-source behavior unless an approved
design explicitly changes it. Fail closed on non-finite state, invalid timestamps, ambiguous
Gazebo identity, failed physical-wrench reconstruction, and out-of-range calibration.

Generic core/controllers/PX4 headers must not include F450, calibration, Gazebo, or Vicon
implementation headers. Airframe-specific constants stay under `vehicles/`. Do not add default
`/custom/*` controller plumbing topics. Keep comments for equations, frame/sign boundaries,
normalization/saturation, timestamp ownership, source provenance, and fail-closed decisions rather
than narrating obvious assignments.
