# Sources and provenance

The refactor starts from baseline commit `986eceb5b39ec6d1ae046e51882ccbb5df922a36` and the audited
external pins in `docs/dependencies.md`. Controller equations were moved and renamed rather than
re-derived where behavior was already audited.

Lee force/moment, desired-attitude derivative, attitude-error, and angular feed-forward behavior
comes from the baseline Lee implementation and its cited Lee SE(3) equations. The PX4
attitude/rate and thrust-normalization mirrors preserve the behavior audited against the pinned
PX4 revision, including PX4-specific constants, limits and sign conventions. The F450 allocator
and physical model preserve the baseline PX4 allocator geometry and Gazebo physical model as two
distinct authorities. Gazebo direct position preserves the baseline identity/timestamp selection
semantics.

## F450 output-range provenance

The simulation wrench model keeps the PX4 output range and the Gazebo motor-model limit as
separate authorities. At the frozen PX4 revision,
`ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_f450` configures the simulation ESC minimum
to `150` and maximum to `1000`. PX4 `GZMixingInterfaceESC` publishes those mixed output integers
directly on the Gazebo motor-speed command topic. The pinned F450 SDF independently sets
`maxRotVelocity = 1032 rad/s`; that is a Gazebo motor-plugin hard limit, not the selected PX4
normalized-output maximum. The physical model therefore uses the audited `150..1000 rad/s`
command mapping while retaining `1032 rad/s` only as source provenance for the simulator limit.

The original project documentation referenced an SLS embedded PX4 fork that is not present as an
audited gitlink in this repository and is not accessible from this environment. This refactor does
not infer, vendor, or claim verification against inaccessible fork contents.
