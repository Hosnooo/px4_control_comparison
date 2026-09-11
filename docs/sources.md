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

The original project documentation referenced an SLS embedded PX4 fork that is not present as an
audited gitlink in this repository and is not accessible from this environment. This refactor does
not infer, vendor, or claim verification against inaccessible fork contents.
