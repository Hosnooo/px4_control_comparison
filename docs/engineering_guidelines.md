# Engineering guidelines

This project is intended to be read and debugged during controller development and flight testing.
Clarity at control boundaries is more important than abstraction.

## Code structure

- Give each file and class one clear responsibility.
- Keep controller mathematics independent of ROS 2 and PX4 message types.
- Keep message callbacks focused on validation and translation; do not bury control equations in callbacks.
- Avoid generic manager, factory, backend, plugin, or `utils` layers unless a concrete need appears.
- Avoid unnecessary inheritance and premature generic abstractions.
- Keep frame conversions centralized and tested rather than repeated at call sites.
- Use meaningful structures and enums, and keep functions small enough to inspect directly.
- Use the repository `.clang-format` configuration consistently.

## Names, units, and frames

Important interfaces make units and frames explicit in names. Examples include
`position_ned_m`, `body_rate_frd_radps`, `body_moment_frd_nm`, and
`normalized_thrust_body_frd`.

Do not hide sign changes or unit conversions. Physical thrust and moment use SI units; PX4 thrust and
torque setpoints are dimensionless normalized control coordinates. NED/FRD is the canonical control
frame; ENU/FLU conversion occurs only at the corresponding boundary adapter.

## Comments

Comments explain information that is not obvious from the C++ itself:

- frame, unit, or sign conventions;
- source equations and source-derived PX4 behavior;
- why a non-obvious implementation decision is necessary;
- safety or validity constraints.

Do not narrate ordinary assignments, loops, or control flow. When reproducing an upstream algorithm,
identify the exact source in `docs/sources.md` and use a concise nearby comment only where that
provenance helps the reader understand the implementation.

## Configuration and safety

Do not embed plausible-looking vehicle parameters or gains as executable defaults. Controller and
vehicle parameters must come from explicit configuration, a pinned simulation model, or measured
hardware calibration as appropriate.

Simulation-only motor and airframe constants never authorize hardware wrench control. Uninitialized
state/reference timestamps are invalid, stale data blocks controller output, and hardware validation is
never claimed without hardware evidence.
