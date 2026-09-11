# Validation

The required local gate is: GCC configure/build/CTest with warnings as errors, Clang
configure/build/CTest, ASan+UBSan host tests, Python architecture/launch/calibration checks, format
verification where `clang-format` is installed, dependency/topic/pin checks, and a final diff
inspection.

A host-only Debian run exercises ROS-independent math/controller behavior and compiles the
`px4_msgs` conversion against audited field-shape stubs. It does **not** prove ROS 2 Jazzy ABI,
Gazebo Harmonic transport, DDS behavior, PX4 SITL dynamics, or flight safety. Those remain
runtime-pending until run in the corresponding environment.

Native PX4 validation additionally requires applying the checked-in, observability-only DDS patch
that enables `/fmu/out/vehicle_angular_velocity` at the frozen PX4 gitlink. The host tests verify
the patch scope but do not claim that a patched PX4 SITL image was built or run here.

## Runtime integration status

The checked-in native boundary does not yet claim closed-loop execution of the controller math.
The implementation-plan checkpoint references an approved parameter/reference specification at
`docs/superpowers/specs/2026-09-10-px4-offboard-controllers-redesign.md`, but that file is absent
from the checkpoint and from both repository branches. The refactor therefore does not invent
controller gains, trajectory-source semantics, PX4-mirror auxiliary inputs, or a Lee wrench
normalizer selection. The boundary fails short of claiming those semantics until the approved
contract is recovered.
