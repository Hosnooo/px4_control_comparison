# Validation

The required local gate is: GCC configure/build/CTest with warnings as errors, Clang
configure/build/CTest, ASan+UBSan host tests, Python architecture/launch/calibration checks, format
verification where `clang-format` is installed, dependency/topic/pin checks, and a final diff
inspection.

A host-only Debian run exercises ROS-independent math/controller behavior and compiles the
`px4_msgs` conversion against audited field-shape stubs. It does **not** prove ROS 2 Jazzy ABI,
Gazebo Harmonic transport, DDS behavior, PX4 SITL dynamics, or flight safety. Those remain
runtime-pending until run in the corresponding environment.
