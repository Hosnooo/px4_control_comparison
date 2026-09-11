#pragma once
#include "px4_offboard_controllers/core/math.hpp"
#include <array>
#include <string>
namespace px4_offboard::vehicles::f450 {
struct F450WrenchConfig { std::array<Vec3,4> allocator_rotor_position_frd_m{}; std::array<double,4> allocator_thrust_coefficient{}; std::array<double,4> allocator_yaw_moment_ratio{}; std::array<Vec3,4> physical_rotor_position_frd_m{}; std::array<double,4> physical_yaw_moment_ratio{}; double motor_thrust_constant_n_per_radps2{}; double esc_speed_min_radps{}; double esc_speed_max_radps{}; double collective_force_tolerance_n{}; double moment_tolerance_nm{}; double jacobian_condition_limit{}; int max_iterations{}; };
struct F450ForwardState { bool ok{}; std::string reason; Vec3 normalized_torque_frd{}; double normalized_collective_thrust{}; std::array<double,4> motor_command{}; std::array<double,4> rotor_speed_radps{}; std::array<double,4> rotor_thrust_n{}; double collective_thrust_n{}; Vec3 body_moment_frd_nm{}; };
enum class F450WrenchStatus { success, invalid_input, singular_jacobian, infeasible, no_convergence };
struct F450WrenchResult { bool ok{}; F450WrenchStatus status{F450WrenchStatus::invalid_input}; std::string reason; Vec3 normalized_torque_frd{}; Vec3 normalized_thrust_body_frd{}; std::array<double,4> motor_command{}; std::array<double,4> rotor_speed_radps{}; std::array<double,4> rotor_thrust_n{}; double reconstructed_collective_thrust_n{}; Vec3 reconstructed_body_moment_frd_nm{}; double collective_force_residual_n{}; int iterations{}; };
F450WrenchConfig frozenF450WrenchConfig();
class F450WrenchModel { public: explicit F450WrenchModel(F450WrenchConfig config); F450ForwardState forward(const Vec3&,double) const; F450WrenchResult normalize(double,const Vec3&,double) const; private: F450WrenchConfig config_; std::array<std::array<double,4>,4> normalized_mixer_{}; };
}  // namespace px4_offboard::vehicles::f450
