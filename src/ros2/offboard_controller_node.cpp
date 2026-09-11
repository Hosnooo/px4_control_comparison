#include "px4_offboard_controllers/core/state.hpp"
#include "px4_offboard_controllers/px4/control_level.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace px4_offboard::ros2_runtime {

inline constexpr std::array<std::string_view, 7> kControllerNames{
    "px4_position",          "px4_velocity", "geometric_acceleration",
    "geometric_attitude",    "geometric_rate", "px4_attitude_rate_mirror",
    "lee_wrench"};

inline constexpr const char *kOffboardModeTopic = "/fmu/in/offboard_control_mode";
inline constexpr const char *kTrajectoryTopic = "/fmu/in/trajectory_setpoint";
inline constexpr const char *kAttitudeSetpointTopic = "/fmu/in/vehicle_attitude_setpoint";
inline constexpr const char *kRatesSetpointTopic = "/fmu/in/vehicle_rates_setpoint";
inline constexpr const char *kThrustSetpointTopic = "/fmu/in/vehicle_thrust_setpoint";
inline constexpr const char *kTorqueSetpointTopic = "/fmu/in/vehicle_torque_setpoint";
inline constexpr const char *kVehicleCommandTopic = "/fmu/in/vehicle_command";
inline constexpr const char *kLocalPositionTopic = "/fmu/out/vehicle_local_position";
inline constexpr const char *kAttitudeTopic = "/fmu/out/vehicle_attitude";
inline constexpr const char *kAngularVelocityTopic = "/fmu/out/vehicle_angular_velocity";
inline constexpr const char *kVehicleStatusTopic = "/fmu/out/vehicle_status";

std::optional<ControllerKind> controllerKindFor(std::string_view name) {
  if (name == "px4_position") return ControllerKind::Px4Position;
  if (name == "px4_velocity") return ControllerKind::Px4Velocity;
  if (name == "geometric_acceleration") return ControllerKind::GeometricAcceleration;
  if (name == "geometric_attitude") return ControllerKind::GeometricAttitude;
  if (name == "geometric_rate") return ControllerKind::GeometricRate;
  if (name == "px4_attitude_rate_mirror") return ControllerKind::Px4AttitudeRateMirror;
  if (name == "lee_wrench") return ControllerKind::LeeWrench;
  return std::nullopt;
}

bool supportedController(std::string_view name) {
  for (const auto candidate : kControllerNames) {
    if (candidate == name) return true;
  }
  return false;
}

}  // namespace px4_offboard::ros2_runtime

#if __has_include(<rclcpp/rclcpp.hpp>)
#include <rclcpp/rclcpp.hpp>

#include "runtime.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("px4_offboard_controller");
  const std::string controller = node->declare_parameter<std::string>("controller", "");
  const auto controller_kind = px4_offboard::ros2_runtime::controllerKindFor(controller);
  if (!controller_kind.has_value()) {
    RCLCPP_FATAL(node->get_logger(), "controller must be one of the seven documented selections");
    rclcpp::shutdown();
    return 2;
  }

  px4_offboard::ros2_runtime::Px4StateInput state_input(
      *node, px4_offboard::requirementsFor(*controller_kind));
  px4_offboard::ros2_runtime::Px4CommandPublisher command_publisher(*node);
  const auto control_level = px4_offboard::controlLevelFor(*controller_kind);

  using namespace std::chrono_literals;
  // PX4's pinned uXRCE-DDS client synchronizes message timestamps against Agent OS time. Keep
  // PX4-bound command timestamps on system time even if another ROS node uses Gazebo /clock.
  const auto px4_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  const auto heartbeat_timer = node->create_wall_timer(
      100ms, [&command_publisher, px4_clock, control_level] {
    const auto now_ns = px4_clock->now().nanoseconds();
    if (now_ns > 0) {
      command_publisher.publishControlMode(
          control_level, static_cast<std::uint64_t>(now_ns / 1000));
    }
  });
  (void)state_input;
  (void)heartbeat_timer;

  // Arming and mode changes are explicit VehicleCommand actions outside controller math. The
  // selected in-process controller owns only setpoint computation and the matching PX4 control
  // level; it never exchanges project-specific ROS plumbing messages with another stage.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#else
int main() { return 0; }
#endif
