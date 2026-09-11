#include "runtime.hpp"

#include <algorithm>
#include <memory>

namespace px4_offboard::ros2_runtime {
namespace {

inline constexpr const char *kLocalPositionTopic = "/fmu/out/vehicle_local_position";
inline constexpr const char *kAttitudeTopic = "/fmu/out/vehicle_attitude";
inline constexpr const char *kAngularVelocityTopic = "/fmu/out/vehicle_angular_velocity";
inline constexpr const char *kVehicleStatusTopic = "/fmu/out/vehicle_status";

}  // namespace

Px4StateInput::Px4StateInput(rclcpp::Node &node, StateRequirements requirements) {
  const auto sensor_qos = rclcpp::SensorDataQoS();

  if (requirements.position || requirements.velocity) {
    local_position_sub_ = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        kLocalPositionTopic, sensor_qos,
        [this, requirements](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          const std::uint64_t timestamp =
              message->timestamp_sample != 0 ? message->timestamp_sample : message->timestamp;
          state_.timestamp_us = std::max(state_.timestamp_us, timestamp);

          if (requirements.position) {
            if (message->xy_valid && message->z_valid) {
              state_.position_ned = Vec3{message->x, message->y, message->z};
            } else {
              state_.position_ned.reset();
            }
          }

          if (requirements.velocity) {
            if (message->v_xy_valid && message->v_z_valid) {
              state_.velocity_ned = Vec3{message->vx, message->vy, message->vz};
            } else {
              state_.velocity_ned.reset();
            }
          }
        });
  }

  if (requirements.attitude) {
    attitude_sub_ = node.create_subscription<px4_msgs::msg::VehicleAttitude>(
        kAttitudeTopic, sensor_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          const std::uint64_t timestamp =
              message->timestamp_sample != 0 ? message->timestamp_sample : message->timestamp;
          state_.timestamp_us = std::max(state_.timestamp_us, timestamp);
          const Quat attitude{message->q[0], message->q[1], message->q[2], message->q[3]};
          if (attitude.finite() && attitude.squaredNorm() > kEps) {
            state_.attitude_ned_frd = attitude.normalized();
          } else {
            state_.attitude_ned_frd.reset();
          }
        });
  }

  if (requirements.body_rate) {
    angular_velocity_sub_ = node.create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        kAngularVelocityTopic, sensor_qos,
        [this](const px4_msgs::msg::VehicleAngularVelocity::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          const std::uint64_t timestamp =
              message->timestamp_sample != 0 ? message->timestamp_sample : message->timestamp;
          state_.timestamp_us = std::max(state_.timestamp_us, timestamp);
          const Vec3 body_rate{message->xyz[0], message->xyz[1], message->xyz[2]};
          if (body_rate.finite()) {
            state_.body_rate_frd = body_rate;
          } else {
            state_.body_rate_frd.reset();
          }
        });
  }

  vehicle_status_sub_ = node.create_subscription<px4_msgs::msg::VehicleStatus>(
      kVehicleStatusTopic, sensor_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        in_offboard_mode_ =
            message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
      });
}

CanonicalState Px4StateInput::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool Px4StateInput::armed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return armed_;
}

bool Px4StateInput::inOffboardMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return in_offboard_mode_;
}

}  // namespace px4_offboard::ros2_runtime
