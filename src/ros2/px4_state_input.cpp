#include "runtime.hpp"

namespace px4_offboard::ros2_runtime {
namespace {

inline constexpr const char *kLocalPositionTopic = "/fmu/out/vehicle_local_position";
inline constexpr const char *kAttitudeTopic = "/fmu/out/vehicle_attitude";
inline constexpr const char *kAngularVelocityTopic = "/fmu/out/vehicle_angular_velocity";
inline constexpr const char *kVehicleStatusTopic = "/fmu/out/vehicle_status";

std::uint64_t sampleTimestamp(std::uint64_t timestamp_sample, std::uint64_t timestamp) {
  return timestamp_sample != 0 ? timestamp_sample : timestamp;
}

double secondsFromMicroseconds(std::uint64_t timestamp_us) {
  return 1e-6 * static_cast<double>(timestamp_us);
}

}  // namespace

Px4StateInput::Px4StateInput(rclcpp::Node &node, StateRequirements requirements) {
  const auto sensor_qos = rclcpp::SensorDataQoS();

  if (requirements.position || requirements.velocity) {
    local_position_sub_ = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        kLocalPositionTopic, sensor_qos,
        [this, requirements](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          const double timestamp_s = secondsFromMicroseconds(
              sampleTimestamp(message->timestamp_sample, message->timestamp));

          if (requirements.position) {
            const Vec3 position{message->x, message->y, message->z};
            if (message->xy_valid && message->z_valid && position.finite()) {
              state_.position_ned = TimedValue<Vec3>{position, timestamp_s};
            } else {
              state_.position_ned.reset();
            }
          }

          if (requirements.velocity) {
            const Vec3 velocity{message->vx, message->vy, message->vz};
            if (message->v_xy_valid && message->v_z_valid && velocity.finite()) {
              state_.velocity_ned = TimedValue<Vec3>{velocity, timestamp_s};
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
          const double timestamp_s = secondsFromMicroseconds(
              sampleTimestamp(message->timestamp_sample, message->timestamp));
          const Quat attitude{message->q[0], message->q[1], message->q[2], message->q[3]};
          if (attitude.finite() && attitude.squaredNorm() > kEps) {
            state_.attitude_ned_frd = TimedValue<Quat>{attitude.normalized(), timestamp_s};
          } else {
            state_.attitude_ned_frd.reset();
          }
        });
  }

  if (requirements.body_rate || requirements.body_angular_acceleration) {
    angular_velocity_sub_ = node.create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        kAngularVelocityTopic, sensor_qos,
        [this, requirements](const px4_msgs::msg::VehicleAngularVelocity::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          const double timestamp_s = secondsFromMicroseconds(
              sampleTimestamp(message->timestamp_sample, message->timestamp));

          if (requirements.body_rate) {
            const Vec3 body_rate{message->xyz[0], message->xyz[1], message->xyz[2]};
            if (body_rate.finite()) {
              state_.body_rate_frd = TimedValue<Vec3>{body_rate, timestamp_s};
            } else {
              state_.body_rate_frd.reset();
            }
          }

          if (requirements.body_angular_acceleration) {
            const Vec3 angular_acceleration{message->xyz_derivative[0],
                                            message->xyz_derivative[1],
                                            message->xyz_derivative[2]};
            if (angular_acceleration.finite()) {
              state_.body_angular_accel_frd =
                  TimedValue<Vec3>{angular_acceleration, timestamp_s};
            } else {
              state_.body_angular_accel_frd.reset();
            }
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
