#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>

// rcl_clock_type_t and RCL_SYSTEM_TIME are provided by rcl in the native ROS 2 headers.
enum rcl_clock_type_t { RCL_SYSTEM_TIME };

namespace rclcpp {

class SensorDataQoS {};
class Logger {};

class Time {
 public:
  std::int64_t nanoseconds() const { return 1; }
};

class Clock {
 public:
  explicit Clock(rcl_clock_type_t) {}
  Time now() const { return {}; }
};

class WallTimer {
 public:
  using SharedPtr = std::shared_ptr<WallTimer>;
};

template <class MessageT>
class Subscription {
 public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;
};

template <class MessageT>
class Publisher {
 public:
  using SharedPtr = std::shared_ptr<Publisher<MessageT>>;
  void publish(const MessageT &) {}
};

class Node {
 public:
  explicit Node(const char *) {}

  template <class ParameterT>
  ParameterT declare_parameter(const char *, const ParameterT &default_value) {
    return default_value;
  }

  Logger get_logger() const { return {}; }

  template <class MessageT, class QoST, class CallbackT>
  typename Subscription<MessageT>::SharedPtr create_subscription(const char *, const QoST &,
                                                                  CallbackT &&) {
    return std::make_shared<Subscription<MessageT>>();
  }

  template <class MessageT, class QoST>
  typename Publisher<MessageT>::SharedPtr create_publisher(const char *, const QoST &) {
    return std::make_shared<Publisher<MessageT>>();
  }

  template <class Rep, class Period, class CallbackT>
  WallTimer::SharedPtr create_wall_timer(std::chrono::duration<Rep, Period>, CallbackT &&) {
    return std::make_shared<WallTimer>();
  }
};

inline void init(int, char **) {}
inline void shutdown() {}

inline void spin(const std::shared_ptr<Node> &) {}

}  // namespace rclcpp

#define RCLCPP_FATAL(logger, ...) \
  do {                            \
    (void)(logger);               \
  } while (false)
