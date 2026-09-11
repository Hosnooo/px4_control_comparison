#pragma once

#include <memory>
#include <utility>

namespace rclcpp {

class SensorDataQoS {};

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
  template <class MessageT, class QoST, class CallbackT>
  typename Subscription<MessageT>::SharedPtr create_subscription(const char *, const QoST &,
                                                                  CallbackT &&) {
    return std::make_shared<Subscription<MessageT>>();
  }

  template <class MessageT, class QoST>
  typename Publisher<MessageT>::SharedPtr create_publisher(const char *, const QoST &) {
    return std::make_shared<Publisher<MessageT>>();
  }
};

}  // namespace rclcpp
