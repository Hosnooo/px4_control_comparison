#if __has_include(<gz/msgs.hh>) && __has_include(<gz/transport.hh>)
#include "gazebo_pose_relay.hpp"
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <iostream>
#include <string>

namespace {
class GazeboPoseRelay {
 public:
  GazeboPoseRelay(std::string world, std::string model)
      : world_(std::move(world)), model_(std::move(model)),
        source_("/world/" + world_ + "/pose/info"),
        direct_("/model/" + model_ + "/direct_pose"),
        publisher_(node_.Advertise<gz::msgs::Pose>(direct_)) {}
  bool subscribe() { return node_.Subscribe(source_, &GazeboPoseRelay::onPoses, this); }
  const std::string &source() const { return source_; }
  const std::string &direct() const { return direct_; }
 private:
  void onPoses(const gz::msgs::Pose_V &poses) {
    const auto result = px4_offboard::ros2_runtime::makeGazeboDirectPose(poses, world_, model_);
    if (!result.ok) {
      std::cerr << "Gazebo direct-pose relay rejected sample: " << result.reason << '\n';
      return;
    }
    if (!publisher_.Publish(result.pose)) {
      std::cerr << "Gazebo direct-pose relay failed to publish on " << direct_ << '\n';
    }
  }
  std::string world_, model_, source_, direct_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
};
}  // namespace

// The relay is built only when Gazebo vendor packages are available. Selection semantics are tested
// in the ROS-independent state-source library; this executable preserves only model identity.
int main(int argc, char **argv) {
  if (argc != 3 || std::string(argv[1]).empty() || std::string(argv[2]).empty()) {
    std::cerr << "usage: gazebo_pose_relay <world_name> <model_name>\n";
    return 2;
  }
  GazeboPoseRelay relay(argv[1], argv[2]);
  if (!relay.subscribe()) return 1;
  std::cout << "Relaying " << relay.source() << " -> " << relay.direct() << '\n';
  gz::transport::waitForShutdown();
  return 0;
}
#else
int main() { return 2; }
#endif
