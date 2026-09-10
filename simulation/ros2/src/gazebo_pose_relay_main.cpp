#include "px4_control_comparison_simulation/gazebo_pose_relay.hpp"

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <iostream>
#include <string>
#include <utility>

namespace {

class GazeboPoseRelay {
 public:
  GazeboPoseRelay(std::string world_name, std::string model_name)
      : world_name_(std::move(world_name)),
        model_name_(std::move(model_name)),
        source_topic_("/world/" + world_name_ + "/pose/info"),
        direct_topic_("/model/" + model_name_ + "/direct_pose"),
        publisher_(node_.Advertise<gz::msgs::Pose>(direct_topic_)) {}

  bool subscribe() {
    return node_.Subscribe(source_topic_, &GazeboPoseRelay::onPoseVector, this);
  }

  const std::string &sourceTopic() const { return source_topic_; }
  const std::string &directTopic() const { return direct_topic_; }

 private:
  void onPoseVector(const gz::msgs::Pose_V &poses) {
    const auto result =
        simulation::runtime::makeGazeboDirectPose(poses, world_name_, model_name_);
    if (!result.ok) {
      std::cerr << "Gazebo direct-pose relay rejected sample: " << result.reason << '\n';
      return;
    }
    if (!publisher_.Publish(result.pose)) {
      std::cerr << "Gazebo direct-pose relay failed to publish on " << direct_topic_ << '\n';
    }
  }

  std::string world_name_;
  std::string model_name_;
  std::string source_topic_;
  std::string direct_topic_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
};

}  // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: gazebo_pose_relay <world_name> <model_name>\n";
    return 2;
  }

  const std::string world_name{argv[1]};
  const std::string model_name{argv[2]};
  if (world_name.empty() || model_name.empty()) {
    std::cerr << "world_name and model_name must be non-empty\n";
    return 2;
  }

  GazeboPoseRelay relay{world_name, model_name};
  if (!relay.subscribe()) {
    std::cerr << "failed to subscribe to " << relay.sourceTopic() << '\n';
    return 1;
  }

  std::cout << "Relaying " << relay.sourceTopic() << " -> " << relay.directTopic() << '\n';
  gz::transport::waitForShutdown();
  return 0;
}
