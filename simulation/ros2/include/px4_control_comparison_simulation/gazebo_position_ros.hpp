#pragma once

#include "simulation/gazebo_position.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string_view>

namespace simulation::runtime {

GazeboPositionResult makeCanonicalPositionFromRosTransform(
    const geometry_msgs::msg::TransformStamped &transform,
    std::string_view expected_world_frame, std::string_view expected_model_name);

}  // namespace simulation::runtime
