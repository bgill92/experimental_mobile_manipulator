#pragma once

/// @file
/// Runtime-tunable manipulation values, read from ROS parameters (config/manipulation.yaml).

#include <string>

#include <Eigen/Core>
#include <tl/expected.hpp>

namespace rclcpp {
class Node;
}  // namespace rclcpp

namespace emma_manipulation {

/// Values that change with testing on the robot; see config/manipulation.yaml for each one.
struct ArmConfig {
  Eigen::VectorXd home_q;
  Eigen::Vector3d table_center;
  Eigen::Vector3d table_size;
};

/// Declares and reads `home_q`, `table_center` and `table_size` on `node`. Every one must be set
/// (no defaults), so a missing or misspelt entry in the YAML fails here instead of planning with
/// a stale value.
[[nodiscard]] tl::expected<ArmConfig, std::string> loadArmConfig(rclcpp::Node& node);

}  // namespace emma_manipulation
