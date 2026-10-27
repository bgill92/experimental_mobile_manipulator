#pragma once

/// @file
/// Timing of joint paths for arm_controller.

#include <vector>

#include <Eigen/Core>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace emma_manipulation {

/// @brief Times a joint path (kArmJoints order) for arm_controller with a per-segment max joint
/// speed.
///
/// The first waypoint is the start state and is dropped. Positions only; the controller
/// interpolates. Each segment takes `max(max|dq| / v_max, min_segment_time)`. Proper time
/// parameterisation (TOPPRA) needs a joint limits YAML and is deferred.
[[nodiscard]] trajectory_msgs::msg::JointTrajectory toJointTrajectory(
    const std::vector<Eigen::VectorXd>& path, double v_max = 0.8,
    double min_segment_time = 0.1);

}  // namespace emma_manipulation
