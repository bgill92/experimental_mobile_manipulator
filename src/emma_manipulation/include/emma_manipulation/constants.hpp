#pragma once

/// @file
/// Shared constants for emma's manipulation, all in metres / radians and `base_link`. Values
/// tuned on the robot (home pose, table) are in config/manipulation.yaml instead; see config.hpp.

#include <array>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>

namespace emma_manipulation {

/// Same order as arm_controller's joints in emma_simulation/config/controllers.yaml. A vector of
/// strings because that is what roboplan's joint-name APIs take.
inline const std::vector<std::string> kArmJoints = {
    "joint2_to_joint1", "joint3_to_joint2",      "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
};

/// Wrist camera looking at kBlockStart from 0.2 m, its optical axis 65 deg below horizontal (at
/// 45 deg the camera would sit almost over the arm base, out of reach). kPlaceXyz and a block a
/// few cm off kBlockStart stay in view. Found by IK in test_planner.cpp (LookPose), which
/// re-checks it.
inline const Eigen::VectorXd kLookQ =
    (Eigen::VectorXd(6) << 0.862, 0.771, -1.533, -0.695, -0.133, 0.855).finished();

inline constexpr std::string_view kBaseFrame = "base_link";
inline constexpr std::string_view kCameraFrame = "wrist_camera_color_optical_frame";
inline constexpr std::string_view kTcpFrame = "tcp";

inline constexpr double kBlockSize = 0.010;
inline const Eigen::Vector3d kBlockStart{0.25, 0.0, 0.155};
inline const Eigen::Vector3d kPlaceXyz{0.25, 0.08, 0.155};

/// Distance backed off along the approach axis for pregrasp, retreat and preplace.
inline constexpr double kPreOffset = 0.05;

/// Grasp tilts away from straight down, leaning the approach away from the robot.
inline constexpr std::array<double, 3> kGraspTilts = {
    0.0, 20.0 * std::numbers::pi / 180.0, 35.0 * std::numbers::pi / 180.0};

inline constexpr double kGripperOpen = 0.0;
inline constexpr double kGripperClosed = -0.007;

}  // namespace emma_manipulation
