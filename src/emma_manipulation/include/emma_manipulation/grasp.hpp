#pragma once

/// @file
/// Top-down grasp poses for a box-shaped block. Pure geometry, no ROS.

#include <span>
#include <vector>

#include <Eigen/Core>

#include "emma_manipulation/constants.hpp"

namespace emma_manipulation {

/// @brief TCP poses (4x4, in the frame `block_xyz` is given in) that grasp a block from above,
/// best first.
///
/// The fingers close along the block axis most perpendicular to the line from the arm base to the
/// block, so the arm reaches across the block rather than along it. The approach axis (TCP z)
/// points down, tilted by each angle in `tilts` away from the arm base. Each tilt comes in two
/// variants with the fingers swapped (TCP rotated 180 deg about z), which the parallel gripper
/// treats as the same grasp but the wrist does not.
[[nodiscard]] std::vector<Eigen::Matrix4d> graspCandidates(
    const Eigen::Vector3d& block_xyz, double block_yaw, const Eigen::Vector2d& arm_base_xy,
    std::span<const double> tilts = kGraspTilts);

/// @brief `tform` moved back by `dist` along its own -z (away from the approach).
[[nodiscard]] Eigen::Matrix4d offset(const Eigen::Matrix4d& tform, double dist);

}  // namespace emma_manipulation
