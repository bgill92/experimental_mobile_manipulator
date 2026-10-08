#include "emma_manipulation/grasp.hpp"

#include <cmath>

#include <Eigen/Geometry>

namespace emma_manipulation {

std::vector<Eigen::Matrix4d> graspCandidates(const Eigen::Vector3d& block_xyz,
                                             const double block_yaw,
                                             const Eigen::Vector2d& arm_base_xy,
                                             const std::span<const double> tilts) {
  const Eigen::Vector2d reach = block_xyz.head<2>() - arm_base_xy;
  const double norm = reach.norm();
  const Eigen::Vector3d d = norm < 1e-9 ? Eigen::Vector3d::UnitX()
                                        : Eigen::Vector3d(reach.x() / norm, reach.y() / norm, 0.0);

  const double c = std::cos(block_yaw);
  const double s = std::sin(block_yaw);
  const Eigen::Vector3d axis_a(c, s, 0.0);
  const Eigen::Vector3d axis_b(-s, c, 0.0);
  // The first axis wins a tie, as Python's min() did.
  const Eigen::Vector3d finger =
      std::abs(axis_b.dot(d)) < std::abs(axis_a.dot(d)) ? axis_b : axis_a;

  // Tilt in the plane across the fingers, so they still close on the block faces. Of the two
  // horizontal directions perpendicular to the fingers, lean toward the reach.
  Eigen::Vector3d lean = finger.cross(Eigen::Vector3d::UnitZ());
  if (lean.dot(d) < 0.0) {
    lean = -lean;
  }

  std::vector<Eigen::Matrix4d> candidates;
  candidates.reserve(2 * tilts.size());
  for (const double tilt : tilts) {
    const Eigen::Vector3d z = -std::cos(tilt) * Eigen::Vector3d::UnitZ() + std::sin(tilt) * lean;
    for (const Eigen::Vector3d& x : {finger, Eigen::Vector3d(-finger)}) {
      Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
      tform.block<3, 1>(0, 0) = x;
      tform.block<3, 1>(0, 1) = z.cross(x);
      tform.block<3, 1>(0, 2) = z;
      tform.block<3, 1>(0, 3) = block_xyz;
      candidates.push_back(tform);
    }
  }
  return candidates;
}

Eigen::Matrix4d offset(const Eigen::Matrix4d& tform, const double dist) {
  Eigen::Matrix4d out = tform;
  out.block<3, 1>(0, 3) -= dist * tform.block<3, 1>(0, 2);
  return out;
}

}  // namespace emma_manipulation
