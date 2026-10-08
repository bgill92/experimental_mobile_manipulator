// Grasp pose geometry, no robot model needed.

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <gtest/gtest.h>

#include "emma_manipulation/grasp.hpp"

namespace emma_manipulation {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

const Eigen::Vector3d kBlock{0.25, 0.0, 0.155};
const Eigen::Vector2d kArmBase{0.1, 0.0};

TEST(GraspTest, CandidatesAreRotationsAtTheBlock) {
  const std::array<double, 3> tilts = {0.0, 20.0 * kDeg, 35.0 * kDeg};
  for (const double yaw : {0.0, 0.3, std::numbers::pi / 2.0, 2.0}) {
    SCOPED_TRACE(yaw);
    const std::vector<Eigen::Matrix4d> cands = graspCandidates(kBlock, yaw, kArmBase, tilts);
    ASSERT_EQ(cands.size(), 2 * tilts.size());
    for (const Eigen::Matrix4d& tform : cands) {
      const Eigen::Matrix3d rot = tform.topLeftCorner<3, 3>();
      EXPECT_TRUE((rot.transpose() * rot).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
      EXPECT_NEAR(rot.determinant(), 1.0, 1e-9);
      EXPECT_TRUE((tform.block<3, 1>(0, 3).isApprox(kBlock, 1e-12)));
      // Fingers close along a block axis, horizontally.
      const Eigen::Vector3d x = tform.block<3, 1>(0, 0);
      EXPECT_NEAR(x.z(), 0.0, 1e-12);
      const Eigen::Vector2d block_axis(std::cos(yaw), std::sin(yaw));
      const double dot = std::abs(x.head<2>().dot(block_axis));
      EXPECT_LT(std::min(dot, std::abs(dot - 1.0)), 1e-9);
    }
  }
}

TEST(GraspTest, TiltLeansAwayFromTheArm) {
  const std::array<double, 2> tilts = {0.0, 30.0 * kDeg};
  const std::vector<Eigen::Matrix4d> cands = graspCandidates(kBlock, 0.0, kArmBase, tilts);
  ASSERT_EQ(cands.size(), 4U);
  EXPECT_TRUE((cands[0].block<3, 1>(0, 2).isApprox(Eigen::Vector3d(0, 0, -1), 1e-12)));
  EXPECT_TRUE((cands[0].block<3, 1>(0, 0).isApprox(Eigen::Vector3d(0, 1, 0), 1e-12)));
  const Eigen::Vector3d z = cands[2].block<3, 1>(0, 2);
  EXPECT_NEAR(z.x(), std::sin(30.0 * kDeg), 1e-12);
  EXPECT_NEAR(z.z(), -std::cos(30.0 * kDeg), 1e-12);
}

TEST(GraspTest, OffsetBacksOffAlongApproach) {
  const std::array<double, 1> tilts = {0.0};
  const Eigen::Matrix4d tform = graspCandidates(kBlock, 0.0, kArmBase, tilts).front();
  const Eigen::Matrix4d backed = offset(tform, 0.05);
  EXPECT_TRUE((backed.block<3, 1>(0, 3).isApprox(Eigen::Vector3d(0.25, 0.0, 0.205), 1e-12)));
  EXPECT_TRUE((backed.topLeftCorner<3, 3>().isApprox(tform.topLeftCorner<3, 3>(), 1e-12)));
}

}  // namespace
}  // namespace emma_manipulation
