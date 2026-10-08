// Planner checks against emma's real URDF, with the table and block in the scene. The tests share
// one seeded planner and run in file order, so the random draws (and results) are reproducible.

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <gtest/gtest.h>
#include <roboplan/core/pose_utils.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "emma_manipulation/constants.hpp"
#include "emma_manipulation/grasp.hpp"
#include "emma_manipulation/planner.hpp"
#include "emma_manipulation/trajectory.hpp"

namespace emma_manipulation {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

[[nodiscard]] Eigen::Matrix4d translation(const Eigen::Vector3d& xyz) {
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.block<3, 1>(0, 3) = xyz;
  return tform;
}

[[nodiscard]] std::string readFile(const std::string& path) {
  const std::ifstream file(path);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

[[nodiscard]] std::vector<Eigen::Matrix4d> pregrasps(const std::vector<Eigen::Matrix4d>& grasps) {
  std::vector<Eigen::Matrix4d> out;
  out.reserve(grasps.size());
  for (const Eigen::Matrix4d& grasp : grasps) {
    out.push_back(offset(grasp, kPreOffset));
  }
  return out;
}

class PlannerTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    const std::string urdf = readFile(EMMA_TEST_URDF);
    ASSERT_FALSE(urdf.empty()) << "cannot read " << EMMA_TEST_URDF;
    planner_ = std::make_unique<ArmPlanner>(urdf, ArmPlannerOptions{.seed = 7U});
    planner_->addBox("table", 2.0 * kTableHalfSize, translation(kTableCenter));
    planner_->addBox("block", Eigen::Vector3d::Constant(kBlockSize), translation(kBlockStart));
    const Eigen::Vector2d arm_base_xy = planner_->fk(kHomeQ, "g_base").block<2, 1>(0, 3);
    grasps_ = graspCandidates(kBlockStart, 0.0, arm_base_xy);
  }

  static void TearDownTestSuite() { planner_.reset(); }

  // A failed ASSERT in SetUpTestSuite only leaves it early; fail each test instead of crashing.
  void SetUp() override { ASSERT_NE(planner_, nullptr) << "planner fixture not built"; }

  static std::unique_ptr<ArmPlanner> planner_;
  static std::vector<Eigen::Matrix4d> grasps_;
};

std::unique_ptr<ArmPlanner> PlannerTest::planner_;
std::vector<Eigen::Matrix4d> PlannerTest::grasps_;

TEST_F(PlannerTest, HomeCollisionFree) {
  EXPECT_TRUE(planner_->fk(kHomeQ).allFinite());
  EXPECT_FALSE(planner_->hasCollisions(kHomeQ));
}

TEST_F(PlannerTest, TcpIs45mmPastGripperBase) {
  const Eigen::Matrix4d rel = planner_->fk(kHomeQ, "gripper_base").inverse() * planner_->fk(kHomeQ);
  EXPECT_NEAR(rel(0, 3), 0.0, 1e-9);
  EXPECT_NEAR(rel(1, 3), 0.0, 1e-9);
  EXPECT_NEAR(rel(2, 3), 0.045, 1e-9);
}

TEST_F(PlannerTest, ArmPositionsOrdersByArmJoints) {
  sensor_msgs::msg::JointState msg;
  msg.name.assign(kArmJoints.rbegin(), kArmJoints.rend());
  msg.name.emplace_back("gripper_controller");
  msg.position = {6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0};
  const tl::expected<Eigen::VectorXd, std::string> q = ArmPlanner::armPositions(msg);
  ASSERT_TRUE(q.has_value()) << q.error();
  EXPECT_TRUE(q->isApprox((Eigen::VectorXd(6) << 1, 2, 3, 4, 5, 6).finished()));

  msg.name.erase(msg.name.begin());  // Drops joint6output_to_joint6.
  msg.position.erase(msg.position.begin());
  const tl::expected<Eigen::VectorXd, std::string> missing = ArmPlanner::armPositions(msg);
  ASSERT_FALSE(missing.has_value());
  EXPECT_NE(missing.error().find("joint6output_to_joint6"), std::string::npos) << missing.error();
}

TEST_F(PlannerTest, IkRoundTrip) {
  const Eigen::VectorXd q_true = (Eigen::VectorXd(6) << 0.3, 0.4, -1.2, -0.6, 0.2, 0.5).finished();
  ASSERT_FALSE(planner_->hasCollisions(q_true));
  const Eigen::Matrix4d target = planner_->fk(q_true);
  const std::optional<Eigen::VectorXd> q = planner_->ik(target, kHomeQ);
  ASSERT_TRUE(q.has_value());
  const auto [pos_err, rot_err] = roboplan::poseError(planner_->fk(*q), target);
  EXPECT_LT(pos_err, 1e-3);
  EXPECT_LT(rot_err, 0.5 * kDeg);
}

TEST_F(PlannerTest, BlockReachableFromHome) {
  // The reachability gate for the table/block constants.
  const tl::expected<ArmPlanner::PlanToAny, std::string> found =
      planner_->planToAny(kHomeQ, pregrasps(grasps_));
  ASSERT_TRUE(found.has_value()) << found.error();
  const auto& [path, index] = *found;
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(path.front().isApprox(kHomeQ, 1e-7));
  const auto [pos_err, rot_err] =
      roboplan::poseError(planner_->fk(path.back()), offset(grasps_[index], kPreOffset));
  std::ignore = rot_err;
  EXPECT_LT(pos_err, 1e-3);
  for (const Eigen::VectorXd& q : path) {
    EXPECT_FALSE(planner_->hasCollisions(q));
  }
}

TEST_F(PlannerTest, LinearGraspAndRetreat) {
  const tl::expected<ArmPlanner::PlanToAny, std::string> found =
      planner_->planToAny(kHomeQ, pregrasps(grasps_));
  ASSERT_TRUE(found.has_value()) << found.error();
  const auto& [path, index] = *found;
  const tl::expected<std::vector<Eigen::VectorXd>, std::string> down =
      planner_->planLinear(path.back(), grasps_[index]);
  ASSERT_TRUE(down.has_value()) << down.error();
  const auto [pos_err, rot_err] = roboplan::poseError(planner_->fk(down->back()), grasps_[index]);
  std::ignore = rot_err;
  EXPECT_LT(pos_err, 1e-3);

  planner_->attach("block", down->back());
  // Detach even when an assertion below returns early, so later tests see the block on the table.
  struct DetachGuard {
    ArmPlanner& planner;
    const Eigen::VectorXd& q;
    ~DetachGuard() { planner.detach("block", q); }
  } const guard{*planner_, down->back()};

  const tl::expected<std::vector<Eigen::VectorXd>, std::string> up =
      planner_->planLinear(down->back(), offset(grasps_[index], kPreOffset));
  ASSERT_TRUE(up.has_value()) << up.error();
  // The held block rises with the TCP.
  const double lifted = planner_->fk(up->back())(2, 3) - planner_->fk(down->back())(2, 3);
  EXPECT_GT(lifted, 0.8 * kPreOffset * std::cos(35.0 * kDeg));
}

TEST(TrajectoryTest, JointTrajectoryTiming) {
  const Eigen::VectorXd home = kHomeQ;
  const std::vector<Eigen::VectorXd> path = {
      home, (home.array() + 0.4).matrix(), (home.array() + 0.41).matrix(), home};
  const trajectory_msgs::msg::JointTrajectory traj = toJointTrajectory(path, 0.8);
  EXPECT_EQ(traj.joint_names, kArmJoints);
  ASSERT_EQ(traj.points.size(), 3U);
  const std::vector<double> expected = {0.5, 0.6, 1.1125};
  for (std::size_t i = 0; i < traj.points.size(); ++i) {
    const builtin_interfaces::msg::Duration& t = traj.points[i].time_from_start;
    EXPECT_NEAR(t.sec + t.nanosec * 1e-9, expected[i], 1e-9);
  }
  const std::vector<double>& last = traj.points.back().positions;
  ASSERT_EQ(last.size(), 6U);
  for (std::size_t j = 0; j < last.size(); ++j) {
    EXPECT_NEAR(last[j], home(static_cast<Eigen::Index>(j)), 1e-12);
  }
}

// Optical frame pose `distance` from `target`, looking along +x tilted `down_deg` down.
[[nodiscard]] Eigen::Matrix4d cameraLookingAt(const Eigen::Vector3d& target, const double distance,
                                              const double down_deg) {
  const double down = down_deg * kDeg;
  const Eigen::Vector3d z(std::cos(down), 0.0, -std::sin(down));
  // Image right; image down then points away from the robot.
  const Eigen::Vector3d x = Eigen::Vector3d::UnitY();
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.block<3, 1>(0, 0) = x;
  tform.block<3, 1>(0, 1) = z.cross(x);
  tform.block<3, 1>(0, 2) = z;
  tform.block<3, 1>(0, 3) = target - distance * z;
  return tform;
}

TEST_F(PlannerTest, LookPose) {
  // How kLookQ was found: IK for the optical frame (through its fixed offset from the TCP).
  const Eigen::Matrix4d cam_in_tcp =
      planner_->fk(kHomeQ).inverse() * planner_->fk(kHomeQ, kCameraFrame);
  const Eigen::Matrix4d goal = cameraLookingAt(kBlockStart, 0.2, 65.0);
  const std::optional<Eigen::VectorXd> q = planner_->ik(goal * cam_in_tcp.inverse(), kLookQ);
  ASSERT_TRUE(q.has_value());
  const Eigen::IOFormat fmt(3, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  std::cout << std::fixed << "look pose by IK: " << q->transpose().format(fmt) << "\n";

  // The hardcoded kLookQ is collision free, reachable from home and sees the work area.
  EXPECT_FALSE(planner_->hasCollisions(kLookQ));
  const tl::expected<std::vector<Eigen::VectorXd>, std::string> plan =
      planner_->planJoint(kHomeQ, kLookQ);
  EXPECT_TRUE(plan.has_value()) << plan.error();
  const Eigen::Matrix4d world_to_cam = planner_->fk(kLookQ, kCameraFrame).inverse();
  // The MuJoCo camera: fovy 65 deg, 640x400.
  const double f = 200.0 / std::tan(65.0 / 2.0 * kDeg);
  const std::vector<std::pair<Eigen::Vector3d, double>> points = {
      {kBlockStart, 20.0},
      {kPlaceXyz, 40.0},
      {kBlockStart + Eigen::Vector3d(0.03, 0.0, 0.0), 40.0},
      {kBlockStart + Eigen::Vector3d(0.0, -0.03, 0.0), 40.0},
  };
  for (const auto& [point, margin] : points) {
    const Eigen::Vector3d p =
        world_to_cam.topLeftCorner<3, 3>() * point + world_to_cam.block<3, 1>(0, 3);
    const double u = f * p.x() / p.z() + 320.0;
    const double v = f * p.y() / p.z() + 200.0;
    SCOPED_TRACE(testing::Message() << "point " << point.transpose() << " u " << u << " v " << v);
    EXPECT_GT(p.z(), 0.15);
    EXPECT_LT(p.z(), 0.3);
    EXPECT_GT(u, margin);
    EXPECT_LT(u, 640.0 - margin);
    EXPECT_GT(v, margin);
    EXPECT_LT(v, 400.0 - margin);
  }
  const Eigen::Vector3d block =
      world_to_cam.topLeftCorner<3, 3>() * kBlockStart + world_to_cam.block<3, 1>(0, 3);
  EXPECT_LT(std::hypot(block.x(), block.y()), 0.01);  // Block on the optical axis.
}

TEST_F(PlannerTest, PlansFromJustPastAJointLimit) {
  // A measured joint state can overshoot a limit slightly; planning must still start.
  Eigen::VectorXd q = kHomeQ;
  q(5) = std::numbers::pi + 1e-4;  // The joint 6 limit is 3.14159.
  EXPECT_FALSE(planner_->hasCollisions(q));
  EXPECT_NEAR(planner_->clamp(q)(5), 3.14159, 1e-7);
  const tl::expected<std::vector<Eigen::VectorXd>, std::string> path =
      planner_->planJoint(q, kHomeQ);
  ASSERT_TRUE(path.has_value()) << path.error();
  EXPECT_TRUE(path->back().isApprox(kHomeQ, 1e-9));
}

}  // namespace
}  // namespace emma_manipulation
