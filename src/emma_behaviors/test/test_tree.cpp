// Tree and leaf checks against the real planner, without ROS communication or the sim. Leaves are
// called directly (planPregrasp outside beet::offload), so no threads are involved. The tests
// share one seeded planner and run in file order, so the results are reproducible.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <numbers>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <beet/observe.hpp>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include "emma_behaviors/leaves.hpp"
#include "emma_behaviors/ros_io.hpp"
#include "emma_behaviors/tree.hpp"
#include "emma_behaviors/viz.hpp"
#include "emma_manipulation/constants.hpp"
#include "emma_manipulation/planner.hpp"

namespace emma_behaviors {
namespace {

using emma_manipulation::kArmJoints;
using emma_manipulation::kBlockStart;
using emma_manipulation::kHomeQ;
using emma_manipulation::kPlaceXyz;
using emma_manipulation::kPreOffset;

// The same checks as in tree.hpp, here so a change there cannot silently drop them.
static_assert(std::is_same_v<beet::input_t<Tree>, Cell>);
static_assert(std::is_same_v<beet::output_t<Tree>, Cell>);
static_assert(std::is_same_v<beet::error_t<Tree>, GaveUp>);
// A linear move needs a chosen grasp: there is no planLinear for Grasps.
static_assert(!std::is_invocable_v<decltype(&planLinear), Grasps, Target>);
static_assert(std::is_invocable_v<decltype(&planLinear), Chosen, Target>);

[[nodiscard]] std::string readFile(const std::string& path) {
  const std::ifstream file(path);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

[[nodiscard]] sensor_msgs::msg::JointState armState(const Eigen::VectorXd& q) {
  sensor_msgs::msg::JointState state;
  state.name = kArmJoints;
  state.position.assign(q.data(), q.data() + q.size());
  return state;
}

class TreeTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    const std::string urdf = readFile(EMMA_TEST_URDF);
    ASSERT_FALSE(urdf.empty()) << "cannot read " << EMMA_TEST_URDF;
    auto planner = std::make_shared<emma_manipulation::ArmPlanner>(
        urdf, emma_manipulation::ArmPlannerOptions{.seed = 7U});
    planner->addBox("table", 2.0 * emma_manipulation::kTableHalfSize,
                    translation(emma_manipulation::kTableCenter));
    cell_ = std::make_unique<Cell>(
        Cell{std::make_shared<Io>(), std::move(planner), std::make_shared<Viz>(Viz::disabled())});
  }

  static void TearDownTestSuite() { cell_.reset(); }

  void SetUp() override {
    // A failed SetUpTestSuite leaves no cell; fail each test cleanly instead of crashing.
    ASSERT_NE(cell_, nullptr) << "fixture not built, see the first failure";
    setJoints(kHomeQ);
  }

  static void setJoints(const Eigen::VectorXd& q) { storeJointState(*cell_->io, armState(q)); }

  [[nodiscard]] static Looked lookedAt(const Eigen::Vector3d& xyz) {
    return Looked{*cell_, makePose(xyz, 0.0, "base_link")};
  }

  static std::unique_ptr<Cell> cell_;
};

std::unique_ptr<Cell> TreeTest::cell_;

TEST(PoseHelpers, RoundTrip) {
  const double yaw = 30.0 * std::numbers::pi / 180.0;
  const Eigen::Matrix4d tform = poseToTform(makePose({0.1, 0.2, 0.3}, yaw, "base_link"));
  const Eigen::Vector3d xyz = tform.block<3, 1>(0, 3);
  EXPECT_TRUE(xyz.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_NEAR(yawOf(makePose({0.0, 0.0, 0.0}, yaw, "base_link")), yaw, 1e-12);
  const Eigen::Matrix3d r = tform.topLeftCorner<3, 3>();
  EXPECT_TRUE((r * r.transpose()).isIdentity(1e-12));
}

TEST(Tree, HasTheExpectedNodes) {
  constexpr auto& nodes = beet::tree_info<Tree>;
  for (const std::string_view label : {"wait for joints", "retry", "recover", "plan look",
                                       "detect", "close gripper", "check placed",
                                       "execute home"}) {
    EXPECT_TRUE(std::any_of(nodes.begin(), nodes.end(),
                            [label](const beet::node_info& n) { return n.label == label; }))
        << "no node labelled \"" << label << "\"";
  }
  EXPECT_EQ(nodes[0].parent, beet::no_parent);
}

TEST_F(TreeTest, RunsTracedAndWaitsForJoints) {
  // A fresh Io has no joint state, so the tree parks on its first leaf and starts no planning
  // thread. This also instantiates the traced tree the executable runs.
  const Cell cell{std::make_shared<Io>(), cell_->planner, cell_->viz};
  beet::StatusTable<Tree> table;
  beet::Runner runner{makeTree(cell), cell, table};
  EXPECT_EQ(runner.tick(), beet::Status::Running);
  EXPECT_EQ(runner.tick(), beet::Status::Running);
  EXPECT_EQ(table.status(0), beet::NodeStatus::Running);
  constexpr auto& nodes = beet::tree_info<Tree>;
  const auto wait = std::find_if(nodes.begin(), nodes.end(), [](const beet::node_info& n) {
    return n.label == "wait for joints";
  });
  ASSERT_NE(wait, nodes.end());
  EXPECT_EQ(table.status(static_cast<std::uint32_t>(wait - nodes.begin())),
            beet::NodeStatus::Running);
}

TEST_F(TreeTest, ComputeGrasps) {
  const Grasps grasps = computeGrasps(lookedAt(kBlockStart));
  ASSERT_FALSE(grasps.grasp.empty());
  EXPECT_EQ(grasps.pregrasp.size(), grasps.grasp.size());
  EXPECT_EQ(grasps.place.size(), grasps.grasp.size());
  EXPECT_EQ(grasps.preplace.size(), grasps.grasp.size());
  EXPECT_TRUE(cell_->planner->hasBox(std::string(kBlockName)));
  for (std::size_t i = 0; i < grasps.grasp.size(); ++i) {
    const Eigen::Matrix4d& grasp = grasps.grasp[i];
    const Eigen::Matrix4d& place = grasps.place[i];
    const Eigen::Vector3d grasp_xyz = grasp.block<3, 1>(0, 3);
    const Eigen::Vector3d place_xyz = place.block<3, 1>(0, 3);
    const Eigen::Vector3d preplace_xyz = grasps.preplace[i].block<3, 1>(0, 3);
    const Eigen::Matrix3d grasp_rotation = grasp.topLeftCorner<3, 3>();
    const Eigen::Matrix3d place_rotation = place.topLeftCorner<3, 3>();
    EXPECT_TRUE(grasp_xyz.isApprox(kBlockStart, 1e-12)) << "candidate " << i;
    EXPECT_TRUE(place_xyz.isApprox(kPlaceXyz, 1e-12)) << "candidate " << i;
    EXPECT_TRUE(place_rotation.isApprox(grasp_rotation, 1e-12)) << "candidate " << i;
    EXPECT_NEAR((preplace_xyz - place_xyz).norm(), kPreOffset, 1e-12) << "candidate " << i;
  }
}

TEST_F(TreeTest, PlanPregraspThenLinearGrasp) {
  const Grasps grasps = computeGrasps(lookedAt(kBlockStart));
  const auto pregrasp = planPregrasp(grasps);
  ASSERT_TRUE(pregrasp.has_value()) << pregrasp.error().why;
  const std::size_t index = pregrasp->state.index;
  ASSERT_LT(index, grasps.grasp.size());
  const trajectory_msgs::msg::JointTrajectory& trajectory = pregrasp->goal.trajectory;
  EXPECT_EQ(trajectory.joint_names, kArmJoints);
  ASSERT_FALSE(trajectory.points.empty());

  const std::vector<double>& q_pregrasp = trajectory.points.back().positions;
  setJoints(Eigen::Map<const Eigen::VectorXd>(q_pregrasp.data(),
                                              static_cast<Eigen::Index>(q_pregrasp.size())));
  const auto grasp = planLinear(pregrasp->state, Target::kGrasp);
  ASSERT_TRUE(grasp.has_value()) << grasp.error().why;
  EXPECT_EQ(grasp->state.index, index);
  const std::vector<double>& q_grasp = grasp->goal.trajectory.points.back().positions;
  const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
      q_grasp.data(), static_cast<Eigen::Index>(q_grasp.size()));
  const Eigen::Vector3d reached = cell_->planner->fk(q).block<3, 1>(0, 3);
  const Eigen::Vector3d target = grasps.grasp[index].block<3, 1>(0, 3);
  EXPECT_LT((reached - target).norm(), 1e-3);
}

TEST_F(TreeTest, CheckPlaced) {
  const Eigen::Vector3d near = kPlaceXyz + Eigen::Vector3d(0.01, 0.0, 0.0);
  EXPECT_TRUE(checkPlaced(lookedAt(near)).has_value());
  const auto far = checkPlaced(lookedAt(kBlockStart));
  ASSERT_FALSE(far.has_value());
  EXPECT_NEAR(far.error().error_m, (kBlockStart - kPlaceXyz).norm(), 1e-12);
}

TEST_F(TreeTest, DetachIfHeldWithoutBlockSucceeds) {
  if (cell_->planner->hasBox(std::string(kBlockName))) {
    cell_->planner->remove(std::string(kBlockName));
  }
  const Cell cell = detachIfHeld(*cell_);
  EXPECT_EQ(cell.planner, cell_->planner);
  EXPECT_FALSE(cell_->planner->hasBox(std::string(kBlockName)));
}

}  // namespace
}  // namespace emma_behaviors
