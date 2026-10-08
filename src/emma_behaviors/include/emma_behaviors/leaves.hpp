#pragma once

/// @file
/// The pick and place's edge types and leaves. Data flows along the tree's edges in these structs
/// instead of through a blackboard; every state carries the Cell (the shared resources) because
/// beet has no other way to reach later nodes.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <beet/beet.hpp>
#include <Eigen/Core>
#include <emma_manipulation/constants.hpp>
#include <emma_manipulation/planner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "emma_behaviors/ros_io.hpp"
#include "emma_behaviors/viz.hpp"

namespace emma_behaviors {

using beet::never;
using beet::Result;
using beet::Task;

/// 5 Hz: the tick rate the old py_trees version ran at, so tick counts match its timings.
inline constexpr std::chrono::milliseconds kTickPeriod{200};
/// About 1 s at the look pose before taking a detection, so the camera image is not blurred.
inline constexpr std::size_t kSettleTicks = 5;
/// About 10 s for a fresh `/block_pose` before the detect node fails with beet::Timeout.
inline constexpr std::size_t kDetectTicks = 50;
/// How far the re-detected block may be from kPlaceXyz for the place to count.
inline constexpr double kPlaceTolerance = 0.02;
/// Name of the block's box in the planning scene and its Rerun entity.
inline constexpr std::string_view kBlockName = "block";

// Errors. Distinct structs because beet merges same-type errors.

/// A planner call failed (no IK, no collision-free path, or no joint state).
struct NoPlan {
  std::string why;
};
/// An action goal was rejected, aborted or cancelled.
struct ActionFailed {
  std::string why;
};
/// The re-detected block is more than kPlaceTolerance from kPlaceXyz.
struct NotPlaced {
  double error_m;
};
/// Recovery ran; the tree reports failure on purpose.
struct GaveUp {};

[[nodiscard]] std::string describe(const NoPlan& error);
[[nodiscard]] std::string describe(const ActionFailed& error);
[[nodiscard]] std::string describe(const NotPlaced& error);
[[nodiscard]] std::string describe(const GaveUp& error);
[[nodiscard]] std::string describe(const beet::Timeout& error);

// Edge types.

/// Shared resources. Three shared_ptrs, so the copies `retry` and `fallback` make are cheap.
struct Cell {
  std::shared_ptr<Io> io;
  std::shared_ptr<emma_manipulation::ArmPlanner> planner;
  std::shared_ptr<Viz> viz;
};

/// A fresh block detection, in `base_link`.
struct Looked {
  Cell cell;
  geometry_msgs::msg::PoseStamped block;
};

/// TCP poses (in `base_link`) for every grasp candidate; index i of each list belongs together.
struct Grasps {
  Cell cell;
  std::vector<Eigen::Matrix4d> grasp;
  std::vector<Eigen::Matrix4d> pregrasp;
  std::vector<Eigen::Matrix4d> place;
  std::vector<Eigen::Matrix4d> preplace;
};

/// Grasps with the candidate the pregrasp plan reached. Linear moves take a Chosen, so moving
/// before a grasp is chosen does not compile.
struct Chosen : Grasps {
  std::size_t index;
};

/// A planned arm motion, ready for execute<S>, and the state to continue with afterwards.
template <class S>
struct Planned {
  S state;
  FollowJointTrajectory::Goal goal;
  /// Names the motion in logs and its Rerun path entity.
  std::string label;
};

/// The Cell inside any state.
template <class S>
[[nodiscard]] const Cell& cellOf(const S& state) {
  if constexpr (std::is_same_v<S, Cell>) {
    return state;
  } else {
    return state.cell;
  }
}

// Pose helpers.

/// @brief 4x4 transform of a pose (the header frame is not checked).
[[nodiscard]] Eigen::Matrix4d poseToTform(const geometry_msgs::msg::PoseStamped& pose);
/// @brief Rotation about z of a pose's orientation.
[[nodiscard]] double yawOf(const geometry_msgs::msg::PoseStamped& pose);
/// @brief A pose at `xyz`, rotated by `yaw` about z.
[[nodiscard]] geometry_msgs::msg::PoseStamped makePose(const Eigen::Vector3d& xyz, double yaw,
                                                       std::string_view frame);
[[nodiscard]] Eigen::Matrix4d translation(const Eigen::Vector3d& xyz);

/// @brief Logs to the Rerun text log and, when the Cell has a ROS node, to its logger.
void logEvent(const Cell& cell, std::string_view text, Viz::Level level = Viz::Level::kInfo);

// Non-template cores of the template leaves below.

/// @brief Plans from the latest joint state to `q_goal` and times the path for arm_controller.
[[nodiscard]] Result<FollowJointTrajectory::Goal, NoPlan> planJointGoal(
    const Cell& cell, const Eigen::VectorXd& q_goal, std::string_view label);

/// @brief Logs the goal's TCP path to Rerun, sends it to arm_controller and waits (one check
/// per tick) for the result.
[[nodiscard]] Task<Result<beet::unit, ActionFailed>> sendArmGoal(
    Cell cell, FollowJointTrajectory::Goal goal, std::string label);

/// @brief Sends a gripper position and waits (one check per tick) for the result.
[[nodiscard]] Task<Result<beet::unit, ActionFailed>> sendGripperGoal(Cell cell, double position);

// Leaves.

/// Waits until a joint state with every arm joint has arrived.
[[nodiscard]] Task<Result<Cell, never>> waitForJoints(Cell cell);

/// RRT from the latest joint state to `q_goal`.
template <class S>
[[nodiscard]] Result<Planned<S>, NoPlan> planJoint(S state, const Eigen::VectorXd& q_goal,
                                                   std::string_view label) {
  auto goal = planJointGoal(cellOf(state), q_goal, label);
  if (!goal.has_value()) {
    return tl::make_unexpected(std::move(goal.error()));
  }
  return Planned<S>{std::move(state), std::move(*goal), std::string(label)};
}

// beet::offload needs a non-generic callable, so each parameterised plan is its own function.
template <class S>
[[nodiscard]] Result<Planned<S>, NoPlan> planLook(S state) {
  return planJoint(std::move(state), emma_manipulation::kLookQ, "look");
}
template <class S>
[[nodiscard]] Result<Planned<S>, NoPlan> planHome(S state) {
  return planJoint(std::move(state), emma_manipulation::kHomeQ, "home");
}

/// RRT to the first reachable pregrasp; the reached candidate becomes the chosen grasp.
[[nodiscard]] Result<Planned<Chosen>, NoPlan> planPregrasp(Grasps grasps);
/// RRT to the chosen candidate's preplace.
[[nodiscard]] Result<Planned<Chosen>, NoPlan> planPreplace(Chosen chosen);

enum class Target { kGrasp, kPregrasp, kPlace, kPreplace };
/// Short straight move to the chosen candidate's `target` pose.
[[nodiscard]] Result<Planned<Chosen>, NoPlan> planLinear(Chosen chosen, Target target);
[[nodiscard]] Result<Planned<Chosen>, NoPlan> planGrasp(Chosen chosen);
/// Back up to the pregrasp, block in hand.
[[nodiscard]] Result<Planned<Chosen>, NoPlan> planLift(Chosen chosen);
[[nodiscard]] Result<Planned<Chosen>, NoPlan> planPlace(Chosen chosen);
/// Back up to the preplace, gripper open.
[[nodiscard]] Result<Planned<Chosen>, NoPlan> planRetreat(Chosen chosen);

/// Runs a planned motion on arm_controller.
///
/// Halting it mid-goal does not cancel the goal; no halt path reaches it in this tree.
template <class S>
[[nodiscard]] Task<Result<S, ActionFailed>> execute(Planned<S> planned) {
  co_await sendArmGoal(cellOf(planned.state), std::move(planned.goal), std::move(planned.label));
  co_return std::move(planned.state);
}

template <class S>
[[nodiscard]] Task<Result<S, ActionFailed>> gripper(S state, double position) {
  co_await sendGripperGoal(cellOf(state), position);
  co_return std::move(state);
}
template <class S>
[[nodiscard]] Task<Result<S, ActionFailed>> openGripper(S state) {
  return gripper(std::move(state), emma_manipulation::kGripperOpen);
}
template <class S>
[[nodiscard]] Task<Result<S, ActionFailed>> closeGripper(S state) {
  return gripper(std::move(state), emma_manipulation::kGripperClosed);
}

/// Waits kSettleTicks ticks (beet has tick counts, not wall timers).
template <class S>
[[nodiscard]] Task<Result<S, never>> settle(S state) {
  for (std::size_t i = 0; i < kSettleTicks; ++i) {
    co_await beet::running;
  }
  co_return std::move(state);
}

/// Waits for a `/block_pose` that arrives after this node starts: one detection per visit, taken
/// once the arm has settled. Holds no resources, so a timeout may destroy it at any tick.
template <class S>
[[nodiscard]] Task<Result<Looked, never>> waitBlockPose(S state) {
  const Io& io = *cellOf(state).io;
  const std::uint64_t seen = io.block_pose_count;
  while (io.block_pose_count == seen || !io.block_pose.has_value()) {
    co_await beet::running;
  }
  co_return Looked{cellOf(state), *io.block_pose};
}

/// Grasp, pregrasp, place and preplace candidates for the detected block. Place i is grasp i
/// moved to kPlaceXyz, so the block lands upright whichever grasp is chosen. Also puts the block
/// in the planning scene and in Rerun.
[[nodiscard]] Grasps computeGrasps(Looked looked);

/// Attaches the block to the wrist in the planning scene.
[[nodiscard]] Chosen attachBlock(Chosen chosen);
/// Leaves the block where the gripper is in the planning scene and re-logs it there.
[[nodiscard]] Chosen detachBlock(Chosen chosen);
/// detachBlock for recovery, where the block may not be held (or known) at all.
[[nodiscard]] Cell detachIfHeld(Cell cell);

/// Succeeds if the re-detected block is within kPlaceTolerance of kPlaceXyz.
[[nodiscard]] Result<Cell, NotPlaced> checkPlaced(Looked looked);

/// Always fails with GaveUp, after recovery has brought the arm home.
[[nodiscard]] Result<Cell, GaveUp> giveUp(Cell cell);

}  // namespace emma_behaviors
