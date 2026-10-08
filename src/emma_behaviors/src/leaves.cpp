#include "emma_behaviors/leaves.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <emma_manipulation/grasp.hpp>
#include <emma_manipulation/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace emma_behaviors {
namespace {

using emma_manipulation::kBlockSize;
using emma_manipulation::kPlaceXyz;
using emma_manipulation::kPreOffset;

const Eigen::Vector4d kBlockRgba{0.9, 0.1, 0.1, 1.0};

[[nodiscard]] std::string formatXyz(const Eigen::Vector3d& xyz) {
  char text[64];
  std::snprintf(text, sizeof(text), "[%.3f %.3f %.3f]", xyz.x(), xyz.y(), xyz.z());
  return text;
}

/// The latest arm joint positions; leaves that need them run after waitForJoints, so a missing
/// joint state here is a programming error.
[[nodiscard]] Eigen::VectorXd requireArmQ(const Cell& cell) {
  auto q = armQ(*cell.io);
  if (!q.has_value()) {
    throw std::runtime_error(q.error());
  }
  return std::move(*q);
}

[[nodiscard]] FollowJointTrajectory::Goal toGoal(const std::vector<Eigen::VectorXd>& path) {
  FollowJointTrajectory::Goal goal;
  goal.trajectory = emma_manipulation::toJointTrajectory(path);
  return goal;
}

[[nodiscard]] Result<Planned<Chosen>, NoPlan> plannedOrError(
    Chosen chosen, const tl::expected<std::vector<Eigen::VectorXd>, std::string>& path,
    std::string_view label) {
  if (!path.has_value()) {
    logEvent(chosen.cell, std::string(label) + ": no plan: " + path.error(),
             Viz::Level::kWarning);
    return tl::make_unexpected(NoPlan{std::string(label) + ": " + path.error()});
  }
  logEvent(chosen.cell,
           std::string(label) + ": planned " + std::to_string(path->size()) + " waypoints");
  return Planned<Chosen>{std::move(chosen), toGoal(*path), std::string(label)};
}

void logEePath(const Cell& cell, const FollowJointTrajectory::Goal& goal, std::string_view label) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(goal.trajectory.points.size() + 1);
  if (const auto q = armQ(*cell.io); q.has_value()) {
    points.push_back(cell.planner->fk(*q).block<3, 1>(0, 3));
  }
  for (const trajectory_msgs::msg::JointTrajectoryPoint& point : goal.trajectory.points) {
    const Eigen::VectorXd q =
        Eigen::Map<const Eigen::VectorXd>(point.positions.data(),
                                          static_cast<Eigen::Index>(point.positions.size()));
    points.push_back(cell.planner->fk(q).block<3, 1>(0, 3));
  }
  cell.viz->logPath(label, points);
}

[[nodiscard]] std::string_view resultCodeName(rclcpp_action::ResultCode code) {
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "succeeded";
    case rclcpp_action::ResultCode::ABORTED:
      return "aborted";
    case rclcpp_action::ResultCode::CANCELED:
      return "canceled";
    case rclcpp_action::ResultCode::UNKNOWN:
      break;
  }
  return "unknown result";
}

template <class Future>
[[nodiscard]] bool ready(const Future& future) {
  return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

/// Sends `goal` and waits for its result, checking once per tick. The futures complete in the
/// action client's callbacks, which run in `rclcpp::spin_some` before each tick.
template <class Action>
Task<Result<typename Action::Result::SharedPtr, ActionFailed>> runGoal(
    typename rclcpp_action::Client<Action>::SharedPtr client, typename Action::Goal goal,
    std::string what) {
  if (!client) {
    co_return tl::make_unexpected(ActionFailed{what + ": no action client"});
  }
  const auto handle_future = client->async_send_goal(goal);
  while (!ready(handle_future)) {
    co_await beet::running;
  }
  const auto handle = handle_future.get();
  if (!handle) {
    co_return tl::make_unexpected(ActionFailed{what + ": goal rejected"});
  }
  const auto result_future = client->async_get_result(handle);
  while (!ready(result_future)) {
    co_await beet::running;
  }
  const auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    co_return tl::make_unexpected(
        ActionFailed{what + ": " + std::string(resultCodeName(wrapped.code))});
  }
  co_return wrapped.result;
}

}  // namespace

std::string describe(const NoPlan& error) { return "no plan: " + error.why; }
std::string describe(const ActionFailed& error) { return "action failed: " + error.why; }
std::string describe(const NotPlaced& error) {
  return "block not placed: " + std::to_string(error.error_m * 1000.0) + " mm from the target";
}
std::string describe(const GaveUp&) { return "gave up after recovery"; }
std::string describe(const beet::Timeout& error) {
  return "timed out after " + std::to_string(error.ticks) + " ticks";
}

Eigen::Matrix4d poseToTform(const geometry_msgs::msg::PoseStamped& pose) {
  const geometry_msgs::msg::Quaternion& o = pose.pose.orientation;
  const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.topLeftCorner<3, 3>() = q.normalized().toRotationMatrix();
  tform.block<3, 1>(0, 3) =
      Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  return tform;
}

double yawOf(const geometry_msgs::msg::PoseStamped& pose) {
  const geometry_msgs::msg::Quaternion& q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::PoseStamped makePose(const Eigen::Vector3d& xyz, double yaw,
                                         std::string_view frame) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = std::string(frame);
  pose.pose.position.x = xyz.x();
  pose.pose.position.y = xyz.y();
  pose.pose.position.z = xyz.z();
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

Eigen::Matrix4d translation(const Eigen::Vector3d& xyz) {
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.block<3, 1>(0, 3) = xyz;
  return tform;
}

void logEvent(const Cell& cell, std::string_view text, Viz::Level level) {
  if (cell.viz) {
    cell.viz->logText(text, level);
  }
  if (cell.io && cell.io->node) {
    const rclcpp::Logger logger = cell.io->node->get_logger();
    const std::string message(text);
    switch (level) {
      case Viz::Level::kInfo:
        RCLCPP_INFO(logger, "%s", message.c_str());
        break;
      case Viz::Level::kWarning:
        RCLCPP_WARN(logger, "%s", message.c_str());
        break;
      case Viz::Level::kError:
        RCLCPP_ERROR(logger, "%s", message.c_str());
        break;
    }
  }
}

Result<FollowJointTrajectory::Goal, NoPlan> planJointGoal(const Cell& cell,
                                                          const Eigen::VectorXd& q_goal,
                                                          std::string_view label) {
  const auto q = armQ(*cell.io);
  if (!q.has_value()) {
    return tl::make_unexpected(NoPlan{std::string(label) + ": " + q.error()});
  }
  const auto path = cell.planner->planJoint(*q, q_goal);
  if (!path.has_value()) {
    logEvent(cell, std::string(label) + ": no plan: " + path.error(), Viz::Level::kWarning);
    return tl::make_unexpected(NoPlan{std::string(label) + ": " + path.error()});
  }
  logEvent(cell, std::string(label) + ": planned " + std::to_string(path->size()) + " waypoints");
  return toGoal(*path);
}

Task<Result<beet::unit, ActionFailed>> sendArmGoal(Cell cell, FollowJointTrajectory::Goal goal,
                                                   std::string label) {
  logEePath(cell, goal, label);
  const auto result = co_await beet::settle(
      runGoal<FollowJointTrajectory>(cell.io->arm, std::move(goal), "execute " + label));
  if (!result.has_value()) {
    logEvent(cell, result.error().why, Viz::Level::kWarning);
    co_return tl::make_unexpected(result.error());
  }
  if (const auto& r = *result; r && r->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
    const std::string why = "execute " + label + ": error code " + std::to_string(r->error_code) +
                            " " + r->error_string;
    logEvent(cell, why, Viz::Level::kWarning);
    co_return tl::make_unexpected(ActionFailed{why});
  }
  logEvent(cell, "execute " + label + ": done");
  co_return beet::unit{};
}

Task<Result<beet::unit, ActionFailed>> sendGripperGoal(Cell cell, double position) {
  ParallelGripperCommand::Goal goal;
  goal.command.name = {std::string(kGripperJoint)};
  goal.command.position = {position};
  const std::string what = "gripper to " + std::to_string(position);
  const auto result =
      co_await beet::settle(runGoal<ParallelGripperCommand>(cell.io->gripper, goal, what));
  if (!result.has_value()) {
    logEvent(cell, result.error().why, Viz::Level::kWarning);
    co_return tl::make_unexpected(result.error());
  }
  logEvent(cell, what + ": done");
  co_return beet::unit{};
}

Task<Result<Cell, never>> waitForJoints(Cell cell) {
  while (!armQ(*cell.io).has_value()) {
    co_await beet::running;
  }
  co_return std::move(cell);
}

Result<Planned<Chosen>, NoPlan> planPregrasp(Grasps grasps) {
  const auto q = armQ(*grasps.cell.io);
  if (!q.has_value()) {
    return tl::make_unexpected(NoPlan{"pregrasp: " + q.error()});
  }
  auto found = grasps.cell.planner->planToAny(*q, grasps.pregrasp);
  if (!found.has_value()) {
    logEvent(grasps.cell, "pregrasp: no plan: " + found.error(), Viz::Level::kWarning);
    return tl::make_unexpected(NoPlan{"pregrasp: " + found.error()});
  }
  logEvent(grasps.cell, "chose grasp candidate " + std::to_string(found->index));
  Chosen chosen{std::move(grasps), found->index};
  return plannedOrError(std::move(chosen), std::move(found->path), "pregrasp");
}

Result<Planned<Chosen>, NoPlan> planPreplace(Chosen chosen) {
  const auto q = armQ(*chosen.cell.io);
  if (!q.has_value()) {
    return tl::make_unexpected(NoPlan{"preplace: " + q.error()});
  }
  const auto found =
      chosen.cell.planner->planToAny(*q, {chosen.preplace.at(chosen.index)});
  if (!found.has_value()) {
    return plannedOrError(std::move(chosen), tl::make_unexpected(found.error()), "preplace");
  }
  return plannedOrError(std::move(chosen), found->path, "preplace");
}

Result<Planned<Chosen>, NoPlan> planLinear(Chosen chosen, Target target) {
  const auto [poses, label] = [&]() -> std::pair<const std::vector<Eigen::Matrix4d>*, const char*> {
    switch (target) {
      case Target::kGrasp:
        return {&chosen.grasp, "grasp"};
      case Target::kPregrasp:
        return {&chosen.pregrasp, "lift"};
      case Target::kPlace:
        return {&chosen.place, "place"};
      case Target::kPreplace:
        return {&chosen.preplace, "retreat"};
    }
    throw std::invalid_argument("unknown linear target");
  }();
  const auto q = armQ(*chosen.cell.io);
  if (!q.has_value()) {
    return tl::make_unexpected(NoPlan{std::string(label) + ": " + q.error()});
  }
  const Eigen::Matrix4d pose = poses->at(chosen.index);
  const auto path = chosen.cell.planner->planLinear(*q, pose);
  return plannedOrError(std::move(chosen), path, label);
}

Result<Planned<Chosen>, NoPlan> planGrasp(Chosen chosen) {
  return planLinear(std::move(chosen), Target::kGrasp);
}
Result<Planned<Chosen>, NoPlan> planLift(Chosen chosen) {
  return planLinear(std::move(chosen), Target::kPregrasp);
}
Result<Planned<Chosen>, NoPlan> planPlace(Chosen chosen) {
  return planLinear(std::move(chosen), Target::kPlace);
}
Result<Planned<Chosen>, NoPlan> planRetreat(Chosen chosen) {
  return planLinear(std::move(chosen), Target::kPreplace);
}

Grasps computeGrasps(Looked looked) {
  const Cell& cell = looked.cell;
  const Eigen::Matrix4d block = poseToTform(looked.block);
  const Eigen::Vector3d block_xyz = block.block<3, 1>(0, 3);
  const double yaw = yawOf(looked.block);
  // The arm base is fixed in base_link, so any configuration gives its position.
  const Eigen::Vector2d arm_base_xy =
      cell.planner->fk(emma_manipulation::kHomeQ, "g_base").block<2, 1>(0, 3);

  Grasps grasps{cell, emma_manipulation::graspCandidates(block_xyz, yaw, arm_base_xy), {}, {}, {}};
  const Eigen::Vector3d shift = kPlaceXyz - block_xyz;
  for (const Eigen::Matrix4d& grasp : grasps.grasp) {
    Eigen::Matrix4d place = grasp;
    place.block<3, 1>(0, 3) += shift;
    grasps.pregrasp.push_back(emma_manipulation::offset(grasp, kPreOffset));
    grasps.place.push_back(place);
    grasps.preplace.push_back(emma_manipulation::offset(place, kPreOffset));
  }

  const Eigen::Vector3d size = Eigen::Vector3d::Constant(kBlockSize);
  cell.planner->addBox(std::string(kBlockName), size, block, kBlockRgba);
  cell.viz->logBox(kBlockName, size, block, kBlockRgba);
  char yaw_text[32];
  std::snprintf(yaw_text, sizeof(yaw_text), "%.0f", yaw * 180.0 / std::numbers::pi);
  logEvent(cell, std::to_string(grasps.grasp.size()) + " grasp candidates for the block at " +
                     formatXyz(block_xyz) + ", yaw " + yaw_text + " deg");
  return grasps;
}

Chosen attachBlock(Chosen chosen) {
  chosen.cell.planner->attach(std::string(kBlockName), requireArmQ(chosen.cell));
  return chosen;
}

Chosen detachBlock(Chosen chosen) {
  const Cell& cell = chosen.cell;
  if (cell.planner->hasBox(std::string(kBlockName))) {
    const Eigen::VectorXd q = requireArmQ(cell);
    cell.planner->detach(std::string(kBlockName), q);
    // Grasps are centred on the block, so the TCP pose is where it was released.
    cell.viz->logBox(kBlockName, Eigen::Vector3d::Constant(kBlockSize), cell.planner->fk(q),
                     kBlockRgba);
  }
  return chosen;
}

Cell detachIfHeld(Cell cell) {
  if (!cell.planner->hasBox(std::string(kBlockName))) {
    return cell;
  }
  if (const auto q = armQ(*cell.io); q.has_value()) {
    cell.planner->detach(std::string(kBlockName), *q);
  }
  return cell;
}

Result<Cell, NotPlaced> checkPlaced(Looked looked) {
  const Eigen::Vector3d xyz(looked.block.pose.position.x, looked.block.pose.position.y,
                            looked.block.pose.position.z);
  const double error = (xyz - kPlaceXyz).norm();
  char text[96];
  std::snprintf(text, sizeof(text), ", %.1f mm from the target", error * 1000.0);
  logEvent(looked.cell, "block at " + formatXyz(xyz) + text);
  if (error > kPlaceTolerance) {
    return tl::make_unexpected(NotPlaced{error});
  }
  return std::move(looked.cell);
}

Result<Cell, GaveUp> giveUp(Cell cell) {
  logEvent(cell, describe(GaveUp{}), Viz::Level::kError);
  return tl::make_unexpected(GaveUp{});
}

}  // namespace emma_behaviors
