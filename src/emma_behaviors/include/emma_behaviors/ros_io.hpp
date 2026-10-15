#pragma once

/// @file
/// The tree's only contact with ROS: topic caches and action clients, filled by callbacks that
/// run in the main loop's executor (`spin_some`) once per tick, before `Runner::tick()`.

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tl/expected.hpp>

namespace emma_behaviors {

inline constexpr std::string_view kJointStatesTopic = "/joint_states";
inline constexpr std::string_view kBlockPoseTopic = "/block_pose";
inline constexpr std::string_view kRobotDescriptionTopic = "/robot_description";
inline constexpr std::string_view kArmAction = "/arm_controller/follow_joint_trajectory";
inline constexpr std::string_view kGripperAction = "/gripper_action_controller/gripper_cmd";
/// The joint name parallel_gripper_controller expects in `ParallelGripperCommand::command`.
inline constexpr std::string_view kGripperJoint = "gripper_controller";

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;

/// @brief Latest topic samples and the action clients. A plain struct so tests can fill it by
/// hand without any ROS communication.
struct Io {
  rclcpp::Node::SharedPtr node;
  /// Latest `/joint_states` sample that has every arm joint. Guarded by `joint_state_mutex`:
  /// planning leaves read it from `beet::offload` threads while the main thread's callbacks
  /// write it.
  std::optional<sensor_msgs::msg::JointState> joint_state;
  mutable std::mutex joint_state_mutex;
  /// Main thread only (written by the callback, read by `waitBlockPose`).
  std::optional<geometry_msgs::msg::PoseStamped> block_pose;
  std::uint64_t block_pose_count = 0;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm;
  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
};

/// @brief The arm's latest joint positions in kArmJoints order.
/// @return The positions, or an error if no complete joint state has arrived yet.
[[nodiscard]] tl::expected<Eigen::VectorXd, std::string> armQ(const Io& io);

/// @brief Stores `joint_state` in `io` if it has every arm joint (under the lock).
void storeJointState(Io& io, const sensor_msgs::msg::JointState& joint_state);

/// @brief Subscribes to `/joint_states` and `/block_pose` and connects the arm and gripper action
/// clients, waiting up to 10 s (wall time) for each action server.
/// @return The I/O, or an error naming the action server that did not appear.
[[nodiscard]] tl::expected<std::shared_ptr<Io>, std::string> openIo(rclcpp::Node::SharedPtr node);

/// @brief Waits for the latched `/robot_description`.
///
/// Wall time, because sim time stands still until `/clock` is published.
/// @return The URDF, or an error after `wall_timeout`.
[[nodiscard]] tl::expected<std::string, std::string> waitForUrdf(
    rclcpp::Node& node, std::chrono::seconds wall_timeout);

}  // namespace emma_behaviors
