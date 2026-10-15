#include "emma_behaviors/ros_io.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <emma_manipulation/planner.hpp>
#include <std_msgs/msg/string.hpp>

namespace emma_behaviors {

tl::expected<Eigen::VectorXd, std::string> armQ(const Io& io) {
  const std::scoped_lock lock(io.joint_state_mutex);
  if (!io.joint_state.has_value()) {
    return tl::make_unexpected(std::string("no joint state with all arm joints yet"));
  }
  return emma_manipulation::ArmPlanner::armPositions(*io.joint_state);
}

void storeJointState(Io& io, const sensor_msgs::msg::JointState& joint_state) {
  // Partial samples (e.g. only the gripper) would make every later armQ() fail.
  if (!emma_manipulation::ArmPlanner::armPositions(joint_state).has_value()) {
    return;
  }
  const std::scoped_lock lock(io.joint_state_mutex);
  io.joint_state = joint_state;
}

tl::expected<std::shared_ptr<Io>, std::string> openIo(rclcpp::Node::SharedPtr node) {
  auto io = std::make_shared<Io>();
  io->node = node;
  // The callbacks hold a raw pointer: Io owns the subscriptions, so it outlives them.
  Io* const raw = io.get();
  io->subs.push_back(node->create_subscription<sensor_msgs::msg::JointState>(
      std::string(kJointStatesTopic), rclcpp::SensorDataQoS(),
      [raw](const sensor_msgs::msg::JointState& msg) { storeJointState(*raw, msg); }));
  io->subs.push_back(node->create_subscription<geometry_msgs::msg::PoseStamped>(
      std::string(kBlockPoseTopic), rclcpp::QoS(1),
      [raw](const geometry_msgs::msg::PoseStamped& msg) {
        raw->block_pose = msg;
        ++raw->block_pose_count;
      }));

  constexpr std::chrono::seconds kServerTimeout{10};
  io->arm = rclcpp_action::create_client<FollowJointTrajectory>(node, std::string(kArmAction));
  if (!io->arm->wait_for_action_server(kServerTimeout)) {
    return tl::make_unexpected("action server " + std::string(kArmAction) + " not available");
  }
  io->gripper =
      rclcpp_action::create_client<ParallelGripperCommand>(node, std::string(kGripperAction));
  if (!io->gripper->wait_for_action_server(kServerTimeout)) {
    return tl::make_unexpected("action server " + std::string(kGripperAction) +
                               " not available");
  }
  return io;
}

tl::expected<std::string, std::string> waitForUrdf(rclcpp::Node& node,
                                                    std::chrono::seconds wall_timeout) {
  std::optional<std::string> urdf;
  const auto sub = node.create_subscription<std_msgs::msg::String>(
      std::string(kRobotDescriptionTopic), rclcpp::QoS(1).transient_local(),
      [&urdf](const std_msgs::msg::String& msg) { urdf = msg.data; });
  // rclcpp::spin_some(node) is deprecated in Lyrical; a local executor does the same.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node.get_node_base_interface());
  const auto deadline = std::chrono::steady_clock::now() + wall_timeout;
  while (!urdf.has_value() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  executor.remove_node(node.get_node_base_interface());
  if (!urdf.has_value()) {
    return tl::make_unexpected("no " + std::string(kRobotDescriptionTopic) + " within " +
                               std::to_string(wall_timeout.count()) + " s");
  }
  return *urdf;
}

}  // namespace emma_behaviors
