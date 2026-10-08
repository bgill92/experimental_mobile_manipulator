#pragma once

/// @file
/// In-process arm planning for emma on top of roboplan. Kept free of rclcpp.

#include <cstddef>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tl/expected.hpp>

#include "emma_manipulation/constants.hpp"

namespace roboplan {
class Scene;
class SimpleIk;
class RRT;
class PathShortcutter;
}  // namespace roboplan

namespace emma_manipulation {

/// @brief emma's URDF adapted for roboplan, with the names of all its links.
struct PlanningUrdf {
  std::string xml;
  std::vector<std::string> link_names;
};

/// @brief Adapts emma's URDF for roboplan.
///
/// - `<mimic>` tags are dropped: Pinocchio requires the mimicked joint to come first and here it
///   does not. The right finger becomes an independent joint left at 0 (open).
/// - The myAGV links have visuals only; their meshes are copied into collisions so the planner
///   keeps the arm out of the base.
/// - coal/assimp ignore the COLLADA `<unit>` tag, so meshes in other units load at the wrong size
///   (gripper 1000x, base 39x). RViz honours the tag, so the robot_description stays as is and the
///   scale is set on the planning side only.
/// @return The rewritten URDF, or a parse error.
[[nodiscard]] tl::expected<PlanningUrdf, std::string> rewriteUrdfForPlanning(
    const std::string& urdf_xml);

/// @brief Directories that resolve the URDF's `package://` meshes: the parents of the share
/// directories of emma_description, mycobot_description and myagv_description.
/// @throws ament_index_cpp::PackageNotFoundError if one of them is not installed.
[[nodiscard]] std::vector<std::filesystem::path> meshPackagePaths();

/// @brief Construction options for ArmPlanner.
///
/// Declared outside the class so it can be a default argument of the constructor (a nested struct
/// with default member initializers cannot be used before the enclosing class is complete).
struct ArmPlannerOptions {
  std::string base_frame{kBaseFrame};
  std::string tip_frame{kTcpFrame};
  double max_planning_time = 2.0;
  /// Seeds the scene, IK and RRT random generators; unset means random.
  std::optional<unsigned> seed;
};

/// @brief Collision-aware IK and motion planning for the arm, in the joint space of kArmJoints.
///
/// Configurations `q` are 6-vectors in kArmJoints order. Poses are 4x4 transforms of the tip frame
/// in the base frame. The base, wheels and gripper fingers stay at their default positions
/// (fingers open) for collision checking.
///
/// Errors on the expected paths (no IK, no plan) are returned; roboplan calls that only fail on
/// programming errors (unknown names) throw std::runtime_error.
///
/// @warning Not thread-safe: the scene scratch, the solvers' generators and the box bookkeeping are
/// shared and mutated. `scene().forwardKinematics(data, ...)` with a caller-owned
/// `pinocchio::Data` is the exception (see roboplan::Scene).
class ArmPlanner {
 public:
  using Options = ArmPlannerOptions;

  struct PlanToAny {
    std::vector<Eigen::VectorXd> path;
    /// Index into the pose list of the pose `path` reaches.
    std::size_t index;
  };

  /// @throws std::runtime_error if the URDF cannot be rewritten/loaded or the group cannot be
  /// built.
  explicit ArmPlanner(const std::string& urdf_xml, const Options& options = {});
  ~ArmPlanner();

  ArmPlanner(const ArmPlanner&) = delete;
  ArmPlanner& operator=(const ArmPlanner&) = delete;

  /// @brief The kArmJoints positions of a JointState, in kArmJoints order.
  /// @return The positions, or an error listing the missing arm joints.
  [[nodiscard]] static tl::expected<Eigen::VectorXd, std::string> armPositions(
      const sensor_msgs::msg::JointState& joint_state);

  /// @brief The scene's full position vector with the arm slots replaced by `q`.
  [[nodiscard]] Eigen::VectorXd fullQ(const Eigen::VectorXd& q) const;

  /// @brief `q` clamped into the joint limits.
  [[nodiscard]] Eigen::VectorXd clamp(const Eigen::VectorXd& q) const;

  [[nodiscard]] bool hasCollisions(const Eigen::VectorXd& q) const;

  /// @brief Pose of `frame` (empty: the tip frame) in the base frame.
  [[nodiscard]] Eigen::Matrix4d fk(const Eigen::VectorXd& q, std::string_view frame = {}) const;

  /// @brief Collision-free IK for the tip at `tform`, or nullopt.
  /// @param local Disables random restarts, so the solution stays on the seed's IK branch.
  [[nodiscard]] std::optional<Eigen::VectorXd> ik(const Eigen::Matrix4d& tform,
                                                  const Eigen::VectorXd& q_seed,
                                                  bool local = false);

  /// @brief Collision-free joint path from `q_start` to `q_goal` (RRT-Connect, shortcut).
  ///
  /// `q_start` is clamped into the joint limits first: a measured joint state can sit a hair past
  /// a limit (joint 6 reads 3.1416 against a 3.14159 limit after a move to it), and RRT rejects
  /// such a start outright.
  [[nodiscard]] tl::expected<std::vector<Eigen::VectorXd>, std::string> planJoint(
      const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal);

  /// @brief Plans to the first reachable pose of `tforms`, in order of preference.
  ///
  /// roboplan 0.7 has no multi-goal RRT, so each pose is solved by IK and planned to in turn.
  [[nodiscard]] tl::expected<PlanToAny, std::string> planToAny(
      const Eigen::VectorXd& q_start, const std::vector<Eigen::Matrix4d>& tforms);

  /// @brief Short straight move: IK seeded at `q_start`, then a collision-checked joint lerp.
  ///
  /// Over a few centimetres the TCP stays close to a line. Fails if IK lands on another branch
  /// (any joint moves more than `max_joint_jump` rad) or any step collides.
  [[nodiscard]] tl::expected<std::vector<Eigen::VectorXd>, std::string> planLinear(
      const Eigen::VectorXd& q_start, const Eigen::Matrix4d& tform, int steps = 10,
      double max_joint_jump = 1.0);

  /// @brief Adds (or replaces) a box obstacle with full side lengths `size`, posed in the base
  /// frame.
  void addBox(const std::string& name, const Eigen::Vector3d& size,
              const Eigen::Matrix4d& tform_in_base,
              const Eigen::Vector4d& rgba = {0.6, 0.6, 0.6, 1.0});

  [[nodiscard]] bool hasBox(const std::string& name) const;

  void remove(const std::string& name);

  /// @brief Rigidly attaches box `name` to the wrist where it is at configuration `q`.
  void attach(const std::string& name, const Eigen::VectorXd& q);

  /// @brief Leaves box `name` where it is at configuration `q`, fixed in the base frame.
  void detach(const std::string& name, const Eigen::VectorXd& q);

  /// @brief The underlying scene, for thread-safe FK with a caller-owned `pinocchio::Data`.
  [[nodiscard]] const roboplan::Scene& scene() const;

  [[nodiscard]] const Options& options() const { return options_; }

 private:
  struct BoxEntry {
    Eigen::Vector3d size;
    Eigen::Vector4d rgba;
    std::string frame;
    Eigen::Matrix4d tform;  // Box pose in `frame`.
  };

  void addEntry(const std::string& name, const BoxEntry& box);
  void move(const std::string& name, const std::string& frame, const Eigen::Matrix4d& tform);

  Options options_;
  std::shared_ptr<roboplan::Scene> scene_;
  Eigen::VectorXd q_full_;
  Eigen::VectorXi arm_idx_;
  std::map<std::string, BoxEntry> boxes_;
  std::unique_ptr<roboplan::SimpleIk> ik_;
  std::unique_ptr<roboplan::SimpleIk> ik_local_;
  std::unique_ptr<roboplan::RRT> rrt_;
  std::unique_ptr<roboplan::PathShortcutter> shortcutter_;
};

}  // namespace emma_manipulation
