#pragma once

/// @file
/// Rerun logging for the pick and place. A pimpl, so only viz.cpp, the executable and the tree
/// graph logger include the (heavy) Rerun headers.

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rerun {
class RecordingStream;
}  // namespace rerun

namespace roboplan {
class Scene;
}  // namespace roboplan

namespace emma_behaviors {

/// @brief Scene and event logging to one Rerun recording. Every method except logUrdf is
/// thread-safe (Rerun recording streams are).
class Viz {
 public:
  enum class Level { kInfo, kWarning, kError };

  /// @brief Writes to `save` if given, else spawns a viewer. A failed spawn logs a warning to
  /// stderr and continues disconnected; a failed save throws std::runtime_error.
  explicit Viz(std::optional<std::filesystem::path> save);

  /// @brief A recording that drops everything, for tests.
  [[nodiscard]] static Viz disabled();

  ~Viz();
  Viz(Viz&&) noexcept;
  Viz& operator=(Viz&&) noexcept;
  Viz(const Viz&) = delete;
  Viz& operator=(const Viz&) = delete;

  /// @brief Logs the robot URDF at `world/robot`.
  ///
  /// The URDF is written to a temporary `.urdf` file (the extension selects Rerun's loader), and
  /// `ROS_PACKAGE_PATH` is set to `package_paths` so its `package://` meshes resolve.
  /// @warning Not thread-safe: it calls setenv and writes a fixed temporary path. Call it at
  /// startup, before any other thread logs.
  void logUrdf(const std::string& xml,
               const std::vector<std::filesystem::path>& package_paths) const;

  /// @brief Logs a box with full side lengths `size`, posed in `base_link`, at `base_link/<name>`.
  void logBox(std::string_view name, const Eigen::Vector3d& size, const Eigen::Matrix4d& tform,
              const Eigen::Vector4d& rgba) const;

  /// @brief Logs a polyline in `base_link` at `base_link/plan/<name>`.
  void logPath(std::string_view name, const std::vector<Eigen::Vector3d>& points) const;

  /// @brief Logs a text event at `log`.
  void logText(std::string_view text, Level level = Level::kInfo) const;

  /// @brief Sets the `tick` and `time` timelines for what is logged next on this thread.
  void setTick(std::int64_t tick, double seconds) const;

  [[nodiscard]] const rerun::RecordingStream& stream() const;

 private:
  struct Impl;
  explicit Viz(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

/// @brief Animates the URDF that Viz::logUrdf logged from joint states.
///
/// Uses its own `pinocchio::Data` with the scene's (immutable) model, so it never touches the
/// planner's scratch state and can run while a planning job is offloaded.
/// @warning Not thread-safe: the private `pinocchio::Data` is mutated by log().
class RobotLogger {
 public:
  RobotLogger(const Viz& viz, const roboplan::Scene& scene);
  ~RobotLogger();
  RobotLogger(const RobotLogger&) = delete;
  RobotLogger& operator=(const RobotLogger&) = delete;

  /// @brief Logs every joint transform at `world/robot/joints/<joint>`. Joints missing from
  /// `joint_state` stay at their default positions.
  void log(const sensor_msgs::msg::JointState& joint_state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace emma_behaviors
