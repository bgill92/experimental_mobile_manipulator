// Plans and runs one arm motion from the command line. The usage text below is the reference.

#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <roboplan/core/pose_utils.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tl/expected.hpp>

#include "emma_manipulation/constants.hpp"
#include "emma_manipulation/planner.hpp"
#include "emma_manipulation/trajectory.hpp"

namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using namespace std::chrono_literals;

constexpr std::string_view kAction = "/arm_controller/follow_joint_trajectory";

constexpr std::string_view kUsage =
    R"(usage: move_arm [-h]
                (--home | --joints Q Q Q Q Q Q | --pose V [V ...] | --linear V [V ...])
                [--no-table] [--v-max V_MAX]

Plan and run one arm motion from the command line.

    ros2 run emma_manipulation move_arm --home
    ros2 run emma_manipulation move_arm --joints 0 0 0 0 0 0
    ros2 run emma_manipulation move_arm --pose 0.25 0 0.20 0 3.1416 0
    ros2 run emma_manipulation move_arm --linear 0.25 0 0.18 0 3.1416 0

Poses are the TCP in base_link: x y z in metres, then optional roll pitch yaw in radians
(fixed axes, applied x then y then z; default 0 0 0). --pose plans around obstacles with
RRT; --linear moves in a short straight line from the current pose. The table from
emma_manipulation/constants.hpp is in the planning scene unless --no-table is given.

options:
  -h, --help            show this help message and exit
  --home                go to HOME_Q
  --joints Q Q Q Q Q Q  arm joint positions in rad
  --pose V [V ...]      TCP pose x y z [roll pitch yaw], planned with RRT
  --linear V [V ...]    TCP pose x y z [roll pitch yaw], straight-line move
  --no-table            leave the table out of the planning scene
  --v-max V_MAX         max joint speed in rad/s (default 0.8)
)";

enum class Mode { kHome, kJoints, kPose, kLinear };

struct Args {
  Mode mode = Mode::kHome;
  std::vector<double> values;
  bool no_table = false;
  double v_max = 0.8;
  bool help = false;
};

[[nodiscard]] tl::expected<double, std::string> tryParseDouble(std::string_view text) {
  double value = 0.0;
  const char* const end = text.data() + text.size();
  if (const auto [ptr, ec] = std::from_chars(text.data(), end, value);
      ec != std::errc{} || ptr != end) {
    return tl::unexpected(std::format("not a number: '{}'", text));
  }
  return value;
}

[[nodiscard]] bool isFlag(std::string_view arg) {
  // A leading '-' followed by a digit or '.' is a negative number, not a flag.
  return arg.size() > 1 && arg[0] == '-' && !(std::isdigit(static_cast<unsigned char>(arg[1])) != 0 || arg[1] == '.');
}

/// Parses the arguments after the program name, mirroring the old argparse interface.
[[nodiscard]] tl::expected<Args, std::string> parseArgs(const std::vector<std::string>& argv) {
  Args args;
  std::optional<std::string_view> target;
  for (std::size_t i = 0; i < argv.size(); ++i) {
    const std::string_view arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      args.help = true;
      return args;
    }
    if (arg == "--no-table") {
      args.no_table = true;
      continue;
    }
    if (arg == "--v-max") {
      if (i + 1 >= argv.size()) {
        return tl::unexpected("argument --v-max: expected one argument");
      }
      const tl::expected<double, std::string> v = tryParseDouble(argv[++i]);
      if (!v.has_value()) {
        return tl::unexpected("argument --v-max: " + v.error());
      }
      if (*v <= 0.0) {
        return tl::unexpected("argument --v-max: must be positive");
      }
      args.v_max = *v;
      continue;
    }
    Mode mode = Mode::kHome;
    if (arg == "--home") {
      mode = Mode::kHome;
    } else if (arg == "--joints") {
      mode = Mode::kJoints;
    } else if (arg == "--pose") {
      mode = Mode::kPose;
    } else if (arg == "--linear") {
      mode = Mode::kLinear;
    } else {
      return tl::unexpected(std::format("unrecognized argument: {}", arg));
    }
    if (target.has_value()) {
      return tl::unexpected(std::format("argument {}: not allowed with argument {}", arg, *target));
    }
    target = arg;
    args.mode = mode;
    while (mode != Mode::kHome && i + 1 < argv.size() && !isFlag(argv[i + 1])) {
      const tl::expected<double, std::string> v = tryParseDouble(argv[++i]);
      if (!v.has_value()) {
        return tl::unexpected(std::format("argument {}: {}", arg, v.error()));
      }
      args.values.push_back(*v);
    }
    if (mode == Mode::kJoints && args.values.size() != emma_manipulation::kArmJoints.size()) {
      return tl::unexpected("argument --joints: expected 6 arguments");
    }
    if ((mode == Mode::kPose || mode == Mode::kLinear) && args.values.size() != 3 &&
        args.values.size() != 6) {
      return tl::unexpected(std::format("argument {}: a pose is x y z [roll pitch yaw]", arg));
    }
  }
  if (!target.has_value()) {
    return tl::unexpected(
        "one of the arguments --home --joints --pose --linear is required");
  }
  return args;
}

/// A 4x4 from x y z [roll pitch yaw], fixed axes applied x then y then z.
[[nodiscard]] Eigen::Matrix4d poseToTform(const std::vector<double>& v) {
  const double roll = v.size() == 6 ? v[3] : 0.0;
  const double pitch = v.size() == 6 ? v[4] : 0.0;
  const double yaw = v.size() == 6 ? v[5] : 0.0;
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.topLeftCorner<3, 3>() = rotation;
  tform.topRightCorner<3, 1>() = Eigen::Vector3d(v[0], v[1], v[2]);
  return tform;
}

/// Reads the robot model and joint state, and runs trajectories on arm_controller.
class MoveArm {
 public:
  MoveArm() : node_(std::make_shared<rclcpp::Node>("move_arm")) {
    urdf_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String& msg) { urdf_ = msg.data; });
    joints_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](const sensor_msgs::msg::JointState& msg) {
          if (auto q = emma_manipulation::ArmPlanner::armPositions(msg); q.has_value()) {
            arm_q_ = std::move(*q);
          }
        });
    client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(node_, std::string(kAction));
    executor_.add_node(node_);
  }

  [[nodiscard]] rclcpp::Logger logger() const { return node_->get_logger(); }

  /// Wall-clock waits: with use_sim_time the node clock stands still until /clock arrives.
  [[nodiscard]] tl::expected<std::string, std::string> waitForUrdf(
      const std::chrono::seconds timeout = 10s) {
    if (!spinUntil([this] { return urdf_.has_value(); }, timeout)) {
      return tl::unexpected("no message on /robot_description");
    }
    return *urdf_;
  }

  /// Waits for an arm joint sample newer than the call.
  [[nodiscard]] tl::expected<Eigen::VectorXd, std::string> freshArmPositions(
      const std::chrono::seconds timeout = 5s) {
    arm_q_.reset();
    if (!spinUntil([this] { return arm_q_.has_value(); }, timeout)) {
      return tl::unexpected("no arm joints on /joint_states");
    }
    return *arm_q_;
  }

  [[nodiscard]] tl::expected<void, std::string> execute(
      const FollowJointTrajectory::Goal& goal, const std::chrono::seconds timeout = 60s) {
    if (!client_->wait_for_action_server(10s)) {
      return tl::unexpected(std::format("{} not available", kAction));
    }
    auto send = client_->async_send_goal(goal);
    if (executor_.spin_until_future_complete(send, 10s) != rclcpp::FutureReturnCode::SUCCESS) {
      return tl::unexpected("no reply to the trajectory goal");
    }
    const auto handle = send.get();
    if (!handle) {
      return tl::unexpected("trajectory goal rejected");
    }
    auto result_future = client_->async_get_result(handle);
    if (executor_.spin_until_future_complete(result_future, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return tl::unexpected("timed out waiting for the trajectory to finish");
    }
    const auto wrapped = result_future.get();
    const std::int32_t code = wrapped.result ? wrapped.result->error_code : -1;
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
        code != FollowJointTrajectory::Result::SUCCESSFUL) {
      const std::string why = wrapped.result ? wrapped.result->error_string : "";
      return tl::unexpected(std::format("trajectory failed ({}): {}", code, why));
    }
    return {};
  }

 private:
  [[nodiscard]] bool spinUntil(const std::function<bool()>& done,
                               const std::chrono::seconds timeout) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (!done() && std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      executor_.spin_once(100ms);
    }
    return done();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  std::optional<std::string> urdf_;
  std::optional<Eigen::VectorXd> arm_q_;
};

[[nodiscard]] int run(MoveArm& move_arm, const Args& args) {
  const rclcpp::Logger log = move_arm.logger();
  const tl::expected<std::string, std::string> urdf = move_arm.waitForUrdf();
  if (!urdf.has_value()) {
    RCLCPP_ERROR(log, "%s", urdf.error().c_str());
    return 1;
  }
  emma_manipulation::ArmPlanner planner(*urdf);
  if (!args.no_table) {
    Eigen::Matrix4d table = Eigen::Matrix4d::Identity();
    table.topRightCorner<3, 1>() = emma_manipulation::kTableCenter;
    planner.addBox("table", 2.0 * emma_manipulation::kTableHalfSize, table,
                   {0.55, 0.4, 0.25, 1.0});
  }
  const tl::expected<Eigen::VectorXd, std::string> q_start = move_arm.freshArmPositions();
  if (!q_start.has_value()) {
    RCLCPP_ERROR(log, "%s", q_start.error().c_str());
    return 1;
  }

  Eigen::Matrix4d target;
  tl::expected<std::vector<Eigen::VectorXd>, std::string> path;
  if (args.mode == Mode::kHome || args.mode == Mode::kJoints) {
    const Eigen::VectorXd q_goal =
        args.mode == Mode::kHome
            ? emma_manipulation::kHomeQ
            : Eigen::Map<const Eigen::VectorXd>(args.values.data(),
                                                static_cast<Eigen::Index>(args.values.size()));
    target = planner.fk(q_goal);
    path = planner.planJoint(*q_start, q_goal);
  } else {
    target = poseToTform(args.values);
    if (args.mode == Mode::kPose) {
      path = planner.planToAny(*q_start, {target}).map(
          [](emma_manipulation::ArmPlanner::PlanToAny found) { return std::move(found.path); });
    } else {
      path = planner.planLinear(*q_start, target);
    }
  }
  if (!path.has_value()) {
    RCLCPP_ERROR(log, "no collision-free plan found: %s", path.error().c_str());
    return 1;
  }
  RCLCPP_INFO(log, "planned %zu waypoints", path->size());

  FollowJointTrajectory::Goal goal;
  goal.trajectory = emma_manipulation::toJointTrajectory(*path, args.v_max);
  if (const tl::expected<void, std::string> done = move_arm.execute(goal); !done.has_value()) {
    RCLCPP_ERROR(log, "%s", done.error().c_str());
    return 1;
  }

  const tl::expected<Eigen::VectorXd, std::string> q_end = move_arm.freshArmPositions();
  if (!q_end.has_value()) {
    RCLCPP_ERROR(log, "%s", q_end.error().c_str());
    return 1;
  }
  const Eigen::Matrix4d reached = planner.fk(*q_end);
  const auto [pos_err, rot_err] = roboplan::poseError(reached, target);
  RCLCPP_INFO(log, "%s",
              std::format("TCP at [{:.4f} {:.4f} {:.4f}], error {:.1f} mm, {:.1f} deg",
                          reached(0, 3), reached(1, 3), reached(2, 3), pos_err * 1000.0,
                          rot_err * 180.0 / std::numbers::pi)
                  .c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> argv_clean = rclcpp::remove_ros_arguments(argc, argv);
  argv_clean.erase(argv_clean.begin());
  const tl::expected<Args, std::string> args = parseArgs(argv_clean);
  if (!args.has_value()) {
    std::cerr << kUsage.substr(0, kUsage.find("\n\n")) << "\nmove_arm: error: " << args.error()
              << "\n";
    return 1;
  }
  if (args->help) {
    std::cout << kUsage;
    return 0;
  }

  rclcpp::init(argc, argv);
  int code = 1;
  {
    MoveArm move_arm;
    try {
      code = run(move_arm, *args);
    } catch (const std::runtime_error& e) {
      // ArmPlanner throws on a URDF it cannot load.
      RCLCPP_ERROR(move_arm.logger(), "%s", e.what());
    } catch (const ament_index_cpp::PackageNotFoundError& e) {
      RCLCPP_ERROR(move_arm.logger(), "mesh package not found: %s", e.what());
    }
  }
  rclcpp::shutdown();
  return code;
}
