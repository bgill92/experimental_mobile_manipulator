// pick_and_place: runs the beet pick-and-place tree against the sim at 5 Hz and logs the tree
// and the scene to Rerun. Exits 0 if the tree succeeds, 1 otherwise.

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include <beet/beet.hpp>
#include <beet/observe.hpp>
#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_path.hpp>
#include <emma_manipulation/config.hpp>
#include <emma_manipulation/constants.hpp>
#include <emma_manipulation/planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <tl/expected.hpp>

#include "emma_behaviors/leaves.hpp"
#include "emma_behaviors/rerun_tree.hpp"
#include "emma_behaviors/ros_io.hpp"
#include "emma_behaviors/tree.hpp"
#include "emma_behaviors/viz.hpp"

namespace emma_behaviors {
namespace {

constexpr std::string_view kUsage =
    "usage: pick_and_place [--save FILE.rrd] [--ros-args ...]\n"
    "Runs the pick-and-place tree against the sim. Logs to a spawned Rerun viewer, or to\n"
    "FILE.rrd with --save (replay with `pixi run rerun FILE.rrd`). Exits 0 if the tree\n"
    "succeeds, 1 otherwise.\n";

struct Args {
  std::optional<std::filesystem::path> save;
};

[[nodiscard]] tl::expected<Args, std::string> parseArgs(const std::vector<std::string>& argv) {
  Args args;
  for (std::size_t i = 1; i < argv.size(); ++i) {
    if (argv[i] == "-h" || argv[i] == "--help") {
      return tl::make_unexpected(std::string());
    }
    if (argv[i] == "--save" && i + 1 < argv.size()) {
      args.save = argv[++i];
    } else {
      return tl::make_unexpected("unknown argument: " + argv[i]);
    }
  }
  return args;
}

[[nodiscard]] std::string_view statusName(beet::NodeStatus status) {
  switch (status) {
    case beet::NodeStatus::Idle:
      return "idle";
    case beet::NodeStatus::Running:
      return "RUNNING";
    case beet::NodeStatus::Success:
      return "success";
    case beet::NodeStatus::Failure:
      return "FAILURE";
    case beet::NodeStatus::Halted:
      return "halted";
  }
  return "?";
}

// One line per node, indented by depth: the final tree for pick-check's log.
void printTree(const beet::StatusTable<Tree>& table) {
  constexpr auto& nodes = beet::tree_info<Tree>;
  std::vector<int> depth(nodes.size(), 0);
  for (std::uint32_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].parent != beet::no_parent) {
      depth[i] = depth[nodes[i].parent] + 1;
    }
    const std::string_view name = nodes[i].label.empty() ? nodes[i].kind : nodes[i].label;
    std::cout << std::string(static_cast<std::size_t>(2 * depth[i]), ' ') << name << " ["
              << statusName(table.status(i)) << "]\n";
  }
}

[[nodiscard]] int run(const Args& args) {
  const auto node = std::make_shared<rclcpp::Node>("pick_and_place");
  const rclcpp::Logger log = node->get_logger();
  // Before waiting on the sim, so a bad YAML fails at once.
  const tl::expected<emma_manipulation::ArmConfig, std::string> config =
      emma_manipulation::loadArmConfig(*node);
  if (!config.has_value()) {
    RCLCPP_ERROR(log, "%s", config.error().c_str());
    return 1;
  }

  const tl::expected<std::string, std::string> urdf = waitForUrdf(*node, std::chrono::seconds(30));
  if (!urdf.has_value()) {
    RCLCPP_ERROR(log, "%s", urdf.error().c_str());
    return 1;
  }
  const auto planner = std::make_shared<emma_manipulation::ArmPlanner>(*urdf);
  const Eigen::Vector3d table_size = config->table_size;
  const Eigen::Matrix4d table_tform = translation(config->table_center);
  const Eigen::Vector4d table_rgba(0.55, 0.4, 0.25, 1.0);
  planner->addBox("table", table_size, table_tform, table_rgba);

  const tl::expected<std::shared_ptr<Io>, std::string> io = openIo(node);
  if (!io.has_value()) {
    RCLCPP_ERROR(log, "%s", io.error().c_str());
    return 1;
  }

  const auto viz = std::make_shared<Viz>(args.save);
  viz->setTick(0, 0.0);
  if (const auto planning_urdf = emma_manipulation::rewriteUrdfForPlanning(*urdf);
      planning_urdf.has_value()) {
    viz->logUrdf(planning_urdf->xml, emma_manipulation::meshPackagePaths());
  } else {
    RCLCPP_WARN(log, "not logging the URDF to Rerun: %s", planning_urdf.error().c_str());
  }
  viz->logBox("table", table_size, table_tform, table_rgba);

  RobotLogger robot(*viz, planner->scene());
  const Cell cell{*io, planner, viz,
                 std::make_shared<const emma_manipulation::ArmConfig>(*config)};
  const rerun_tree::GraphLogger<Tree> graph(viz->stream(), "tree");
  beet::StatusTable<Tree> table;
  beet::Runner runner{makeTree(cell), cell, table.listeners()};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const auto start = std::chrono::steady_clock::now();
  auto next = start;
  beet::Status status = beet::Status::Running;
  std::int64_t tick = 0;
  while (status == beet::Status::Running && rclcpp::ok()) {
    const std::chrono::duration<double> wall = std::chrono::steady_clock::now() - start;
    viz->setTick(tick, wall.count());
    executor.spin_some();
    {
      const std::scoped_lock lock((*io)->joint_state_mutex);
      if ((*io)->joint_state.has_value()) {
        robot.log(*(*io)->joint_state);
      }
    }
    status = runner.tick();
    std::ignore = graph.log(table);
    ++tick;
    next += kTickPeriod;
    std::this_thread::sleep_until(next);
  }

  printTree(table);
  if (status == beet::Status::Success) {
    RCLCPP_INFO(log, "pick and place succeeded after %ld ticks", static_cast<long>(tick));
    return 0;
  }
  RCLCPP_ERROR(log, "pick and place %s after %ld ticks",
               status == beet::Status::Failure ? "failed" : "was interrupted",
               static_cast<long>(tick));
  return 1;
}

}  // namespace
}  // namespace emma_behaviors

int main(int argc, char** argv) {
  const std::vector<std::string> argv_clean = rclcpp::remove_ros_arguments(argc, argv);
  const auto args = emma_behaviors::parseArgs(argv_clean);
  if (!args.has_value()) {
    // An empty error means --help.
    if (args.error().empty()) {
      std::cout << emma_behaviors::kUsage;
      return 0;
    }
    std::cerr << args.error() << "\n" << emma_behaviors::kUsage;
    return 2;
  }
  // The default parameters file goes first, so a --params-file or -p on the command line
  // overrides it.
  const std::string params =
      (ament_index_cpp::get_package_share_path("emma_manipulation") / "config/manipulation.yaml")
          .string();
  std::vector<const char*> ros_argv = {argv[0], "--ros-args", "--params-file", params.c_str(),
                                       "--"};
  ros_argv.insert(ros_argv.end(), argv + 1, argv + argc);
  rclcpp::init(static_cast<int>(ros_argv.size()), ros_argv.data());
  const int code = emma_behaviors::run(*args);
  rclcpp::shutdown();
  return code;
}
