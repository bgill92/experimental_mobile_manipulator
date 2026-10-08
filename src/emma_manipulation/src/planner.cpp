#include "emma_manipulation/planner.hpp"

#include <algorithm>
#include <array>
#include <set>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <roboplan/core/geometry_wrappers.hpp>
#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_rrt/rrt.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>
#include <tinyxml2.h>

namespace emma_manipulation {
namespace {

constexpr std::string_view kGroup = "arm";

// Packages whose package:// meshes the URDF references.
constexpr std::array<std::string_view, 3> kMeshPackages = {"emma_description",
                                                           "mycobot_description",
                                                           "myagv_description"};

// Filename fragment -> mesh scale; see rewriteUrdfForPlanning for why.
constexpr std::array<std::pair<std::string_view, const char*>, 2> kMeshScales = {{
    {"parallel_gripper/", "0.001 0.001 0.001"},  // Millimetres.
    {"myagv_", "0.0254 0.0254 0.0254"},          // Inches.
}};

// Link pairs that never need checking: the fingers only slide relative to the wrist, and the
// wrist camera is bolted to the gripper next to it.
constexpr std::array<std::string_view, 3> kWristParts = {"gripper_left", "gripper_right",
                                                         "wrist_camera_link"};
constexpr std::array<std::string_view, 2> kWristLinks = {"joint6", "joint6_flange"};

// roboplan places added geometry relative to the parent frame's *joint*, not the frame itself, so
// held objects hang off the link of the last arm joint (whose frame is that joint's frame) rather
// than off tcp. base_link coincides with the root, so obstacles in base_link are unaffected.
constexpr std::string_view kAttachLink = "joint6_flange";

// Links that may touch an object held in the gripper.
constexpr std::array<std::string_view, 3> kGripperLinks = {"gripper_base", "gripper_left",
                                                           "gripper_right"};

// Every element named `name` below `parent`, depth first, like Python's Element.iter().
void collectElements(tinyxml2::XMLElement* parent, const std::string_view name,
                     std::vector<tinyxml2::XMLElement*>& out) {
  for (tinyxml2::XMLElement* child = parent->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement()) {
    if (name == child->Name()) {
      out.push_back(child);
    }
    collectElements(child, name, out);
  }
}

[[nodiscard]] std::vector<tinyxml2::XMLElement*> elements(tinyxml2::XMLElement* root,
                                                          const std::string_view name) {
  std::vector<tinyxml2::XMLElement*> out;
  collectElements(root, name, out);
  return out;
}

[[nodiscard]] std::string_view attributeOr(const tinyxml2::XMLElement* element,
                                           const char* attribute) {
  const char* value = element->Attribute(attribute);
  return value == nullptr ? std::string_view{} : std::string_view{value};
}

// roboplan's setters only fail on unknown names, i.e. programming errors here.
void check(const tl::expected<void, std::string>& result, const std::string_view what) {
  if (!result.has_value()) {
    throw std::runtime_error(std::string(what) + ": " + result.error());
  }
}

}  // namespace

tl::expected<PlanningUrdf, std::string> rewriteUrdfForPlanning(const std::string& urdf_xml) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_xml.c_str(), urdf_xml.size()) != tinyxml2::XML_SUCCESS) {
    return tl::unexpected(std::string("URDF does not parse: ") + doc.ErrorStr());
  }
  tinyxml2::XMLElement* const robot = doc.RootElement();
  if (robot == nullptr || std::string_view(robot->Name()) != "robot") {
    return tl::unexpected(std::string("URDF root element is not <robot>"));
  }

  for (tinyxml2::XMLElement* const joint : elements(robot, "joint")) {
    while (tinyxml2::XMLElement* const mimic = joint->FirstChildElement("mimic")) {
      joint->DeleteChild(mimic);
    }
  }

  PlanningUrdf out;
  for (tinyxml2::XMLElement* const link : elements(robot, "link")) {
    if (const char* name = link->Attribute("name"); name != nullptr) {
      out.link_names.emplace_back(name);
    }
    const tinyxml2::XMLElement* const visual = link->FirstChildElement("visual");
    if (visual == nullptr || link->FirstChildElement("collision") != nullptr) {
      continue;
    }
    const tinyxml2::XMLElement* const geometry = visual->FirstChildElement("geometry");
    const tinyxml2::XMLElement* const mesh =
        geometry == nullptr ? nullptr : geometry->FirstChildElement("mesh");
    if (mesh == nullptr || attributeOr(mesh, "filename").find("myagv_") == std::string_view::npos) {
      continue;
    }
    tinyxml2::XMLElement* const collision = doc.NewElement("collision");
    for (const tinyxml2::XMLElement* child = visual->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
      if (const std::string_view tag = child->Name(); tag == "origin" || tag == "geometry") {
        collision->InsertEndChild(child->DeepClone(&doc));
      }
    }
    link->InsertEndChild(collision);
  }

  for (tinyxml2::XMLElement* const mesh : elements(robot, "mesh")) {
    const std::string_view filename = attributeOr(mesh, "filename");
    for (const auto& [key, scale] : kMeshScales) {
      if (filename.find(key) != std::string_view::npos) {
        mesh->SetAttribute("scale", scale);
      }
    }
  }

  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  out.xml = printer.CStr();
  return out;
}

std::vector<std::filesystem::path> meshPackagePaths() {
  std::vector<std::filesystem::path> paths;
  for (const std::string_view pkg : kMeshPackages) {
    // package://pkg/... resolves against the directory holding the package share dir.
    paths.push_back(
        std::filesystem::path(ament_index_cpp::get_package_share_directory(std::string(pkg)))
            .parent_path());
  }
  return paths;
}

ArmPlanner::ArmPlanner(const std::string& urdf_xml, const Options& options) : options_(options) {
  const tl::expected<PlanningUrdf, std::string> urdf = rewriteUrdfForPlanning(urdf_xml);
  if (!urdf.has_value()) {
    throw std::runtime_error(urdf.error());
  }
  const std::vector<std::filesystem::path> mesh_paths = meshPackagePaths();
  // Pinocchio and coal report a bad model with logic_error subclasses; rethrow them as the
  // documented runtime_error so callers need only one handler.
  try {
    scene_ = std::make_shared<roboplan::Scene>(
        "emma", roboplan::loadUrdfSceneDescriptionFromXml(urdf->xml, mesh_paths));
  } catch (const std::invalid_argument& e) {
    throw std::runtime_error(std::string("cannot load the URDF into roboplan: ") + e.what());
  } catch (const std::out_of_range& e) {
    throw std::runtime_error(std::string("cannot load the URDF into roboplan: ") + e.what());
  }
  check(scene_->addGroupFromChain(std::string(kGroup), "g_base", options_.tip_frame),
        "addGroupFromChain");
  check(scene_->allowAdjacentLinkCollisions(), "allowAdjacentLinkCollisions");

  // Pairs naming a link the URDF lacks (no camera) are skipped.
  const std::set<std::string, std::less<>> links(urdf->link_names.begin(),
                                                 urdf->link_names.end());
  std::vector<std::pair<std::string, std::string>> disabled;
  for (const std::string_view part : kWristParts) {
    for (const std::string_view wrist : kWristLinks) {
      if (links.contains(part) && links.contains(wrist)) {
        disabled.emplace_back(part, wrist);
      }
    }
  }
  check(scene_->setCollisions(disabled, false), "setCollisions");
  if (options_.seed.has_value()) {
    scene_->setRngSeed(*options_.seed);
  }

  q_full_ = scene_->getCurrentJointPositions();
  arm_idx_ = scene_->getJointPositionIndices(kArmJoints);

  const std::string group(kGroup);
  ik_ = std::make_unique<roboplan::SimpleIk>(
      scene_, roboplan::SimpleIkOptions{.group_name = group,
                                        .max_iters = 200,
                                        .max_time = 0.05,
                                        .max_restarts = 5,
                                        .check_collisions = true});
  // Linear moves must stay on the seed's IK branch, so no random restarts.
  ik_local_ = std::make_unique<roboplan::SimpleIk>(
      scene_, roboplan::SimpleIkOptions{.group_name = group,
                                        .max_iters = 200,
                                        .max_time = 0.05,
                                        .max_restarts = 0,
                                        .check_collisions = true});
  rrt_ = std::make_unique<roboplan::RRT>(
      scene_, roboplan::RRTOptions{.group_name = group,
                                   .max_connection_distance = 1.0,
                                   .collision_check_step_size = 0.02,
                                   .max_planning_time = options_.max_planning_time,
                                   .rrt_connect = true});
  shortcutter_ = std::make_unique<roboplan::PathShortcutter>(
      scene_, roboplan::PathShortcuttingOptions{
                  .group_name = group,
                  .max_step_size = 0.02,
                  .max_iters = 200,
                  .seed = options_.seed.has_value() ? static_cast<int>(*options_.seed) : -1});
  if (options_.seed.has_value()) {
    ik_->setRngSeed(*options_.seed);
    rrt_->setRngSeed(*options_.seed);
  }
}

ArmPlanner::~ArmPlanner() = default;

tl::expected<Eigen::VectorXd, std::string> ArmPlanner::armPositions(
    const sensor_msgs::msg::JointState& joint_state) {
  std::unordered_map<std::string_view, double> by_name;
  const std::size_t count = std::min(joint_state.name.size(), joint_state.position.size());
  for (std::size_t i = 0; i < count; ++i) {
    by_name.emplace(joint_state.name[i], joint_state.position[i]);
  }
  Eigen::VectorXd q(static_cast<Eigen::Index>(kArmJoints.size()));
  std::string missing;
  for (std::size_t i = 0; i < kArmJoints.size(); ++i) {
    if (const auto it = by_name.find(kArmJoints[i]); it != by_name.end()) {
      q(static_cast<Eigen::Index>(i)) = it->second;
    } else {
      missing += (missing.empty() ? "" : ", ") + kArmJoints[i];
    }
  }
  if (!missing.empty()) {
    return tl::unexpected("JointState lacks arm joints: " + missing);
  }
  return q;
}

Eigen::VectorXd ArmPlanner::fullQ(const Eigen::VectorXd& q) const {
  Eigen::VectorXd full = q_full_;
  for (Eigen::Index i = 0; i < arm_idx_.size(); ++i) {
    full(arm_idx_(i)) = q(i);
  }
  return full;
}

Eigen::VectorXd ArmPlanner::clamp(const Eigen::VectorXd& q) const {
  const Eigen::VectorXd full = scene_->clampToValidConfiguration(fullQ(q));
  return full(arm_idx_);
}

bool ArmPlanner::hasCollisions(const Eigen::VectorXd& q) const {
  return scene_->hasCollisions(fullQ(q));
}

Eigen::Matrix4d ArmPlanner::fk(const Eigen::VectorXd& q, const std::string_view frame) const {
  return scene_->forwardKinematics(fullQ(q),
                                   frame.empty() ? options_.tip_frame : std::string(frame),
                                   options_.base_frame);
}

std::optional<Eigen::VectorXd> ArmPlanner::ik(const Eigen::Matrix4d& tform,
                                              const Eigen::VectorXd& q_seed, const bool local) {
  const roboplan::CartesianConfiguration goal{options_.base_frame, options_.tip_frame, tform};
  const roboplan::JointConfiguration start{.joint_names = kArmJoints, .positions = q_seed};
  roboplan::JointConfiguration solution;
  roboplan::SimpleIk& solver = local ? *ik_local_ : *ik_;
  if (!solver.solveIk(goal, start, solution)) {
    return std::nullopt;
  }
  return solution.positions;
}

tl::expected<std::vector<Eigen::VectorXd>, std::string> ArmPlanner::planJoint(
    const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal) {
  if (hasCollisions(q_goal)) {
    return tl::unexpected(std::string("goal configuration is in collision"));
  }
  const roboplan::JointConfiguration start{.joint_names = kArmJoints, .positions = clamp(q_start)};
  const roboplan::JointConfiguration goal{.joint_names = kArmJoints, .positions = q_goal};
  const tl::expected<roboplan::JointPath, std::string> path = rrt_->plan(start, goal);
  if (!path.has_value()) {
    return tl::unexpected("RRT failed: " + path.error());
  }
  if (path->positions.empty()) {
    return tl::unexpected(std::string("RRT returned an empty path"));
  }
  return shortcutter_->shortcut(*path).positions;
}

tl::expected<ArmPlanner::PlanToAny, std::string> ArmPlanner::planToAny(
    const Eigen::VectorXd& q_start, const std::vector<Eigen::Matrix4d>& tforms) {
  for (std::size_t index = 0; index < tforms.size(); ++index) {
    const std::optional<Eigen::VectorXd> q_goal = ik(tforms[index], q_start);
    if (!q_goal.has_value()) {
      continue;
    }
    if (tl::expected<std::vector<Eigen::VectorXd>, std::string> path = planJoint(q_start, *q_goal);
        path.has_value()) {
      return PlanToAny{.path = std::move(*path), .index = index};
    }
  }
  return tl::unexpected(std::string("no candidate reachable"));
}

tl::expected<std::vector<Eigen::VectorXd>, std::string> ArmPlanner::planLinear(
    const Eigen::VectorXd& q_start, const Eigen::Matrix4d& tform, const int steps,
    const double max_joint_jump) {
  const Eigen::VectorXd q0 = clamp(q_start);
  const std::optional<Eigen::VectorXd> q_goal = ik(tform, q0, /*local=*/true);
  if (!q_goal.has_value()) {
    return tl::unexpected(std::string("no IK solution near the start"));
  }
  if (const double jump = (*q_goal - q0).cwiseAbs().maxCoeff(); jump > max_joint_jump) {
    return tl::unexpected("IK switched branch (a joint moves " + std::to_string(jump) + " rad)");
  }
  std::vector<Eigen::VectorXd> path;
  path.reserve(static_cast<std::size_t>(steps) + 1);
  for (int i = 0; i <= steps; ++i) {
    path.push_back(q0 + (*q_goal - q0) * (static_cast<double>(i) / steps));
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (hasCollisions(path[i])) {
      return tl::unexpected("straight move collides at step " + std::to_string(i));
    }
  }
  return path;
}

void ArmPlanner::addBox(const std::string& name, const Eigen::Vector3d& size,
                        const Eigen::Matrix4d& tform_in_base, const Eigen::Vector4d& rgba) {
  if (hasBox(name)) {
    remove(name);
  }
  addEntry(name, BoxEntry{.size = size,
                          .rgba = rgba,
                          .frame = options_.base_frame,
                          .tform = tform_in_base});
}

bool ArmPlanner::hasBox(const std::string& name) const { return boxes_.contains(name); }

void ArmPlanner::remove(const std::string& name) {
  check(scene_->removeGeometry(name), "removeGeometry");
  boxes_.erase(name);
}

void ArmPlanner::attach(const std::string& name, const Eigen::VectorXd& q) {
  const BoxEntry& box = boxes_.at(name);
  const Eigen::Matrix4d t_box_base = fk(q, box.frame) * box.tform;
  const Eigen::Matrix4d t_link = fk(q, kAttachLink);
  move(name, std::string(kAttachLink), t_link.inverse() * t_box_base);
  std::vector<std::pair<std::string, std::string>> pairs;
  for (const std::string_view link : kGripperLinks) {
    pairs.emplace_back(name, link);
  }
  check(scene_->setCollisions(pairs, false), "setCollisions");
}

void ArmPlanner::detach(const std::string& name, const Eigen::VectorXd& q) {
  const BoxEntry& box = boxes_.at(name);
  const Eigen::Matrix4d t_box_base = fk(q, box.frame) * box.tform;
  move(name, options_.base_frame, t_box_base);
}

const roboplan::Scene& ArmPlanner::scene() const { return *scene_; }

void ArmPlanner::addEntry(const std::string& name, const BoxEntry& box) {
  check(scene_->addBoxGeometry(name, box.frame,
                               roboplan::Box(box.size.x(), box.size.y(), box.size.z()), box.tform,
                               box.rgba),
        "addBoxGeometry");
  boxes_.insert_or_assign(name, box);
  // Objects resting on each other are not collisions.
  std::vector<std::pair<std::string, std::string>> others;
  for (const auto& [other, entry] : boxes_) {
    if (other != name) {
      others.emplace_back(name, other);
    }
  }
  if (!others.empty()) {
    check(scene_->setCollisions(others, false), "setCollisions");
  }
}

void ArmPlanner::move(const std::string& name, const std::string& frame,
                      const Eigen::Matrix4d& tform) {
  BoxEntry box = boxes_.at(name);
  remove(name);
  box.frame = frame;
  box.tform = tform;
  addEntry(name, box);
}

}  // namespace emma_manipulation
