#include "emma_behaviors/viz.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rerun.hpp>
#include <roboplan/core/scene.hpp>

namespace emma_behaviors {
namespace {

constexpr std::string_view kAppId = "emma_pick_and_place";
constexpr std::string_view kBaseLink = "base_link";

[[nodiscard]] rerun::Color toColor(const Eigen::Vector4d& rgba) {
  const auto channel = [](double v) {
    return static_cast<std::uint8_t>(std::lround(std::clamp(v, 0.0, 1.0) * 255.0));
  };
  return {channel(rgba[0]), channel(rgba[1]), channel(rgba[2]), channel(rgba[3])};
}

[[nodiscard]] rerun::Vec3D toVec(const Eigen::Vector3d& v) {
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

[[nodiscard]] rerun::Transform3D toTransform(const Eigen::Matrix4d& tform) {
  const Eigen::Matrix3f r = tform.topLeftCorner<3, 3>().cast<float>();
  const rerun::Vec3D columns[3] = {{r(0, 0), r(1, 0), r(2, 0)},
                                   {r(0, 1), r(1, 1), r(2, 1)},
                                   {r(0, 2), r(1, 2), r(2, 2)}};
  const Eigen::Vector3d p = tform.block<3, 1>(0, 3);
  return rerun::Transform3D::from_translation_mat3x3(toVec(p), columns);
}

}  // namespace

struct Viz::Impl {
  rerun::RecordingStream rec;
};

Viz::Viz(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Viz::Viz(std::optional<std::filesystem::path> save)
    : impl_(std::make_unique<Impl>(Impl{rerun::RecordingStream(kAppId)})) {
  if (save.has_value()) {
    if (const rerun::Error error = impl_->rec.save(save->string()); error.is_err()) {
      throw std::runtime_error("cannot save Rerun recording to " + save->string() + ": " +
                               error.description);
    }
    return;
  }
  if (const rerun::Error error = impl_->rec.spawn(); error.is_err()) {
    std::cerr << "warning: cannot spawn the Rerun viewer (" << error.description
              << "); continuing without it\n";
  }
}

Viz Viz::disabled() {
  // Rerun has no disabled-stream constructor; streams copy the global default when created.
  const bool was_enabled = rerun::is_default_enabled();
  rerun::set_default_enabled(false);
  auto impl = std::make_unique<Impl>(Impl{rerun::RecordingStream(kAppId)});
  rerun::set_default_enabled(was_enabled);
  return Viz(std::move(impl));
}

Viz::~Viz() = default;
Viz::Viz(Viz&&) noexcept = default;
Viz& Viz::operator=(Viz&&) noexcept = default;

void Viz::logUrdf(const std::string& xml,
                  const std::vector<std::filesystem::path>& package_paths) const {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "emma_pick_and_place.urdf";
  {
    std::ofstream file(path);
    file << xml;
  }
  std::string joined;
  for (const std::filesystem::path& package_path : package_paths) {
    joined += (joined.empty() ? "" : ":") + package_path.string();
  }
  // Rerun resolves package:// URIs through ROS_PACKAGE_PATH.
  setenv("ROS_PACKAGE_PATH", joined.c_str(), 1);
  impl_->rec.log_file_from_path(path, "world/robot", true);
}

void Viz::logBox(std::string_view name, const Eigen::Vector3d& size,
                 const Eigen::Matrix4d& tform, const Eigen::Vector4d& rgba) const {
  const Eigen::Quaterniond q(Eigen::Matrix3d(tform.topLeftCorner<3, 3>()));
  const Eigen::Vector3d center = tform.block<3, 1>(0, 3);
  // In base_link, the URDF link frame planning uses (the UR5 example logs to "world").
  impl_->rec.log(std::string(kBaseLink) + "/" + std::string(name),
                 rerun::Boxes3D::from_centers_and_half_sizes({toVec(center)}, {toVec(size / 2.0)})
                     .with_quaternions({rerun::Quaternion::from_xyzw(
                         static_cast<float>(q.x()), static_cast<float>(q.y()),
                         static_cast<float>(q.z()), static_cast<float>(q.w()))})
                     .with_colors({toColor(rgba)})
                     .with_fill_mode(rerun::components::FillMode::Solid),
                 rerun::CoordinateFrame(std::string(kBaseLink)));
}

void Viz::logPath(std::string_view name, const std::vector<Eigen::Vector3d>& points) const {
  std::vector<rerun::Vec3D> strip;
  strip.reserve(points.size());
  for (const Eigen::Vector3d& p : points) {
    strip.push_back(toVec(p));
  }
  impl_->rec.log(std::string(kBaseLink) + "/plan/" + std::string(name),
                 rerun::LineStrips3D({rerun::components::LineStrip3D(strip)})
                     .with_colors({rerun::Color(80, 160, 240)}),
                 rerun::CoordinateFrame(std::string(kBaseLink)));
}

void Viz::logText(std::string_view text, Level level) const {
  rerun::TextLogLevel rr_level = rerun::TextLogLevel::Info;
  switch (level) {
    case Level::kInfo:
      break;
    case Level::kWarning:
      rr_level = rerun::TextLogLevel::Warning;
      break;
    case Level::kError:
      rr_level = rerun::TextLogLevel::Error;
      break;
  }
  impl_->rec.log("log", rerun::TextLog(std::string(text)).with_level(rr_level));
}

void Viz::setTick(std::int64_t tick, double seconds) const {
  impl_->rec.set_time_sequence("tick", tick);
  impl_->rec.set_time_duration_secs("time", seconds);
}

const rerun::RecordingStream& Viz::stream() const { return impl_->rec; }

struct RobotLogger::Impl {
  struct JointFrames {
    std::string joint;
    std::string parent;
    std::string child;
  };

  // The stream, not the Viz: a moved Viz keeps the same stream.
  const rerun::RecordingStream& rec;
  const roboplan::Scene& scene;
  pinocchio::Data data;
  Eigen::VectorXd q;
  // Index into q of each single-dof joint, by name. Continuous joints (the wheels, two entries
  // each) are left at their defaults.
  std::map<std::string, int, std::less<>> q_index;
  std::vector<JointFrames> frames;
};

RobotLogger::RobotLogger(const Viz& viz, const roboplan::Scene& scene)
    : impl_(std::make_unique<Impl>(Impl{viz.stream(), scene, pinocchio::Data(scene.getModel()),
                                        scene.getCurrentJointPositions(), {}, {}})) {
  const pinocchio::Model& model = scene.getModel();
  for (std::size_t id = 1; id < model.joints.size(); ++id) {
    if (model.joints[id].nq() == 1) {
      impl_->q_index.emplace(model.names[id], model.joints[id].idx_q());
    }
  }
  // Rerun's URDF loader names the transform between two links after the joint, with the links as
  // its frames; pinocchio has a JOINT frame between each BODY frame and its parent.
  const auto& model_frames = model.frames;
  for (const pinocchio::Frame& child : model_frames) {
    if (child.type != pinocchio::BODY) {
      continue;
    }
    const pinocchio::Frame& joint = model_frames[child.parentFrame];
    if (joint.type != pinocchio::JOINT) {
      continue;
    }
    impl_->frames.push_back({joint.name, model_frames[joint.parentFrame].name, child.name});
  }
}

RobotLogger::~RobotLogger() = default;

void RobotLogger::log(const sensor_msgs::msg::JointState& joint_state) {
  const std::size_t n = std::min(joint_state.name.size(), joint_state.position.size());
  for (std::size_t i = 0; i < n; ++i) {
    if (const auto it = impl_->q_index.find(joint_state.name[i]); it != impl_->q_index.end()) {
      impl_->q[it->second] = joint_state.position[i];
    }
  }
  const rerun::RecordingStream& rec = impl_->rec;
  for (const Impl::JointFrames& f : impl_->frames) {
    const Eigen::Matrix4d tform =
        impl_->scene.forwardKinematics(impl_->data, impl_->q, f.child, f.parent);
    rec.log("world/robot/joints/" + f.joint,
            toTransform(tform).with_parent_frame(f.parent).with_child_frame(f.child));
  }
}

}  // namespace emma_behaviors
