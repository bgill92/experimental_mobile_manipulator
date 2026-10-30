#include "planar_lidar_plugin.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <string>
#include <tuple>

#include <pluginlib/class_list_macros.hpp>

namespace emma_simulation {

namespace {

template <typename T>
[[nodiscard]] T param(rclcpp::Node& node, const std::string& name, const T& fallback) {
  const std::string full = "mujoco_plugins." + node.get_sub_namespace() + "." + name;
  if (!node.has_parameter(full)) {
    node.declare_parameter(full, fallback);
  }
  return node.get_parameter(full).get_value<T>();
}

}  // namespace

bool PlanarLidarPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/) {
  node_ = node;
  const auto site = param<std::string>(*node_, "site", "laser_frame");
  const auto frame_id = param<std::string>(*node_, "frame_id", site);
  const auto topic = param<std::string>(*node_, "topic", "/scan");
  const double rate = param<double>(*node_, "publish_rate", 10.0);
  const int64_t beams = param<int64_t>(*node_, "beams", 360);
  const double range_min = param<double>(*node_, "range_min", 0.12);
  range_max_ = param<double>(*node_, "range_max", 8.0);

  site_id_ = mj_name2id(model, mjOBJ_SITE, site.c_str());
  if (site_id_ < 0) {
    RCLCPP_ERROR(node_->get_logger(), "PlanarLidarPlugin: no site '%s' in the MuJoCo model", site.c_str());
    return false;
  }
  if (rate <= 0.0 || beams <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "PlanarLidarPlugin: publish_rate and beams must be > 0");
    return false;
  }

  const double increment = 2.0 * std::numbers::pi / static_cast<double>(beams);
  local_dirs_.resize(3 * beams);
  for (int64_t i = 0; i < beams; ++i) {
    const double angle = -std::numbers::pi + static_cast<double>(i) * increment;
    local_dirs_[3 * i] = std::cos(angle);
    local_dirs_[3 * i + 1] = std::sin(angle);
    local_dirs_[3 * i + 2] = 0.0;
  }
  world_dirs_.resize(3 * beams);
  dists_.resize(beams);
  geom_ids_.resize(beams);

  scan_.header.frame_id = frame_id;
  scan_.angle_min = static_cast<float>(-std::numbers::pi);
  scan_.angle_max = static_cast<float>(-std::numbers::pi + static_cast<double>(beams - 1) * increment);
  scan_.angle_increment = static_cast<float>(increment);
  scan_.scan_time = static_cast<float>(1.0 / rate);
  scan_.range_min = static_cast<float>(range_min);
  scan_.range_max = static_cast<float>(range_max_);
  scan_.ranges.resize(beams);

  period_ = rclcpp::Duration::from_seconds(1.0 / rate);
  last_publish_ = node_->get_clock()->now();
  publisher_raw_ = node_->create_publisher<LaserScan>(topic, rclcpp::SensorDataQoS());
  publisher_ = std::make_unique<realtime_tools::RealtimePublisher<LaserScan>>(publisher_raw_);
  RCLCPP_INFO(node_->get_logger(), "PlanarLidarPlugin: %ld beams from site '%s' on %s at %.1f Hz",
              static_cast<long>(beams), site.c_str(), topic.c_str(), rate);
  return true;
}

void PlanarLidarPlugin::update(const mjModel* model, mjData* data) {
  const rclcpp::Time now = node_->get_clock()->now();
  // Sim time jumps back on a world reset; restart the period instead of going silent.
  if (now >= last_publish_ && now - last_publish_ < period_) {
    return;
  }
  last_publish_ = now;

  const mjtNum* rotation = data->site_xmat + 9 * site_id_;
  const int beams = static_cast<int>(dists_.size());
  for (int i = 0; i < beams; ++i) {
    mju_mulMatVec3(&world_dirs_[3 * i], rotation, &local_dirs_[3 * i]);
  }
  // ponytail: casts against every geom, visual meshes included (~6 ms per 360-beam scan in
  // FloorPlan201). A geomgroup mask would cut that ~5x, but the bare scenes keep colliding
  // geoms in group 0 alongside the room's visuals.
  mj_multiRay(model, data, data->site_xpos + 3 * site_id_, world_dirs_.data(), nullptr, 1,
              model->site_bodyid[site_id_], geom_ids_.data(), dists_.data(), nullptr, beams, range_max_);

  constexpr float kInf = std::numeric_limits<float>::infinity();
  for (int i = 0; i < beams; ++i) {
    // mj_multiRay returns -1 for a miss; REP 117 says +inf for nothing within range.
    scan_.ranges[i] = (dists_[i] < 0.0 || dists_[i] > range_max_) ? kInf : static_cast<float>(dists_[i]);
  }
  scan_.header.stamp = now;
  std::ignore = publisher_->try_publish(scan_);
}

void PlanarLidarPlugin::cleanup() {
  publisher_.reset();
  publisher_raw_.reset();
}

}  // namespace emma_simulation

PLUGINLIB_EXPORT_CLASS(emma_simulation::PlanarLidarPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
