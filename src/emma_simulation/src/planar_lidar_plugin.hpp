#pragma once

#include <memory>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace emma_simulation {

/**
 * @brief 2D lidar for mujoco_ros2_control: casts a fan of rays from an MJCF site and publishes
 *        a sensor_msgs/LaserScan.
 *
 * MuJoCo rangefinder sensors are evaluated on every physics step, which in a furnished room
 * slows the sim below real time. This plugin casts the rays with mj_multiRay only at the publish
 * rate. Beams lie in the site's x-y plane, counter-clockwise from -pi, like a REP 117 scan.
 *
 * Parameters (under mujoco_plugins.<plugin name>.):
 *   site          - MJCF site the rays start from; its body is excluded. Default "laser_frame".
 *   frame_id      - LaserScan frame. Default: the site name.
 *   topic         - Default "/scan".
 *   publish_rate  - Hz. Default 10.
 *   beams         - Rays per revolution. Default 360.
 *   range_min     - m. Default 0.12.
 *   range_max     - m; misses and returns past it are +inf. Default 8.
 */
class PlanarLidarPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
 public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

 private:
  using LaserScan = sensor_msgs::msg::LaserScan;

  rclcpp::Node::SharedPtr node_;
  int site_id_{-1};
  double range_max_{0.0};
  // Beam directions in the site frame, 3 per beam.
  std::vector<mjtNum> local_dirs_;
  std::vector<mjtNum> world_dirs_;
  std::vector<mjtNum> dists_;
  std::vector<int> geom_ids_;
  rclcpp::Duration period_{0, 0};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  LaserScan scan_;
  rclcpp::Publisher<LaserScan>::SharedPtr publisher_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<LaserScan>> publisher_;
};

}  // namespace emma_simulation
