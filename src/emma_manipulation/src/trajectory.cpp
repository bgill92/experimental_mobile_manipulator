#include "emma_manipulation/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include "emma_manipulation/constants.hpp"

namespace emma_manipulation {

trajectory_msgs::msg::JointTrajectory toJointTrajectory(const std::vector<Eigen::VectorXd>& path,
                                                        const double v_max,
                                                        const double min_segment_time) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = kArmJoints;
  double t = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::VectorXd& point = path[i];
    const double max_step = (point - path[i - 1]).cwiseAbs().maxCoeff();
    t += std::max(max_step / v_max, min_segment_time);
    const std::int64_t total_ns = std::llround(t * 1e9);

    trajectory_msgs::msg::JointTrajectoryPoint out;
    out.positions.assign(point.data(), point.data() + point.size());
    out.time_from_start.sec = static_cast<std::int32_t>(total_ns / 1'000'000'000);
    out.time_from_start.nanosec = static_cast<std::uint32_t>(total_ns % 1'000'000'000);
    traj.points.push_back(std::move(out));
  }
  return traj;
}

}  // namespace emma_manipulation
