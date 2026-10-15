#include "emma_manipulation/config.hpp"

#include <cstddef>
#include <format>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "emma_manipulation/constants.hpp"

namespace emma_manipulation {
namespace {

[[nodiscard]] tl::expected<Eigen::VectorXd, std::string> declareVector(rclcpp::Node& node,
                                                                       const std::string& name,
                                                                       const std::size_t size) {
  std::vector<double> value;
  try {
    value = node.declare_parameter<std::vector<double>>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException&) {
    return tl::unexpected(std::format("parameter {} is not set", name));
  } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
    return tl::unexpected(
        std::format("parameter {} must be a list of doubles (write 0.0, not 0)", name));
  }
  if (value.size() != size) {
    return tl::unexpected(
        std::format("parameter {} needs {} numbers, got {}", name, size, value.size()));
  }
  return Eigen::Map<const Eigen::VectorXd>(value.data(), static_cast<Eigen::Index>(size));
}

}  // namespace

tl::expected<ArmConfig, std::string> loadArmConfig(rclcpp::Node& node) {
  const tl::expected<Eigen::VectorXd, std::string> home_q =
      declareVector(node, "home_q", kArmJoints.size());
  if (!home_q.has_value()) {
    return tl::unexpected(home_q.error());
  }
  const tl::expected<Eigen::VectorXd, std::string> table_center =
      declareVector(node, "table_center", 3);
  if (!table_center.has_value()) {
    return tl::unexpected(table_center.error());
  }
  const tl::expected<Eigen::VectorXd, std::string> table_size =
      declareVector(node, "table_size", 3);
  if (!table_size.has_value()) {
    return tl::unexpected(table_size.error());
  }
  return ArmConfig{*home_q, *table_center, *table_size};
}

}  // namespace emma_manipulation
