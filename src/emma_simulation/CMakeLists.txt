cmake_minimum_required(VERSION 3.20)
project(emma_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

# mujoco_ros2_control plugins loaded by the sim's controller manager (see config/lidar.yaml).
add_library(${PROJECT_NAME}_plugins SHARED src/planar_lidar_plugin.cpp)
target_compile_features(${PROJECT_NAME}_plugins PRIVATE cxx_std_20)
target_link_libraries(${PROJECT_NAME}_plugins PRIVATE
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins
  pluginlib::pluginlib
  rclcpp::rclcpp
  realtime_tools::realtime_tools
  ${sensor_msgs_TARGETS}
)
install(TARGETS ${PROJECT_NAME}_plugins LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)

install(DIRECTORY config launch mujoco rviz scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
