cmake_minimum_required(VERSION 3.20)
project(emma_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roboplan_core REQUIRED)
find_package(roboplan_rrt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/grasp.cpp
  src/planner.cpp
  src/trajectory.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# ament_target_dependencies is deprecated in Lyrical, so link the exported
# message targets directly.
target_link_libraries(${PROJECT_NAME} PUBLIC
  roboplan::roboplan
  roboplan_rrt::roboplan_rrt
  roboplan_simple_ik::roboplan_simple_ik
  tinyxml2::tinyxml2
  Eigen3::Eigen
  ament_index_cpp::ament_index_cpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)

add_executable(move_arm src/move_arm.cpp)
target_link_libraries(move_arm
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${control_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# lib/<package> so `ros2 run emma_manipulation move_arm` finds it.
install(TARGETS move_arm DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  builtin_interfaces
  Eigen3
  roboplan_core
  roboplan_rrt
  roboplan_simple_ik
  sensor_msgs
  tinyxml2
  trajectory_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(emma_description REQUIRED)

  # An explicit output path: the default writes next to the input, which is
  # inside install/emma_description.
  xacro_add_xacro_file(
    ${emma_description_DIR}/../urdf/emma.urdf.xacro
    ${CMAKE_CURRENT_BINARY_DIR}/test/emma.urdf
  )
  add_custom_target(${PROJECT_NAME}_test_urdf DEPENDS ${XACRO_OUTPUT_FILE})

  ament_add_gtest(test_grasp test/test_grasp.cpp)
  target_link_libraries(test_grasp ${PROJECT_NAME})

  ament_add_gtest(test_planner test/test_planner.cpp TIMEOUT 300)
  add_dependencies(test_planner ${PROJECT_NAME}_test_urdf)
  target_link_libraries(test_planner ${PROJECT_NAME})
  target_compile_definitions(test_planner PRIVATE
    EMMA_TEST_URDF="${XACRO_OUTPUT_FILE}"
  )
endif()

ament_package()
