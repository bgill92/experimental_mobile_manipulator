cmake_minimum_required(VERSION 3.25)
project(emma_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(emma_manipulation REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# beet is header-only and has no package.xml; it is consumed straight from the submodule.
set(BEET_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BEET_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../external_packages/beet
  ${CMAKE_CURRENT_BINARY_DIR}/beet
  EXCLUDE_FROM_ALL
)

# Leaves, ROS I/O and Rerun logging, shared by the executable and the tests. Static so nothing
# needs installing; only viz.cpp includes rerun.hpp.
add_library(${PROJECT_NAME}_core STATIC
  src/leaves.cpp
  src/ros_io.cpp
  src/viz.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# ament_target_dependencies is deprecated in Lyrical, so link the exported targets directly.
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    emma_manipulation::emma_manipulation
    beet::beet
    Eigen3::Eigen
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ament_index_cpp::ament_index_cpp
    ${control_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
  PRIVATE
    rerun_sdk
)

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${PROJECT_NAME}_core rerun_sdk)
install(TARGETS pick_and_place DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch scripts DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

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

  ament_add_gtest(test_tree test/test_tree.cpp TIMEOUT 300)
  add_dependencies(test_tree ${PROJECT_NAME}_test_urdf)
  target_link_libraries(test_tree ${PROJECT_NAME}_core)
  target_compile_definitions(test_tree PRIVATE
    EMMA_TEST_URDF="${XACRO_OUTPUT_FILE}"
  )
endif()

ament_package()
