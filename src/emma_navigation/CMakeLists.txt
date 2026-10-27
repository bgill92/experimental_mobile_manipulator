cmake_minimum_required(VERSION 3.20)
project(emma_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps scripts
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
