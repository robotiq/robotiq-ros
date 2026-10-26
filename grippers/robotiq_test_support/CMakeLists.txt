cmake_minimum_required(VERSION 3.8)
project(robotiq_test_support)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

# An INTERFACE target that carries the source: the main() compiles into every
# test executable that links it, against that executable's own gtest, so no
# gtest flavour or link order is baked in here.
add_library(rclcpp_gtest_main INTERFACE)
add_library(${PROJECT_NAME}::rclcpp_gtest_main ALIAS rclcpp_gtest_main)
target_sources(rclcpp_gtest_main INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rclcpp_gtest_main.cpp>
  $<INSTALL_INTERFACE:share/${PROJECT_NAME}/src/rclcpp_gtest_main.cpp>
)
target_link_libraries(rclcpp_gtest_main INTERFACE
  rclcpp::rclcpp
  rcpputils::rcpputils
)

install(FILES src/rclcpp_gtest_main.cpp DESTINATION share/${PROJECT_NAME}/src)
install(TARGETS rclcpp_gtest_main EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rcpputils)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Same linter set as the other gripper packages: no copyright or cpplint
  # pass, clang-format (ci-format) is the formatting authority, and xmllint
  # needs the network.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
