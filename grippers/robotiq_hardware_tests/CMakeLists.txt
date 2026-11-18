# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.22)
project(robotiq_hardware_tests LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
# Already present when this package is configured as part of a larger project
# (the dev/ IDE aggregate); found through its install prefix otherwise.
if(NOT TARGET robotiq_driver::robotiq_driver)
  find_package(robotiq_driver REQUIRED)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(full_test
    src/gripper_interface_test.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
# The gripper SDK is compiled into librobotiq_driver.so and its headers are
# installed alongside that package's own, so linking the driver is enough to
# reach Robotiq::Gripper — there is no second package to find.
target_link_libraries(full_test robotiq_driver::robotiq_driver)

# Without this the executable only ever exists under build/, so
# `ros2 run robotiq_hardware_tests full_test` cannot find it.
install(
  TARGETS full_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
