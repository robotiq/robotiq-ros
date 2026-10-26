# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)
# Already present when this package is configured as part of a larger project
# (the dev/ IDE aggregate); found through its install prefix otherwise.
if(NOT TARGET robotiq_test_support::rclcpp_gtest_main)
  find_package(robotiq_test_support REQUIRED)
endif()

ament_lint_auto_find_test_dependencies()

# GMock throws an error if we don't switch off this option in tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-sign-conversion)
endif()

# One gmock binary for the whole package, so the suite runs — and debugs — as a
# single target. Pick a subset with --gtest_filter.
ament_add_gmock(${PROJECT_NAME}_tests
    test_gripper_scaling.cpp                     # register <-> joint arithmetic
    test_hardware_parameters.cpp                 # HardwareInfo -> ConnectionConfig
    test_rclcpp_logger.cpp                       # SDK log sink -> /rosout
    test_robotiq_gripper_hardware_interface.cpp  # plugin load, interfaces, use_dummy
)
target_include_directories(${PROJECT_NAME}_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
# Where this build installs the plugin, for the tests that load it by name.
target_compile_definitions(${PROJECT_NAME}_tests
    PRIVATE ROBOTIQ_DRIVER_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}"
)
target_link_libraries(
    ${PROJECT_NAME}_tests
    robotiq_driver
    hardware_interface::hardware_interface
    robotiq_test_support::rclcpp_gtest_main
)
