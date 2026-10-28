cmake_minimum_required(VERSION 3.10)
project(robotiq_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    rviz
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_controller_config test/test_controller_config.py)
  # Expands the xacro, so it needs the package's own share dir on the ament index.
  ament_add_pytest_test(test_ros2_control_urdf test/test_ros2_control_urdf.py TIMEOUT 120)
  ament_add_pytest_test(test_hand_e_c10_urdf test/test_hand_e_c10_urdf.py TIMEOUT 120)
endif()

ament_package()
