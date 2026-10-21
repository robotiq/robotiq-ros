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
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)
# The RViz views differ only in their camera: distance, focal point x y z,
# pitch and yaw. gripper_models.py picks one per gripper_model.
function(install_rviz_view name distance x y z pitch yaw)
  set(RVIZ_DISTANCE ${distance})
  set(RVIZ_FOCAL_X ${x})
  set(RVIZ_FOCAL_Y ${y})
  set(RVIZ_FOCAL_Z ${z})
  set(RVIZ_PITCH ${pitch})
  set(RVIZ_YAW ${yaw})
  configure_file(rviz/view_urdf.rviz.in rviz/${name} @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rviz/${name} DESTINATION share/${PROJECT_NAME}/rviz)
endfunction()
install_rviz_view(view_urdf.rviz 0.270967 -0.024239 -0.029138 0.050779 0.595398 0.840398)
install_rviz_view(view_urdf_2f_140.rviz 0.380689 -0.028251 -0.054114 0.081193 0.485398 0.920398)

# PickNik's 0.0.1 mesh paths, which hand-written descriptions may still use;
# removed in the next major release.
install(DIRECTORY meshes/visual/2f_85/ DESTINATION share/${PROJECT_NAME}/meshes/visual)
install(DIRECTORY meshes/collision/2f_85/ DESTINATION share/${PROJECT_NAME}/meshes/collision)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_controller_config test/test_controller_config.py)
  # Expands the xacro, so it needs the package's own share dir on the ament index.
  ament_add_pytest_test(test_ros2_control_urdf test/test_ros2_control_urdf.py TIMEOUT 120)
  ament_add_pytest_test(test_hand_e_c10_urdf test/test_hand_e_c10_urdf.py TIMEOUT 120)
  ament_add_pytest_test(test_hand_e_stroke_urdf test/test_hand_e_stroke_urdf.py TIMEOUT 120)
endif()

ament_package()
