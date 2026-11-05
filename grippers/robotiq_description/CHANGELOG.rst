^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotiq_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* First release on the ROS build farm. Earlier releases: https://github.com/robotiq/robotiq-ros/releases
* Changes for users of PickNik's 0.0.1 release:

  * Mock hardware (``use_fake_hardware``) declares only the driven joint, ``robotiq_85_left_knuckle_joint``. ``robot_state_publisher`` derives the other finger joints from the URDF's mimic tags.
  * ``sim_isaac`` selects ``topic_based_ros2_control/TopicBasedSystem`` instead of ``isaac_ros2_control/IsaacSystem``, on the same ``/isaac_joint_commands`` and ``/isaac_joint_states`` topics.
  * ``sim_ignition`` selects ``gz_ros2_control/GazeboSimSystem`` instead of ``ign_ros2_control/IgnitionSystem``.
  * Deprecated, and removed in the next major release: the ``sim_ignition``, ``sim_isaac`` and ``fake_sensor_commands`` arguments, ``urdf/robotiq_gripper.ros2_control.xacro``, the ``connected_to`` argument of ``ur_to_robotiq``, and the 2F-85 meshes directly under ``meshes/visual`` and ``meshes/collision``.
