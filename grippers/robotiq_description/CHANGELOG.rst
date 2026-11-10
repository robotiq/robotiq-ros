^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotiq_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* First release on the ROS build farm. Earlier releases: https://github.com/robotiq/robotiq-ros/releases
* ``robotiq_gripper_ros2_control`` accepts PickNik ``main``'s ``sim_isaac``, ``isaac_joint_commands`` and ``isaac_joint_states`` arguments on the 2F-85 and the 2F-140, deprecated like in the ``robotiq_gripper`` macros.
* Changes for users of PickNik's 0.0.1 release:

  * Mock hardware (``use_fake_hardware``) declares only the driven joint, ``robotiq_85_left_knuckle_joint``. ``robot_state_publisher`` derives the other finger joints from the URDF's mimic tags.
  * ``sim_isaac`` selects ``topic_based_ros2_control/TopicBasedSystem`` instead of ``isaac_ros2_control/IsaacSystem``, on the same ``/isaac_joint_commands`` and ``/isaac_joint_states`` topics.
  * ``sim_ignition`` selects ``gz_ros2_control/GazeboSimSystem`` instead of ``ign_ros2_control/IgnitionSystem``.
  * Deprecated, and removed in the next major release: the ``sim_ignition``, ``sim_isaac`` and ``fake_sensor_commands`` arguments, ``urdf/robotiq_gripper.ros2_control.xacro``, the ``connected_to`` argument of ``ur_to_robotiq``, and the 2F-85 meshes directly under ``meshes/visual`` and ``meshes/collision``.
* Changes for users of the 2F-140 description of 1.1 to 1.3:

  * The description is rebuilt from Robotiq's Isaac Sim asset of the current 2F-140: new meshes, and link frames at each link's pivot, parallel to ``robotiq_140_base_link``. Link and joint names are unchanged. It hands the driver ``gripper_profile`` ``2f_140``.
  * ``gripper_closed_position`` is 0.77 instead of 0.695, fitted to a real 2F-140, and ``finger_joint`` reaches 0.785 instead of 0.7. A goal of 0.695 no longer closes the gripper fully.
  * The pad frames, ``left_inner_finger_pad`` and ``right_inner_finger_pad``, keep their orientation and move to the centre of the pad's contact face.
  * ``urdf/robotiq_2f_140.xacro`` keeps only the ``finger_links``, ``finger_joint`` and ``right_outer_knuckle_joint`` macros, which a ``robotiq_2f_140_macro.urdf.xacro`` copied from 1.3 calls; the per-link macros are gone. The mimic joints turn the other way, and their limits follow ``finger_joint``'s.
  * Deprecated, and removed in the next major release: the ``stroke`` argument of ``finger_links``, and the 1.3 meshes under ``meshes/visual/2f_140`` and ``meshes/collision/2f_140`` that the description no longer uses. ``meshes/collision/2f_140/robotiq_2f_140_base_link.stl`` now holds the new base's hull.
* ``view_gripper.launch.py`` takes ``gripper_model``, and both launches pick the RViz view by it.
