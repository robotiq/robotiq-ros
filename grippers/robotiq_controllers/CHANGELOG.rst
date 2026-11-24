^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotiq_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.1 (2026-11-24)
------------------
* First release on the ROS build farm. Earlier releases: https://github.com/robotiq/robotiq-ros/releases

1.4.0 (2026-11-20)
------------------
* ``normalized_gripper_cmd``, a second action on the gripper controller, takes the ``gripper_cmd`` goal with a position from 0 = open to 1 = closed. On Jazzy and newer, its speed and force also run from 0 to 1.
* ``robotiq_controllers/NormalizedPositionBroadcaster`` publishes the gripper's position on ``joint_states`` as ``robotiq_normalized_joint``, 0 = open to 1 = closed.
* ``RobotiqActivationController`` takes a ``reactivate_gpio`` parameter, default ``reactivate_gripper``.
* ``normalized_gripper_cmd`` scales a normalized speed over ``gripper_profile``'s speed range, as the driver does.
* Changes for users of PickNik's 0.0.1 release: ``robotiq_controllers`` now depends on ``robotiq_driver`` and ``robotiq_msgs``.
