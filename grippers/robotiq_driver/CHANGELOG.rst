^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotiq_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* First release on the ROS build farm. Earlier releases: https://github.com/robotiq/robotiq-ros/releases
* Deprecated: ``motorCurrentFromRegister`` and ``kAmperesPerCurrentCount``; use ``Robotiq::units::motorCurrentFromRegister``.
* The 2F joint position is the SDK's knuckle angle, and ``gripper_closed_position`` only gives its sign. A description whose closed position is not the profile's angle keeps the linear mapping, with a deprecation warning.
* ``gripper_min_speed``: rSP scales from the gripper's slowest speed rather than from zero, so a speed below full scale commands a lower register than before. ``gripper_speed_multiplier`` is a fraction of that range.
* ``gripper_profile`` accepts ``2f_140``, the 2F-140's own SDK profile.
* ``gripper_profile`` accepts ``hand_e_100``, the Hand-E 100's own SDK profile.
