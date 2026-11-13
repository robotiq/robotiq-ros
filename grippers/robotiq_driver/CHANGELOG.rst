^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotiq_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* First release on the ROS build farm. Earlier releases: https://github.com/robotiq/robotiq-ros/releases
* Deprecated: ``motorCurrentFromRegister`` and ``kAmperesPerCurrentCount``; use ``Robotiq::units::motorCurrentFromRegister``.
* ``gripper_min_speed``: rSP scales from the gripper's slowest speed rather than from zero, so a speed below full scale commands a lower register than before. ``gripper_speed_multiplier`` is a fraction of that range.
* ``gripper_profile`` accepts ``2f_140``, the 2F-140's own SDK profile.
* ``gripper_profile`` accepts ``hand_e_100``, the Hand-E 100's own SDK profile.
