^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotiq_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* First release on the ROS build farm. Earlier releases: https://github.com/robotiq/robotiq-ros/releases
* Deprecated: ``motorCurrentFromRegister`` and ``kAmperesPerCurrentCount``; use ``Robotiq::units::motorCurrentFromRegister``.
* rSP spans ``gripper_profile``'s speed range, halved on the Hand-E family, from the gripper's slowest speed rather than from zero, so a speed below full scale commands a lower register than before. ``gripper_speed_multiplier`` is a fraction of that range.
* ``gripper_force_multiplier`` is a fraction of ``gripper_max_force`` rather than of 235 N, so below 1 it starts the 2F-140, Hand-E and Hand-E C10 at a lower force than before.
* ``gripper_max_speed`` defaults to the profile's top speed. It sets the speed at rSP 255; it does not cap the speed.
* ``gripper_profile`` accepts ``2f_140``, the 2F-140's own SDK profile.
* ``gripper_profile`` accepts ``hand_e_100``, the Hand-E 100's own SDK profile.
