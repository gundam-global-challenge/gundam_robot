^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gundam_rx78_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2020-08-28)
------------------
* put 2 blank lines after class definition (`#14 <https://github.com/gundam-global-challenge/gundam_robot/issues/14>`_)
* Contributors: Kei Okada

0.0.3 (2020-01-28)
------------------
* fix walk demo (`#8 <https://github.com/gundam-global-challenge/gundam_robot/issues/8>`_)
* Contributors: Naoki Hiraoka

0.0.2 (2020-01-20)
------------------
* Add more examples by @Naoki-Hiraoka (`#6 <https://github.com/gundam-global-challenge/gundam_robot/issues/6>`_)

  * Merge https://github.com/gundam-global-challenge/gundam_robot/pull/4  with test code
  * fix some parameters
  * add python sample (walk motion)
  * Simplify URDF (simplify, scale and mergenode collada)
    - calculate mass property from convex_hull
    - fix check_position, now base_link is the center of foot

* add test code and meta package (`#5 <https://github.com/gundam-global-challenge/gundam_robot/issues/5>`_)

  * roslint only used in CATKIN_ENABLE_TESTING
  * add joint_trajectory_controller to depends and find_package

* Contributors: Kei Okada, Naoki Hiraoka

0.0.1 (2019-07-01)
------------------
* Add controllers for gundom robot (`#1 <https://github.com/gundam-global-challenge/gundam_robot/issues/1>`_)

  * call controller_manager/spawner with --shutdown-timeout 0.1, this is deprecated on melodic
  * fix Typo Loaiding -> Loading by @Naoki-Hiraoka
  * respect @Naoki-Hiraoka 's dynamics parameters  https://github.com/Naoki-Hiraoka/gundam_robot/tree/gundam-walk
  * pip8ify python files
  * add roslint check
  * update GGC_TestModel_rx78_20170112.urdf and gundam_rx78_control.yaml with latest ggc_dae_to_urdf.py
  * add joint_trajectory_controller from https://github.com/ros-controls/ros_controllers/pull/411
  * update sample program from joint_position_client_example.py to joint_trajectory_client_example.py
  * update GGC_TestModel_rx78_20170112.urdf and gundam_rx78_control.yaml with latest ggc_dae_to_urdf.py
  * start fullbod_controller istead of each joint position controller
  * update GGC_TestModel_rx78_20170112.urdf and gundam_rx78_control.yaml with latest ggc_dae_to_urdf.py
  * joint_position_client_example.py: update reset pose
  * add mesh and urdf generated from GGC_TestModel_rx78_20170112
  * commit 2019/03/06 pinned & position controller version

* Contributors: Kei Okada, Naoki Hiraoka
