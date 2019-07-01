^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gundam_rx78_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
