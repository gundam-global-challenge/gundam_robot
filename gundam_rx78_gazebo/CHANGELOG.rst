^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gundam_rx78_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2020-08-28)
------------------
* skip test_gundam_spawn/test_gundam_walk for kinetic (`#14 <https://github.com/gundam-global-challenge/gundam_robot/issues/14>`_)
* Contributors: Kei Okada

0.0.3 (2020-01-28)
------------------
* add more walking tests (`#8 <https://github.com/gundam-global-challenge/gundam_robot/issues/8>`_)

  * update travis.yml
  * fix check_walk_pose.py to take target pos/ros as argument and also takes walking pattern fiels

* Contributors: Kei Okada

0.0.2 (2020-01-20)
------------------
* Add more examples by @Naoki-Hiraoka (`#6 <https://github.com/gundam-global-challenge/gundam_robot/issues/6>`_)

  * Merge https://github.com/gundam-global-challenge/gundam_robot/pull/4  with test code
  * add test to check walk-forward.csv
  * add gundam_rx78_walk.launch
  * install test directory
  * start gazebo simulation clock by default
  * Simplify URDF (simplify, scale and mergenode collada)
    - calculate mass property from convex_hull
    - fix check_position, now base_link is the center of foot

* add test code and meta package (`#5 <https://github.com/gundam-global-challenge/gundam_robot/issues/5>`_)

* Contributors: Kei Okada, Naoki Hiraoka

0.0.1 (2019-07-01)
------------------
* Add first gundam gazebon simulator (`#1 <https://github.com/gundam-global-challenge/gundam_robot/issues/1>`_)

  * comment out fast_foot because the origin of the house differs in kinetic and melodic
  * gundam_rx78_gazebo depends on gundam_rx78_control
  * update initial joint position, set paused=true as default
  * commit 2019/03/06 pinned & position controller version

* Contributors: Kei Okada
