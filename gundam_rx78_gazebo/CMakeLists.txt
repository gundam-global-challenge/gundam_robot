cmake_minimum_required(VERSION 2.8.3)
project(gundam_rx78_gazebo)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############
install(DIRECTORY launch worlds test
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   USE_SOURCE_PERMISSIONS
   )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint rostest)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  if(NOT $ENV{ROS_DISTRO} STREQUAL "kinetic")
    add_rostest(test/test_gundam_spawn.launch)
    add_rostest(test/test_gundam_walk.launch)
  endif()
  set(ROSLINT_PYTHON_OPTS --max-line-length=180)
  roslint_python()
  roslint_add_test()
endif()
