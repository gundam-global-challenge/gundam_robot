cmake_minimum_required(VERSION 2.8.3)
project(gundam_rx78_description)

find_package(catkin REQUIRED COMPONENTS roslint)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############
install(DIRECTORY config launch meshes scripts urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  set(ROSLINT_PYTHON_OPTS --max-line-length=220 --ignore=W504,W605,E131,E741,E711,E722) # skip continuation line unaligned for hanging indent
  roslint_python()
  roslint_add_test()
endif()

