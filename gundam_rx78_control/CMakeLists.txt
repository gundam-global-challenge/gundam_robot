cmake_minimum_required(VERSION 2.8.3)
project(gundam_rx78_control)

find_package(catkin REQUIRED COMPONENTS controller_interface joint_trajectory_controller pluginlib)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS controller_interface joint_trajectory_controller pluginlib
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/joint_trajectory_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY config launch sample
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   USE_SOURCE_PERMISSIONS
   )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


install(FILES joint_trajectory_controller.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  set(ROSLINT_PYTHON_OPTS --max-line-length=180 --ignore=E221,E222,E241) # skip multiple spaces before/after operator
  roslint_python()
  roslint_add_test()
endif()
