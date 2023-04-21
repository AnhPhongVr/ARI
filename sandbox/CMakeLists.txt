cmake_minimum_required(VERSION 3.0.2)
project(sandbox)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  control_msgs
  trajectory_msgs
  pocketsphinx
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Nothing

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# Nothing

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(run_dab src/move/run_dab.cpp)
add_executable(run_default_position src/move/run_default_position.cpp)
add_executable(run_forward src/move/base/run_forward.cpp)
add_executable(run_backward src/move/base/run_backward.cpp)
add_executable(welcome src/move/base/welcome.cpp)
add_executable(run_right_arm src/move/base/run_right_arm.cpp)
add_executable(run_rotation src/move/base/run_rotation.cpp)
add_executable(publish src/publish.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(run_dab ${catkin_LIBRARIES})
target_link_libraries(run_default_position ${catkin_LIBRARIES})
target_link_libraries(run_forward ${catkin_LIBRARIES})
target_link_libraries(run_backward ${catkin_LIBRARIES})
target_link_libraries(welcome ${catkin_LIBRARIES})
target_link_libraries(run_right_arm ${catkin_LIBRARIES})
target_link_libraries(run_rotation ${catkin_LIBRARIES})
target_link_libraries(publish ${catkin_LIBRARIES})


#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/move/run_wave.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/move/run_shaking_hands.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/joy/run_joy_ari.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/welcome/run_welcome.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/say.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/reply.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/repeat.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/repeat2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/face_detector_python.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/tryfr.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/website.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS run_dab run_default_position run_forward
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Testing ##
#############

# Nothing
