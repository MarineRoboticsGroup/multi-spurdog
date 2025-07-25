cmake_minimum_required(VERSION 3.0.2)
project(spurdog_acomms)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# add_compile_options(-std=c++17)
# set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM")
set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM")
#set(EIGEN3_INCLUDE_DIR /usr/local/include/eigen3)
# set(EIGEN3_INCLUDE_DIRS "/usr/include/eigen3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ros_acomms
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  message_generation
)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bar30SoundSpeed.msg
  PoseWithAssoc.msg
  RangeWithAssoc.msg
  InitPrior.msg
  PartialGraph.msg
  CycleGraph.msg
  CommsCycleStatus.msg
  AcommsCycleStatus.msg
  PoseFactorStamped.msg
  RangeFactorStamped.msg
  TestData.msg
  AdvancedGraphUpdate.msg
  BasicGraphUpdate.msg
  BetweenFactor.msg
  RangeFactor.msg
  ReceivedSignalStats.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PreintegrateIMU.srv
  PreintegrateImu.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs#   std_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES spurdog_acomms
  CATKIN_DEPENDS ros_acomms roscpp rospy sensor_msgs std_msgs message_runtime geometry_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/spurdog_acomms.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/spurdog_acomms_node.cpp)
# add_executable(imu_preintegrator src/imu_preintegrator.cpp)
# add_executable(manifold_preintegrator src/manifold_preintegrator.cpp)
# add_executable(gyro_preintegrator src/gyro_preintegrator.cpp)
add_executable(odometry src/odometry.cpp)
# add_executable(dvl_integrator src/dvl_integrator.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
# Some of these can be removed in the future:
# target_link_libraries(imu_preintegrator
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   gtsam
# )
# target_link_libraries(manifold_preintegrator
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   gtsam
# )
# target_link_libraries(gyro_preintegrator
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   gtsam
# )
target_link_libraries(odometry
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)
# target_link_libraries(dvl_integrator
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   gtsam
# )
add_dependencies(odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
install(DIRECTORY src/spurdog_acomms_utils
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/spurdog_acomms_utils
)
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/ping_test.py
  src/extract_range_data.py
  src/comms_cycle_mgr.py
  src/factor_graph_mgr.py
  src/comms_ltwt.py
  src/comms_lbl.py
  src/comms_multiagent.py
  src/comms_lbl_multiagent.py
  src/imu_sensor_handler.py
  src/imu_cal.py
  src/test_preintegration.py
  src/extract_preintegration.py
  src/test_timing.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_spurdog_acomms.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
