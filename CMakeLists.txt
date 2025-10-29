cmake_minimum_required(VERSION 3.0.2)
project(waypoints_path_planner)
# Set a default value if not defined
if(NOT DEFINED veh_type)
  set(veh_type "ADT3")
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  car_like_planner_ros
  ecl_geometry
  geometry_msgs
  interactive_markers
  pluginlib
 # podzero_message_definitions
  rosbag
  roscpp
  rospy
  rqt_gui
  rqt_gui_cpp
  rviz
  std_msgs
  std_srvs
  tf
  tf_conversions
  visualization_msgs
  waypoints_path_planning_tool
  message_generation
 
  )
  # Add vehicle-specific defines and dependencies
set(VEHICLE_CATKIN_DEP "")
if(${veh_type} STREQUAL "ACA1")
  add_definitions(-DVEH_TYPE_ACA1)
  find_package(aca1_message_definitions REQUIRED)
  set(VEHICLE_CATKIN_DEP aca1_message_definitions)
  include_directories(${aca1_message_definitions_INCLUDE_DIRS})
elseif(${veh_type} STREQUAL "POD")
  add_definitions(-DVEH_TYPE_POD)
  find_package(podzero_message_definitions REQUIRED)
  set(VEHICLE_CATKIN_DEP podzero_message_definitions)
  include_directories(${podzero_message_definitions_INCLUDE_DIRS})
elseif(${veh_type} STREQUAL "ADT3")
  add_definitions(-DVEH_TYPE_ADT3)
  find_package(adt3_message_definitions REQUIRED)
  set(VEHICLE_CATKIN_DEP adt3_message_definitions)
  include_directories(${adt3_message_definitions_INCLUDE_DIRS})
elseif(${veh_type} STREQUAL "STL2")
  add_definitions(-DVEH_TYPE_STL2)
  find_package(stl2_message_definitions REQUIRED)
  set(VEHICLE_CATKIN_DEP stl2_message_definitions)
  include_directories(${stl2_message_definitions_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Unsupported vehicle type: ${veh_type}")
endif()

message(STATUS "Using vehicle type: ${veh_type}")

find_package(waypoints_path_planning_tool REQUIRED) 
message(STATUS "Catkin INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message(STATUS "Catkin LIBS: ${catkin_LIBRARIES}")

if (CMAKE_VERSION VERSION_LESS 3.4)
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  endif(CCACHE_FOUND)
endif()

find_package(PkgConfig) 
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)
if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()
 
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread timer chrono filesystem graph)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(HEADER_FILES include/waypoints_path_planner/waypoints_path_planner.h
include/waypoints_path_planner/waypoints_path_planner_panel_management.h
include/waypoints_path_planner/status_monitoring_panel.h
include/waypoints_path_planner/visualization_manager.h)

set(EXTERNAL_INCLUDE_DIRS 
		${Eigen_INCLUDE_DIRS} 
		${EIGEN_INCLUDE_DIRS} 
		${CERES_INCLUDE_DIRS} 
		${Boost_INCLUDE_DIRS}
		${car_like_planner_ros_INCLUDE_DIRS}
		${waypoints_path_planning_tool_INCLUDE_DIRS}
		)
		
set(EXTERNAL_LIBS   
  		${CERES_LIBRARIES}
  		${Boost_LIBRARIES} 
		${car_like_planner_ros_LIBRARIES}
		${waypoints_path_planning_tool_LIBRARIES}
  	)

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
	DIRECTORY msg
 	FILES 
  	SectionTrajectory.msg
  	SectionTrajectoryArray.msg
	waypointinfo.msg
	routeinfo.msg
	routeinfoarray.msg
	waypointinfoarray.msg
	nogozoneinfo.msg
	nogozoneinfoarray.msg
    podbaselinepointsarray.msg
    currentpodposewithtrajectoryln.msg
    waypointsfilepath.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   startNavigation.srv
   alignWaypoints.srv
   alternativeRoute.srv
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
  std_msgs
  geometry_msgs 
  waypoints_path_planner
 # interactive_waypoints
  
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
  LIBRARIES waypoints_path_planning_tool car_like_planner_ros ${EXTERNAL_LIBS}
  CATKIN_DEPENDS 
	  roscpp
	  rospy
	  rosbag
	  rviz
	  std_msgs
	  geometry_msgs
	  actionlib_msgs
	  actionlib
	  tf
	  std_srvs
	  interactive_markers
	  visualization_msgs
	  tf_conversions
	  pluginlib
	  rqt_gui
	  rqt_gui_cpp
	  message_generation
	  message_runtime
	  ecl_geometry
	#  podzero_message_definitions
      car_like_planner_ros
      waypoints_path_planning_tool
      ${VEHICLE_CATKIN_DEP}

	
#  DEPENDS system_lib
)

set(ROS_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS rviz)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 
 include_directories(
	include
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${car_like_planner_ros_INCLUDE_DIRS}
  
)

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
else()
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS})


if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/waypoints_path_planner.cpp src/main.cpp src/status_monitoring_panel.cpp src/visualization_manager.cpp ${MOC_FILES}
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${QT_LIBRARIES} ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} ${EXTERNAL_LIBS} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Widgets ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} ${EXTERNAL_LIBS} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif()

#set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME waypoint_path_planner PREFIX "")


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_node src/waypoints_path_planner.cpp src/main.cpp ${MOC_FILES})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
##set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME waypoints_path_planner_node PREFIX "")


## Add cmake target dependencies of the executable
## same as for the library above
 #add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#if(rviz_QT_VERSION VERSION_LESS "5")
#  target_link_libraries(${PROJECT_NAME}_node PUBLIC ${QT_LIBRARIES} ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} ${EXTERNAL_LIBS} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
#else()
#  target_link_libraries(${PROJECT_NAME}_node PUBLIC Qt5::Widgets ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} ${EXTERNAL_LIBS} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
#endif()


add_executable(navigator_waypoints src/navigator_interactive_waypoint.cpp)
add_dependencies(navigator_waypoints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(navigator_waypoints ${QT_LIBRARIES} ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} ${EXTERNAL_LIBS} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
else()
  target_link_libraries(navigator_waypoints Qt5::Widgets ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES} ${EXTERNAL_LIBS} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(navigator_waypoints PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()




## Mark executables and/or libraries for installation
# install(TARGETS navigator_waypoints ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_waypoints_path_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
