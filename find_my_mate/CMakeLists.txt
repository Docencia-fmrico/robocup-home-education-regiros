cmake_minimum_required(VERSION 2.8.3)
project(find_my_mate)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  kobuki_msgs
  sensor_msgs
  roslint
  cv_bridge
  geometry_msgs
  behaviortree_cpp_v3
  cameras_cpp
  message_filters
  darknet_ros_msgs
  pcl_conversions
  pcl_ros
  tf
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  navfn
  color_filter
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_filters
  actionlib
  actionlib_msgs
  dialogflow_ros_msgs
  gb_dialog 
  sound_play
  std_msgs
)

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  cv_bridge
  behaviortree_cpp_v3
  cameras_cpp
  geometry_msgs
  message_filters
  darknet_ros_msgs
  tf
  pcl_conversions
  pcl_ros
  image_transport
  message_runtime
  tf2_geometry_msgs
  tf2
  tf2_ros
  actionlib_msgs
  move_base_msgs
  navfn
  color_filter
  behaviortree_cpp_v3 
  color_filter 
  darknet_ros_msgs 
  roscpp
  std_msgs 
  sensor_msgs
  geometry_msgs
  message_filters
#  DEPENDS system_lib
)
find_package(BehaviorTreeV3)
find_package(OpenCV)


find_package(BehaviorTreeV3)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${ZMQ_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/find_my_mate.cpp
# )
#add_library(asr_Starting_bt_node SHARED src/find_my_mate/Starting.cpp)
#target_link_libraries(asr_Starting_bt_node find_my_mate)

#add_library(asr_Description_bt_node SHARED src/find_my_mate/Description.cpp)
#target_link_libraries(asr_Description_bt_node find_my_mate)

#add_library(asr_GetInfo_bt_node SHARED src/find_my_mate/GetInfo.cpp)
#target_link_libraries(asr_GetInfo_bt_node find_my_mate)

#add_library(asr_GoPosition_bt_node SHARED src/find_my_mate/GoPosition.cpp)
#target_link_libraries(asr_GoPosition_bt_node find_my_mate)

#add_library(asr_RotateUPerson_bt_node SHARED src/find_my_mate/RotateUPerson.cpp)
#target_link_libraries(asr_RotateUPerson_bt_node find_my_mate)

#list(APPEND plugin_libs
#  asr_RotateUPerson_bt_node
#  asr_GoPosition_bt_node
#  asr_GetInfo_bt_node
#  asr_Starting_bt_node
#  asr_Description_bt_node
#)

#foreach(bt_plugin ${plugin_libs})
#  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
#  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
#endforeach()

#add_executable(find_my_mate_node 
#  src/find_my_mate_node.cpp
#)

#target_link_libraries(find_my_mate_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/find_my_mate_node.cpp)
#add_executable(startingtest_node src/teststarting.cpp)
add_executable(testRotUPerson_node src/testRotUPerson_node.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(startingtest_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(testRotUPerson_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(startingtest_node ${catkin_LIBRARIES})
target_link_libraries(testRotUPerson_node ${catkin_LIBRARIES})
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

add_library(asr_gposition_bt_node SHARED src/Position/Client_nav.cpp)
add_library(asr_sposition_bt_node SHARED src/Position/SendPoint.cpp)
target_link_libraries(asr_gposition_bt_node )
target_link_libraries(asr_sposition_bt_node )

list(APPEND plugin_libs
  asr_gposition_bt_node
  asr_sposition_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(node_prueba 
	src/nodo_prueba.cpp
)
target_link_libraries(node_prueba ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})
