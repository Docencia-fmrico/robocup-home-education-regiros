cmake_minimum_required(VERSION 2.8.3)
project(robocup)

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
)
find_package(BehaviorTreeV3)
find_package(OpenCV)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/person_detected/detect_BBX.cpp
  src/person_detected/person_tf.cpp
)

add_library(asr_movement_bt_node SHARED src/movement/Move.cpp)
add_library(asr_detectedperson_bt_node SHARED src/person_detected/DetectedPerson.cpp)
add_library(asr_sendpoint_bt_node SHARED src/start/SendPoint.cpp)
target_link_libraries(asr_detectedperson_bt_node robocup)
target_link_libraries(asr_movement_bt_node robocup)
target_link_libraries(asr_sendpoint_bt_node robocup)


list(APPEND plugin_libs
  asr_detectedperson_bt_node
  asr_movement_bt_node
  asr_sendpoint_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(node_tree_move 
	src/nodo_tree_move.cpp
)
target_link_libraries(node_tree_move ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(node_first_goal
	src/nodo_first_goal.cpp
)
target_link_libraries(node_first_goal ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(follow_person_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})

#roslint_cpp(
#)