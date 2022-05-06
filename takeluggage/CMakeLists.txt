cmake_minimum_required(VERSION 3.0.2)
project(takeluggage)

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
  dialogflow_ros_msgs
  gb_dialog 
  sound_play
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
  dialogflow_ros_msgs
  gb_dialog 
  sound_play
)
find_package(BehaviorTreeV3)
find_package(OpenCV)


include_directories(
  include
  ${ZMQ_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})



add_library(asr_goposition_take_bt_node SHARED src/Move/Client_nav.cpp)
add_library(asr_gorefere_take_bt_node SHARED src/Move/SendReferee.cpp)
add_library(asr_start_take_bt_node SHARED src/speech/Start.cpp)
add_library(asr_DetectLuggage_take_bt_node SHARED src/speech/DetectLuggage.cpp)
add_library(asr_WaitForOrder_take_bt_node SHARED src/speech/WaitForOrder.cpp)
add_library(asr_Say_take_bt_node SHARED src/speech/Say.cpp)


list(APPEND plugin_libs
  asr_goposition_take_bt_node
  asr_gorefere_take_bt_node
  asr_start_take_bt_node
  asr_DetectLuggage_take_bt_node
  asr_WaitForOrder_take_bt_node
  asr_Say_take_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(node_take_luggage
	src/node_bt_take.cpp src/speech/Chat.cpp
)
target_link_libraries(node_take_luggage ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})
