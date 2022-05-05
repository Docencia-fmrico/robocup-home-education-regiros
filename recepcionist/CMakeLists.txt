cmake_minimum_required(VERSION 3.0.2)
project(recepcionist)

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



add_library(asr_goposition_bt_node SHARED src/Move/Client_nav.cpp)
add_library(asr_sendentry_bt_node SHARED src/Move/SendEntry.cpp)
add_library(asr_sendsit_bt_node SHARED src/Move/SendSit.cpp)
add_library(asr_gorefere_bt_node SHARED src/Move/SendReferee.cpp)

add_library(asr_askage_bt_node SHARED src/speech/AskAge.cpp)
add_library(asr_askdrink_bt_node SHARED src/speech/AskDrink.cpp)
add_library(asr_askname_bt_node SHARED src/speech/AskName.cpp)
add_library(asr_detectseat_bt_node SHARED src/speech/DetectSeat.cpp)
add_library(asr_offerseat_bt_node SHARED src/speech/OfferSeat.cpp)
add_library(asr_sayinfo_bt_node SHARED src/speech/SayInfo.cpp)
add_library(asr_start_bt_node SHARED src/speech/Start.cpp)


list(APPEND plugin_libs
asr_goposition_bt_node
asr_sendsit_bt_node
asr_sendentry_bt_node
asr_gorefere_bt_node
asr_askage_bt_node
asr_askdrink_bt_node
asr_askname_bt_node
asr_detectseat_bt_node
asr_offerseat_bt_node
asr_sayinfo_bt_node
asr_start_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(node_recepcionist 
	src/node_bt_recept.cpp src/speech/Chat.cpp
)
target_link_libraries(node_recepcionist ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})

roslint_cpp(
 src/speech_node.cpp
 include/recepcionist/AskAge.h src/speech/AskAge.cpp
 include/recepcionist/AskDrink.h src/speech/AskDrink.cpp
 include/recepcionist/AskName.h src/speech/AskName.cpp
 include/recepcionist/Chat.h src/speech/Chat.cpp
 include/recepcionist/DetectSeat.h src/speech/DetectSeat.cpp
 include/recepcionist/OfferSeat.h src/speech/OfferSeat.cpp
 include/recepcionist/SayInfo.h src/speech/SayInfo.cpp
 include/recepcionist/Start.h src/speech/Start.cpp
)