cmake_minimum_required(VERSION 3.0.2)
project(speech)

add_compile_options(-std=c++17)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  dialogflow_ros_msgs
  gb_dialog
  roscpp
  roslint
  sound_play
  std_msgs
  std_srvs
  behaviortree_cpp_v3
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs dialogflow_ros_msgs gb_dialog roscpp roslint sound_play std_msgs std_srvs behaviortree_cpp_v3
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(asr_Move_bt_node SHARED src/speech/Move.cpp)
add_library(asr_AskAge_bt_node SHARED src/speech/AskAge.cpp)
add_library(asr_AskDrink_bt_node SHARED src/speech/AskDrink.cpp)
add_library(asr_AskName_bt_node SHARED src/speech/AskName.cpp)
add_library(asr_DetectSeat_bt_node SHARED src/speech/DetectSeat.cpp)
add_library(asr_OfferSeat_bt_node SHARED src/speech/OfferSeat.cpp)
add_library(asr_SayInfo_bt_node SHARED src/speech/SayInfo.cpp)
add_library(asr_Start_bt_node SHARED src/speech/Start.cpp)

list(APPEND plugin_libs
  asr_Move_bt_node
  asr_AskAge_bt_node
  asr_AskDrink_bt_node
  asr_AskName_bt_node
  asr_DetectSeat_bt_node
  asr_OfferSeat_bt_node
  asr_SayInfo_bt_node
  asr_Start_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(speech_node src/speech_node.cpp)

target_link_libraries(speech_node
  ${catkin_LIBRARIES}
  ${ZMQ_LIBRARIES}
)
