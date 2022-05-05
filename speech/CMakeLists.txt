cmake_minimum_required(VERSION 3.0.2)
project(speech)

add_compile_options(-std=c++17)
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
find_package(BehaviorTreeV3)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)


add_library(asr_Move_bt_node SHARED src/speech/Move.cpp)
target_link_libraries(asr_Move_bt_node)

add_library(asr_AskAge_bt_node SHARED src/speech/AskAge.cpp)
target_link_libraries(asr_AskAge_bt_node)

add_library(asr_AskDrink_bt_node SHARED src/speech/AskDrink.cpp)
target_link_libraries(asr_AskDrink_bt_node)

add_library(asr_AskName_bt_node SHARED src/speech/AskName.cpp)
target_link_libraries(asr_AskName_bt_node)

add_library(asr_DetectSeat_bt_node SHARED src/speech/DetectSeat.cpp)
target_link_libraries(asr_DetectSeat_bt_node)

add_library(asr_OfferSeat_bt_node SHARED src/speech/OfferSeat.cpp)
target_link_libraries(asr_OfferSeat_bt_node)

add_library(asr_SayInfo_bt_node SHARED src/speech/SayInfo.cpp)
target_link_libraries(asr_SayInfo_bt_node)

add_library(asr_Start_bt_node SHARED src/speech/Start.cpp)
target_link_libraries(asr_Start_bt_node)


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

add_executable(speech_node src/speech_node.cpp src/speech/Chat.cpp)

target_link_libraries(speech_node
  ${catkin_LIBRARIES}
  ${ZMQ_LIBRARIES}
)

roslint_cpp(
 src/speech_node.cpp
 include/speech/AskAge.h src/speech/AskAge.cpp
 include/speech/AskDrink.h src/speech/AskDrink.cpp
 include/speech/AskName.h src/speech/AskName.cpp
 include/speech/Chat.h src/speech/Chat.cpp
 include/speech/DetectSeat.h src/speech/DetectSeat.cpp
 include/speech/Move.h src/speech/Move.cpp
 include/speech/OfferSeat.h src/speech/OfferSeat.cpp
 include/speech/SayInfo.h src/speech/SayInfo.cpp
 include/speech/Start.h src/speech/Start.cpp
)