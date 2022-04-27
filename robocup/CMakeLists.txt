cmake_minimum_required(VERSION 2.8.3)
project(robocup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

#find_package(ZMQ)
#if(ZMQ_FOUND)
#    message(STATUS "ZeroMQ found.")
#    add_definitions(-DZMQ_FOUND)
#else()
#  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
#endif()

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
)

find_package(OpenCV )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${ZMQ_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



#add_library(robocup_home_education src/robocup_home_education/PIDController.cpp)

#add_library(asr_if_person_bt_node SHARED src/robocup_home_education/IfPerson.cpp)
#target_link_libraries(asr_if_person_bt_node robocup_home_education)

#add_library(asr_process_bt_node SHARED src/robocup_home_education/Process.cpp)
#target_link_libraries(asr_process_bt_node robocup_home_education)

#add_library(asr_moving_bt_node SHARED src/robocup_home_education/Move.cpp)
#target_link_libraries(asr_moving_bt_node robocup_home_education)

#list(APPEND plugin_libs
#  asr_if_person_bt_node
#  asr_moving_bt_node
#  asr_process_bt_node
#)
link_directories(
	${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(nodo_person_tf 
	src/nodo_person_tf.cpp
)

target_link_libraries(nodo_person_tf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(node_person_box 
	src/node_person_box.cpp
)

target_link_libraries(node_person_box ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(node_person_filtered 
	src/nodo_rgbd_filter.cpp
)

target_link_libraries(node_person_filtered ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#foreach(bt_plugin ${plugin_libs})
#  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
#  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
#endforeach()

#add_executable(follow_person_node 
#  src/followperson_node.cpp
#)

#target_link_libraries(follow_person_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})

roslint_cpp(
  src/node_person_tf.cpp
)