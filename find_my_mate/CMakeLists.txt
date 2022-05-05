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
