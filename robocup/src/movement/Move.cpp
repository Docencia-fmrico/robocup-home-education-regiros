
// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <string>

#include "robocup_home_education/Move.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace robocup_home_education
{

Move::Move(
  const std::string& name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BTNavAction(name, action_name, config), counter_(0),
  listener(buffer)
{
  Client = n.serviceClient<navfn::MakeNavPlan>("nav_");
}

void
Move::on_halt()
{
  ROS_INFO("Move halt");
}
// /move_base/

void
Move::fillPathRequest(navfn::MakeNavPlanRequest &request)
{
  geometry_msgs::TransformStamped bf2base_msg;
  tf2::Stamped<tf2::Transform> bf2base;
  std::string error;

  if (buffer.canTransform("map", "base_link", ros::Time(0), ros::Duration(0.1), &error))
  {
    bf2base_msg = buffer.lookupTransform("map", "base_link", ros::Time(0));

    tf2::fromMsg(bf2base_msg, bf2base);
    
    ROS_INFO("map -> base [%lf, %lf, %lf]\t%lf ago",
      bf2base.getOrigin().x(),
      bf2base.getOrigin().y(),
      bf2base.getOrigin().z(),
      (ros::Time::now() - bf2base.stamp_).toSec());

    request.start.header.frame_id ="map";
    request.start.pose.position.x = bf2base.getOrigin().x(); // initial position x coordinate Aqui posicion del robot
    request.start.pose.position.y = bf2base.getOrigin().y(); // initial position y coordinate
    request.start.pose.position.z = bf2base.getOrigin().z();
    request.start.pose.orientation.x = bf2base.getRotation().x() ;
    request.start.pose.orientation.y = bf2base.getRotation().x();
    request.start.pose.orientation.z = bf2base.getRotation().x();
    request.start.pose.orientation.w = bf2base.getRotation().x();// directio
  }

  p = getInput<PointTF>("point").value();

  request.goal.header.frame_id = "map" ; 
  request.goal.pose.position.x = p.x; // end point coordinate 
  request.goal.pose.position.y = p.y;
  request.goal.pose.position.y = p.z;
  request.goal.pose.orientation.x = 0.0 ;
  request.goal.pose.orientation.y = 0.0 ;
  request.goal.pose.orientation.z = 0.0 ;
  request.goal.pose.orientation.w = 1.0 ;
}

void
Move::on_start()
{
  //Aqui comienza a rodar el robot
  p = getInput<PointTF>("point").value();


  move_base_msgs::MoveBaseGoal goal;
  std::cerr << "PUNTOS: ["<< p.x << ", " << p.y << ", " << p.z << "]" << std::endl;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = p.x;
  goal.target_pose.pose.position.y = p.y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  set_goal(goal);

  ROS_INFO("Move start");
}


BT::NodeStatus
Move::on_tick()
{
  p = getInput<PointTF>("point").value();

  fillPathRequest(srv.request);
  
  if(Client.call(srv)) {
    ROS_INFO("LLAMADA AL SERVICIO HECHA");
    if(srv.response.plan_found) {
      int sz = srv.response.path.size();
      std::cerr << sz << "-> TAMAÑO" << std::endl;
    } else{
      ROS_INFO("NO SE ENCUENTRA PLAN");
    }
    
  } else {
    ROS_INFO("NO SE LLAMA AL SERVICIO");
  }

  ROS_INFO("COUNTER: %d", counter_);
  
  if (counter_++ == 30)
  {
    ROS_INFO("Move tick");
    std::cerr << "New Goal===========================" << std::endl;
    std::cerr << "PUNTOS: ["<< p.x << ", " << p.y << ", " << p.z << "]" << std::endl;
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = p.x;
    goal.target_pose.pose.position.y = p.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    counter_ = 0;
    set_goal(goal);
    //return BT::NodeStatus::FAILURE; //Para actualizar la ruta 
  } else {
    ROS_INFO("MOVE FAILURE");
  }
  /*if(p.dist < 1.0) {
    return BT::NodeStatus::SUCCESS;
  }*/
  return BT::NodeStatus::RUNNING;
}

void
Move::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

}  // namespace robocup_home_education

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<robocup_home_education::Move>(
        name, "move_base", config);
    };

  factory.registerBuilder<robocup_home_education::Move>(
    "Move", builder);
}
/*
geometry_msgs/PoseStamped start
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/PoseStamped goal
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
---
uint8 plan_found
string error_message
geometry_msgs/PoseStamped[] path
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w

*/

/*move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::TransformStamped bf2base_msg;
  tf2::Stamped<tf2::Transform> bf2base;
  std::string error;
  if (buffer.canTransform("map", "base_link", ros::Time(0), ros::Duration(0.1), &error))
  {
    bf2base_msg = buffer.lookupTransform("map", "base_link", ros::Time(0));

    tf2::fromMsg(bf2base_msg, bf2base);
    
    ROS_INFO("map -> base [%lf, %lf, %lf]\t%lf ago",
      bf2base.getOrigin().x(),
      bf2base.getOrigin().y(),
      bf2base.getOrigin().z(),
      (ros::Time::now() - bf2base.stamp_).toSec());

    ROS_INFO("Move start");
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = bf2base.getOrigin().x();
    goal.target_pose.pose.position.y = bf2base.getOrigin().y();
    goal.target_pose.pose.position.z = bf2base.getOrigin().z();
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    set_goal(goal);
  }
  else
  {
    ROS_ERROR("%s", error.c_str());
  }*/