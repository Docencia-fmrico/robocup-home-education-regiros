
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
: BTNavAction(name, action_name, config), counter_(0)
{
  Client = n.serviceClient<navfn::MakeNavPlan>("MakeNavPlan", true);
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
  p = getInput<PointTF>("point").value();

  request.start.header.frame_id ="map";
  request.start.pose.position.x = p.x; // initial position x coordinate 
  request.start.pose.position.y = p.y; // initial position y coordinate
  request.start.pose.position.z = p.z;
  request.start.pose.orientation.x = 0.0 ;
  request.start.pose.orientation.y = 0.0 ;
  request.start.pose.orientation.z = 0.0 ;
  request.start.pose.orientation.w = 1.0 ;// directio

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
  move_base_msgs::MoveBaseGoal goal;
  ROS_INFO("Move start");
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -1.000000;//-34.920000, -11.880000
  goal.target_pose.pose.position.y = -11.240000;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  set_goal(goal);
}

BT::NodeStatus
Move::on_tick()
{
  //p = getInput<PointTF>("point").value();

  fillPathRequest(srv.request);

  if(Client.call(srv)) {
    long sz = size(srv.response.plan.poses);
    geometry_msgs::PoseStamped path_ = ;
  }

  ROS_INFO("Move tick");
  
  if (counter_++ == 20)
  {
    std::cerr << "New Goal===========================" << std::endl;
    std::cerr << "PUNTOS: ["<< p.x << ", " << p.y << ", " << p.z << "]" << std::endl;
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.6;
    goal.target_pose.pose.position.y = p.y-0.6;
    goal.target_pose.pose.position.z = p.z;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    set_goal(goal);
  } else {
    ROS_INFO("Move FAILURE");
  }

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