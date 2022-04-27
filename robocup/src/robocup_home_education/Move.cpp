
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
}

void
Move::on_halt()
{
  ROS_INFO("Move halt");  //Se realiza el halt
}

void
Move::on_start()
{
  move_base_msgs::MoveBaseGoal goal;
  ROS_INFO("Move start");
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0;
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
  ROS_INFO("Move tick");
  person = getInput<robocup_home_education::objectinimage>("person").value();
  
  /*if (counter_++ == 20) //Poner condicion de si la persona no ha sido detectada
  if (!person.detected)
  {
    std::cerr << "Looking for person" << std::endl;
    //Hacer que de vueltas

  } else */
  if (counter_++ == 20)
  {
    counter_ = 0;
    std::cerr << "New Goal===========================" << std::endl;
    //std::cerr << "Informacion de la persona: "<< person.depth << " Profundidad; "<< person.detected << " Deteccion" << std::endl;
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = person.depth; //person.depth
    goal.target_pose.pose.position.y = person.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    set_goal(goal);
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
